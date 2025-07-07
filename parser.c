#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/types.h>

#define UART_PORT "/dev/cu.usbmodemR00810384" // CHANGE THIS
#define BAUDRATE 921600
#define UART_BUFFER_SIZE 4096
#define MAX_POINTS 100
#define MAX_OBJECTS 100

const uint8_t MAGIC_WORD[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

typedef enum
{
    TLV_DETECTED_POINTS = 1,
    TLV_POINT_CLOUD = 1020,
    TLV_OBJ_LIST = 1010,
    TLV_INDEX = 1011,
    TLV_HEIGHT = 1012,
    TLV_PRESENCE = 1021
} tlvType;

typedef struct
{
    uint32_t version;
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
} mmwHeader;

typedef struct
{
    float elevationUnit;
    float azimuthUnit;
    float dopplerUnit;
    float rangeUnit;
    float snrUnit;
} pointUnit;

typedef struct
{
    int8_t elevation;
    int8_t azimuth;
    int16_t doppler;
    int16_t range;
    int16_t snr;
} pointObj;

typedef struct __attribute__((__packed__))
{
    uint32_t tid;
    float posX, posY, posZ;
    float velX, velY, velZ;
    float accX, accY, accZ;
    float ec[16];
    float g;
    float confidenceLevel;
} listTlv;

typedef struct
{
    uint8_t targetID;
} indexTlv;

typedef struct
{
    uint32_t present;
} presenceTlv;

typedef struct
{
    uint8_t targetID;
    float maxZ;
    float minZ;
} heightTlv;

typedef struct
{
    mmwHeader header;
    pointObj points[MAX_POINTS];
    pointUnit units;
    int numPoints;
    listTlv objects[MAX_OBJECTS];
    int numObjects;
    indexTlv indices[MAX_POINTS];
    int numIndices;
    heightTlv heights[MAX_OBJECTS];
    int numHeights;
    presenceTlv presence;
    int hasPresence;
} radarFrame;

uint32_t parse_uint32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

float parse_float_le(const uint8_t *data)
{
    uint32_t int_val = parse_uint32_le(data);
    return *(float *)&int_val;
}

int16_t parse_int16_le(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0]) | ((uint16_t)data[1] << 8));
}

void parse_header(const uint8_t *buffer, mmwHeader *header)
{
    header->version = parse_uint32_le(&buffer[0]);
    header->totalPacketLen = parse_uint32_le(&buffer[4]);
    header->platform = parse_uint32_le(&buffer[8]);
    header->frameNumber = parse_uint32_le(&buffer[12]);
    header->timeCpuCycles = parse_uint32_le(&buffer[16]);
    header->numDetectedObj = parse_uint32_le(&buffer[20]);
    header->numTLVs = parse_uint32_le(&buffer[24]);
    header->subFrameNumber = parse_uint32_le(&buffer[28]);
}

void parse_tlv(const uint8_t *buffer, int numTLVs, int offset, int total_len, radarFrame *frame)
{
    for (int i = 0; i < numTLVs; i++)
    {
        if (offset + 8 > total_len)
        {
            printf("[ERROR] TLV header extends beyond buffer\n");
            return;
        }

        uint32_t type = parse_uint32_le(&buffer[offset]);
        uint32_t length = parse_uint32_le(&buffer[offset + 4]);
        offset += 8;

        if (offset + length > total_len)
        {
            printf("[ERROR] TLV payload extends beyond buffer (type=%u, len=%u)\n", type, length);
            return;
        }

        const uint8_t *payload = &buffer[offset];
        printf("TLV TYPE: %d, LENGTH: %d\n", type, length);

        switch (type)
        {
        case TLV_POINT_CLOUD:
            if (length < 20)
            {
                printf("[ERROR] Point cloud TLV too short (len = %u)\n", length);
                break;
            }

            // Parse compression units (5 floats)
            frame->units.elevationUnit = parse_float_le(&payload[0]);
            frame->units.azimuthUnit = parse_float_le(&payload[4]);
            frame->units.dopplerUnit = parse_float_le(&payload[8]);
            frame->units.rangeUnit = parse_float_le(&payload[12]);
            frame->units.snrUnit = parse_float_le(&payload[16]);

            printf("[DEBUG] Units: elev=%.6f, azim=%.6f, doppler=%.6f, range=%.6f, snr=%.6f\n",
                   frame->units.elevationUnit, frame->units.azimuthUnit,
                   frame->units.dopplerUnit, frame->units.rangeUnit, frame->units.snrUnit);

            // Parse compressed points
            const uint8_t *point_data = payload + 20;
            int point_payload_len = length - 20;
            int numPoints = point_payload_len / 8; // Each point is 8 bytes

            if (numPoints > MAX_POINTS)
            {
                printf("[WARNING] Truncating %d points to %d\n", numPoints, MAX_POINTS);
                numPoints = MAX_POINTS;
            }

            frame->numPoints = numPoints;

            // Parse each compressed point
            for (int p = 0; p < numPoints; p++)
            {
                const uint8_t *pt_data = point_data + (p * 8);
                frame->points[p].elevation = (int8_t)pt_data[0];
                frame->points[p].azimuth = (int8_t)pt_data[1];
                frame->points[p].doppler = parse_int16_le(&pt_data[2]);
                frame->points[p].range = parse_int16_le(&pt_data[4]);
                frame->points[p].snr = parse_int16_le(&pt_data[6]);
            }

            printf("[INFO] Parsed %d compressed points from TLV 1020\n", numPoints);
            break;

        case TLV_DETECTED_POINTS:
            // Legacy format - points are not compressed
            frame->numPoints = length / sizeof(pointObj);
            if (frame->numPoints > MAX_POINTS)
                frame->numPoints = MAX_POINTS;
            memcpy(frame->points, payload, frame->numPoints * sizeof(pointObj));
            printf("[INFO] Parsed %d uncompressed points from TLV 1\n", frame->numPoints);
            break;

        case TLV_OBJ_LIST:
            frame->numObjects = length / sizeof(listTlv);
            if (frame->numObjects > MAX_OBJECTS)
                frame->numObjects = MAX_OBJECTS;
            memcpy(frame->objects, payload, frame->numObjects * sizeof(listTlv));
            printf("[INFO] Parsed %d objects from TLV 1010\n", frame->numObjects);
            break;

        case TLV_INDEX:
            frame->numIndices = length / sizeof(indexTlv);
            if (frame->numIndices > MAX_POINTS)
                frame->numIndices = MAX_POINTS;
            memcpy(frame->indices, payload, frame->numIndices * sizeof(indexTlv));
            printf("[INFO] Parsed %d indices from TLV 1011\n", frame->numIndices);
            break;

        case TLV_HEIGHT:
            frame->numHeights = length / sizeof(heightTlv);
            if (frame->numHeights > MAX_OBJECTS)
                frame->numHeights = MAX_OBJECTS;
            memcpy(frame->heights, payload, frame->numHeights * sizeof(heightTlv));
            printf("[INFO] Parsed %d height estimates from TLV 1012\n", frame->numHeights);
            break;

        case TLV_PRESENCE:
            if (length >= sizeof(presenceTlv))
            {
                frame->presence.present = parse_uint32_le(payload);
                frame->hasPresence = 1;
                printf("[INFO] Parsed presence indication: %s\n",
                       frame->presence.present ? "PRESENT" : "NOT PRESENT");
            }
            break;

        default:
            printf("[WARNING] Unknown TLV type: %u (length: %u)\n", type, length);
            break;
        }
        offset += length;
    }

    // Print decompressed point cloud data
    if (frame->numPoints > 0)
    {
        printf("\n--- Point Cloud Data (first 5 points) ---\n");
        for (int i = 0; i < frame->numPoints && i < 5; i++)
        {
            float elevation = frame->points[i].elevation * frame->units.elevationUnit;
            float azimuth = frame->points[i].azimuth * frame->units.azimuthUnit;
            float doppler = frame->points[i].doppler * frame->units.dopplerUnit;
            float range = frame->points[i].range * frame->units.rangeUnit;
            float snr = frame->points[i].snr * frame->units.snrUnit;

            printf("Point %d: elev=%.3f, azim=%.3f, doppler=%.3f, range=%.3f, snr=%.3f\n",
                   i, elevation, azimuth, doppler, range, snr);
        }
    }

    // Print tracked objects
    if (frame->numObjects > 0)
    {
        printf("\n--- Tracked Objects ---\n");
        for (int i = 0; i < frame->numObjects; i++)
        {
            printf("Track %d: TID=%u, pos=(%.2f, %.2f, %.2f), vel=(%.2f, %.2f, %.2f), conf=%.3f\n",
                   i, frame->objects[i].tid,
                   frame->objects[i].posX, frame->objects[i].posY, frame->objects[i].posZ,
                   frame->objects[i].velX, frame->objects[i].velY, frame->objects[i].velZ,
                   frame->objects[i].confidenceLevel);
        }
    }
}

int find_magic_word(const uint8_t *buffer, int length)
{
    for (int i = 0; i <= length - 8; i++)
    {
        if (memcmp(&buffer[i], MAGIC_WORD, 8) == 0)
            return i;
    }
    return -1;
}

int setup_uart(const char *port_name)
{
    int fd = open(port_name, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror("UART open");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, 921600);
    cfsetospeed(&options, 921600);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CREAD | CLOCAL);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void write_frame_to_file(const radarFrame *frame)
{
    // Ensure /data directory exists
    mkdir("data", 0777);

    char filepath[256];
    snprintf(filepath, sizeof(filepath), "data/frame_%u.txt", frame->header.frameNumber);

    FILE *file = fopen(filepath, "w");
    if (!file)
    {
        perror("Failed to open frame metadata file");
        return;
    }

    const mmwHeader *h = &frame->header;

    fprintf(file, "---- Frame %u ----\n", h->frameNumber);
    fprintf(file, "Version: %u\n", h->version);
    fprintf(file, "Total Packet Length: %u\n", h->totalPacketLen);
    fprintf(file, "Platform: %u\n", h->platform);
    fprintf(file, "Time CPU Cycles: %u\n", h->timeCpuCycles);
    fprintf(file, "Number of Detected Objects: %u\n", h->numDetectedObj);
    fprintf(file, "Number of TLVs: %u\n", h->numTLVs);
    fprintf(file, "Number of Points: %d\n", frame->numPoints);
    fprintf(file, "Number of Objects: %d\n", frame->numObjects);
    fprintf(file, "Subframe Number: %u\n", h->subFrameNumber);
    if (frame->hasPresence)
    {
        fprintf(file, "Presence: %s\n", frame->presence.present ? "DETECTED" : "NOT DETECTED");
    }
    fclose(file);

    if (frame->numPoints > 0)
    {
        char pointPath[256];
        snprintf(pointPath, sizeof(pointPath), "data/frame_%u_points.csv", h->frameNumber);
        FILE *pointFile = fopen(pointPath, "w");
        if (!pointFile)
        {
            perror("Failed to open point cloud CSV");
            return;
        }

        fprintf(pointFile, "elevation,azimuth,doppler,range,snr\n");
        for (int i = 0; i < frame->numPoints; i++)
        {
            float elevation = frame->points[i].elevation * frame->units.elevationUnit;
            float azimuth = frame->points[i].azimuth * frame->units.azimuthUnit;
            float doppler = frame->points[i].doppler * frame->units.dopplerUnit;
            float range = frame->points[i].range * frame->units.rangeUnit;
            float snr = frame->points[i].snr * frame->units.snrUnit;

            fprintf(pointFile, "%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    elevation, azimuth, doppler, range, snr);
        }
        fclose(pointFile);
    }

    if (frame->numObjects > 0)
    {
        char objPath[256];
        snprintf(objPath, sizeof(objPath), "data/frame_%u_objects.csv", h->frameNumber);
        FILE *objFile = fopen(objPath, "w");
        if (!objFile)
        {
            perror("Failed to open objects CSV");
            return;
        }

        fprintf(objFile, "tid,posX,posY,posZ,velX,velY,velZ,accX,accY,accZ,confidence\n");
        for (int i = 0; i < frame->numObjects; i++)
        {
            const listTlv *obj = &frame->objects[i];
            fprintf(objFile, "%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    obj->tid, obj->posX, obj->posY, obj->posZ,
                    obj->velX, obj->velY, obj->velZ,
                    obj->accX, obj->accY, obj->accZ,
                    obj->confidenceLevel);
        }
        fclose(objFile);
    }
}

int main()
{
    uint8_t buffer[UART_BUFFER_SIZE];
    int uart_fd = setup_uart(UART_PORT);
    if (uart_fd < 0)
        return 1;

    printf("Connected to %s, waiting for radar data...\n", UART_PORT);

    size_t buffer_len = 0;
    while (1)
    {
        ssize_t bytes = read(uart_fd, buffer + buffer_len, UART_BUFFER_SIZE - buffer_len);
        if (bytes <= 0)
            continue;
        buffer_len += bytes;

        int magic_idx = find_magic_word(buffer, buffer_len);
        if (magic_idx >= 0 && buffer_len >= magic_idx + 40)
        {
            mmwHeader header;
            parse_header(&buffer[magic_idx + 8], &header);

            if (buffer_len >= magic_idx + header.totalPacketLen)
            {
                radarFrame frame = {0};
                frame.header = header;
                int tlv_offset = magic_idx + 8 + 32;

                printf("\n=== Processing Frame %u ===\n", header.frameNumber);
                parse_tlv(buffer, header.numTLVs, tlv_offset, magic_idx + header.totalPacketLen, &frame);

                write_frame_to_file(&frame);
                printf("Saved frame_%u.txt with %d points and %d objects\n",
                       frame.header.frameNumber, frame.numPoints, frame.numObjects);

                size_t remaining = buffer_len - (magic_idx + header.totalPacketLen);
                memmove(buffer, buffer + magic_idx + header.totalPacketLen, remaining);
                buffer_len = remaining;
            }
        }

        if (buffer_len > UART_BUFFER_SIZE - 512)
            buffer_len = 0;
        usleep(1000);
    }
    close(uart_fd);
    return 0;
}