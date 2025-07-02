#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN64
#elif defined(__linux__)
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#elif defined(__APPLE__)
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

#define UART_PORT "/dev/cu.usbmodemR00810384" // CHANGE THIS
#define BAUDRATE 921600
#define UART_BUFFER_SIZE 4096
#define MAX_POINTS 100
#define MAX_OBJECTS 100

const uint8_t MAGIC_WORD[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

/*
 * @brief uart packet header object
 */
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

// TODO: C23 to support enum underlying typing
// TLV Type:
// 1020 = Point Cloud
// 1010 = Target object list
// 1011 = Target Index
// 1012 = Target Height
// 1021 = Presence Indication
// typedef enum : uint32_t {
//   TLV_DETECTED_POINTS = 1,
//   TLV_POINT_CLOUD = 1020,
//   TLV_OBJ_LIST = 1010,
//   TLV_INDEX = 1011,
//   TLV_PRESENCE = 1021
// } tlvType;

typedef enum
{
  TLV_DETECTED_POINTS = 1,
  TLV_POINT_CLOUD = 1020,
  TLV_OBJ_LIST = 1010,
  TLV_INDEX = 1011,
  TLV_PRESENCE = 1021
} tlvType;

typedef struct
{
  tlvType type;
  uint32_t length;
} tlvHeader;

/*
 * @brief scaling factor for point cloud decompression
 */
typedef struct
{
  float elevationUnit;
  float azimuthUnit;
  float dopplerUnit;
  float rangeUnit;
  float snrUnit;
} pointUnit;

/*
 * @brief compressed point cloud object
 */
typedef struct
{
  int8_t elevation; // radians
  int8_t azimuth;   // radians
  int16_t doppler;  // m/s
  int16_t range;    // m
  int16_t snr;      // ratio
} pointObj;

/*
 * @brief target information object
 */
typedef struct
{
  uint32_t tid;
  float posX;
  float posY;
  float posZ;
  float velX;
  float velY;
  float velZ;
  float accX;
  float accY;
  float accZ;
  float *ec[16];
  float g;
  float confidenceLevel;
} listTlv;

/*
 * @brief unique target ID object
 *
 * 0 - 249: valid
 * 253: point not associated: weak SNR
 * 254: point not associated: outside interest boundary
 * 255: point not considered: noise
 */
typedef struct
{
  uint8_t targetID; // 0 - 249 valid
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
  int numPoints;

  listTlv objects[MAX_OBJECTS];
  int numObjects;

  indexTlv indices[MAX_POINTS];
  int numIndices;

  presenceTlv presence;
  int hasPresence;

  heightTlv heights[MAX_OBJECTS];
  int numHeights;
} radarFrame;

int find_magic_word(const uint8_t *buffer, int length)
{
  for (int i = 0; i <= length - 8; i++)
  {
    if (memcmp(&buffer[i], MAGIC_WORD, 8) == 0)
    {
      return i;
    }
  }
  return -1;
}

uint32_t parse_uint32(const uint8_t *data)
{
  return ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

void parse_header(const uint8_t *buffer, mmwHeader *header)
{
  header->version = parse_uint32(&buffer[0]);
  header->totalPacketLen = parse_uint32(&buffer[4]);
  header->platform = parse_uint32(&buffer[8]);
  header->frameNumber = parse_uint32(&buffer[16]);
  header->timeCpuCycles = parse_uint32(&buffer[20]);
  header->numDetectedObj = parse_uint32(&buffer[24]);
  header->numTLVs = parse_uint32(&buffer[28]);
  header->subFrameNumber = parse_uint32(&buffer[32]);
}

void parse_tlv(const uint8_t *buffer, int numTLVs, int offset, int total_len,
               radarFrame *frame)
{
  for (int i = 0; i < numTLVs; i++)
  {
    if (offset + sizeof(tlvHeader) > total_len)
    {
      printf("TLV header out of bounds\n");
      break;
    }

    tlvHeader *tlv = (tlvHeader *)&buffer[offset];
    uint8_t *payload = (uint8_t *)&buffer[offset + sizeof(tlvHeader)];
    uint32_t payload_len = tlv->length - sizeof(tlvHeader);

    if (offset + tlv->length > total_len)
    {
      printf("TLV payload exceeds buffer\n");
      break;
    }

    switch (tlv->type)
    {
    case TLV_POINT_CLOUD:
      frame->numPoints = payload_len / sizeof(pointObj);
      memcpy(frame->points, payload, frame->numPoints * sizeof(pointObj));
      break;
    case TLV_OBJ_LIST:
      frame->numObjects = payload_len / sizeof(listTlv);
      memcpy(frame->objects, payload, frame->numObjects * sizeof(listTlv));
      break;
    case TLV_INDEX:
      frame->numIndices = payload_len / sizeof(indexTlv);
      memcpy(frame->indices, payload, frame->numIndices * sizeof(indexTlv));
      break;
    case TLV_PRESENCE:
      if (payload_len >= sizeof(presenceTlv))
      {
        memcpy(&frame->presence, payload, sizeof(presenceTlv));
        frame->hasPresence = 1;
      }
      break;
    default:
      printf("TLV %d — Type: %u (unknown)\n", i, tlv->type);
      break;
    }

    offset += tlv->length;
  }
}

int setup_uart(const char *port_name)
{
  int fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
    perror("Error opening UART");
    return -1;
  }

  struct termios options;
  tcgetattr(fd, &options);
  cfsetispeed(&options, BAUDRATE);
  cfsetospeed(&options, BAUDRATE);

  // 8N1, no flow control
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag |= CREAD | CLOCAL;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
  options.c_iflag &= ~(IXON | IXOFF | IXANY);         // no software flow
  options.c_oflag &= ~OPOST;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 1; // 100ms read timeout

  tcsetattr(fd, TCSANOW, &options);
  return fd;
}

void write_frame_to_file(const radarFrame *frame)
{
  char filepath[256];
  snprintf(filepath, sizeof(filepath), "data/frame_%u.txt", frame->header.frameNumber);

  FILE *file = fopen(filepath, "w");
  if (!file)
  {
    perror("Failed to write frame file");
    return;
  }

  const mmwHeader *header = &frame->header;

  // Write header
  fprintf(file, "---- Frame %u ----\n", header->frameNumber);
  fprintf(file, "Version: %u\n", header->version);
  fprintf(file, "Total Packet Length: %u\n", header->totalPacketLen);
  fprintf(file, "Platform: %u\n", header->platform);
  fprintf(file, "Frame Number: %u\n", header->frameNumber);
  fprintf(file, "Time CPU Cycles: %u\n", header->timeCpuCycles);
  fprintf(file, "Number of Detected Objects: %u\n", header->numDetectedObj);
  fprintf(file, "Number of TLVs: %u\n", header->numTLVs);
  fprintf(file, "Subframe Number: %u\n\n", header->subFrameNumber);

  // Write TLV data
  if (frame->numPoints > 0)
  {
    fprintf(file, "TLV 1020 - POINT_CLOUD: %d points\n", frame->numPoints);
    for (int i = 0; i < frame->numPoints; i++)
    {
      pointObj pt = frame->points[i];
      fprintf(file,
              "  Point %d: Elev: %d, Azim: %d, Doppler: %d, Range: %d, SNR: %d\n",
              i, pt.elevation, pt.azimuth, pt.doppler, pt.range, pt.snr);
    }
    fprintf(file, "\n");
  }

  if (frame->numObjects > 0)
  {
    fprintf(file, "TLV 1010 - OBJECT_LIST: %d objects\n", frame->numObjects);
    for (int i = 0; i < frame->numObjects; i++)
    {
      listTlv obj = frame->objects[i];
      fprintf(file,
              "  Object %d: TID: %u, Pos: (%.2f, %.2f, %.2f), Vel: (%.2f, %.2f, %.2f), "
              "Acc: (%.2f, %.2f, %.2f), G: %.2f, Conf: %.2f\n",
              i, obj.tid, obj.posX, obj.posY, obj.posZ,
              obj.velX, obj.velY, obj.velZ,
              obj.accX, obj.accY, obj.accZ,
              obj.g, obj.confidenceLevel);
    }
    fprintf(file, "\n");
  }

  if (frame->numIndices > 0)
  {
    fprintf(file, "TLV 1011 - INDEX: %d indices\n", frame->numIndices);
    for (int i = 0; i < frame->numIndices; i++)
    {
      fprintf(file, "  Index %d: Target ID: %u\n", i, frame->indices[i].targetID);
    }
    fprintf(file, "\n");
  }

  if (frame->hasPresence)
  {
    fprintf(file, "TLV 1021 - PRESENCE: %s\n\n", frame->presence.present ? "Detected" : "Not Detected");
  }

  if (frame->numHeights > 0)
  {
    fprintf(file, "TLV 1012 - HEIGHT: %d entries\n", frame->numHeights);
    for (int i = 0; i < frame->numHeights; i++)
    {
      heightTlv h = frame->heights[i];
      fprintf(file, "  Height %d: Target ID: %u, MinZ: %.2f, MaxZ: %.2f\n",
              i, h.targetID, h.minZ, h.maxZ);
    }
    fprintf(file, "\n");
  }

  fclose(file);
}

int main()
{
  uint8_t uart_buffer[UART_BUFFER_SIZE];
  size_t buffer_len = 0;

  int uart_fd = setup_uart(UART_PORT);
  if (uart_fd < 0)
  {
    return 1;
  }

  printf("Listening on %s...\n", UART_PORT);

  // Ensure data directory exists
  struct stat st = {0};
  if (stat("data", &st) == -1)
  {
    mkdir("data", 0755); // Create directory with rwxr-xr-x permissions
  }

  while (1)
  {
    // Read UART data into buffer
    ssize_t bytes_read = read(uart_fd, uart_buffer + buffer_len, UART_BUFFER_SIZE - buffer_len);
    if (bytes_read < 0)
    {
      perror("UART read error");
      break;
    }
    buffer_len += bytes_read;

    int magic_idx = find_magic_word(uart_buffer, buffer_len);
    if (magic_idx >= 0 && buffer_len >= magic_idx + 40)
    {
      mmwHeader header;
      parse_header(&uart_buffer[magic_idx + 8], &header);

      if (buffer_len >= magic_idx + header.totalPacketLen)
      {
        radarFrame frame = {0}; // zero initialize entire struct
        frame.header = header;

        printf("\n---- Frame %u ----\n", header.frameNumber);

        int tlv_offset = magic_idx + 8 + sizeof(mmwHeader);
        parse_tlv(uart_buffer, header.numTLVs, tlv_offset, buffer_len, &frame);

        // TODO: store frame or write to file here
        char filepath[256];
        snprintf(filepath, sizeof(filepath), "data/frame_%u.txt", header.frameNumber);
        write_frame_to_file(&frame);

        // Remove parsed data from buffer
        size_t remaining = buffer_len - (magic_idx + header.totalPacketLen);
        memmove(uart_buffer, uart_buffer + magic_idx + header.totalPacketLen, remaining);
        buffer_len = remaining;
      }
    }

    if (buffer_len >= UART_BUFFER_SIZE - 512) // reset buffer to prevent overflow
    {
      printf("Warning: UART buffer flushed (too large)\n");
      buffer_len = 0;
    }
    usleep(1000); // throttling
  }

  close(uart_fd);
  return 0;
}
