#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define UART_PORT "/dev/ttyUSB0" // Change this to match your radar port
#define BAUDRATE 921600          // Radar typically uses 921600 baud
#define UART_BUFFER_SIZE 4096

const uint8_t MAGIC_WORD[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

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
    uint32_t type;
    uint32_t length;
} tlvHeader;

typedef struct
{
    uint32_t tid;
    float posX;
    float posY;
    float posZ;
    float velX;
    float velY;
    float velZ;
} trackedObj;

int find_magic_word(const uint8_t *buffer, int length)
{
    for (int i = 0; i <= length - 8; i++)
    {
        if (memcmp(&buffer[i], MAGIC_WORD, 8) == 0)
        {
            return i;
        }
    }
    return -1; // not fonud
}

uint32_t parse_uint32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
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

void parse_tracked_obj(const uint8_t *payload, uint32_t payload_len)
{
    if (payload_len < 4)
        return;
    uint16_t numTracks = payload[0] | (payload[1] << 8);
    printf("Tracked objects: %u\n", numTracks);

    const uint8_t *ptr = payload + 4;

    for (uint16_t i = 0; i < numTracks; i++)
    {
        if ((ptr + sizeof(trackedObj)) > payload + payload_len)
            break;

        trackedObj obj;
        memcpy(&obj, ptr, sizeof(trackedObj));

        printf("ID %u: pos (%.2f, %.2f, %.2f), vel (%.2f, %.2f, %.2f)\n",
               obj.tid, obj.posX, obj.posY, obj.posZ, obj.velX, obj.velY, obj.velZ);

        ptr += sizeof(trackedObj);
    }
}

void parse_tlv(const uint8_t *buffer, int numTLVs, int offset, int total_len)
{
    for (int i = 0; i < numTLVs; i++)
    {
        if (offset + sizeof(tlvHeader) > total_len)
            break;

        tlvHeader *tlv = (tlvHeader *)&buffer[offset];
        uint8_t *payload = (uint8_t *)&buffer[offset + sizeof(tlvHeader)];
        uint32_t payload_len = tlv->length - sizeof(tlvHeader);

        if (offset + tlv->length > total_len)
            break;

        switch (tlv->type)
        {
        case 7:
            printf("TLV %d — Type: %u, Length: %u\n", i, tlv->type, tlv->length);
            parse_tracked_obj(payload, payload_len);
            break;
        default:
            printf("TLV %d — Type: %u (unknown)\n", i, tlv->type);
            break;
        }

        offset += tlv->length;
    }
}

// void parse_tlv(const uint8_t *buffer, int numTLVs, int offset)
// {
//     for (int i = 0; i < numTLVs; i++)
//     {
//         tlvHeader *tlv = (tlvHeader *)&buffer[offset];
//         uint8_t *payload = &buffer[offset + sizeof(tlvHeader)];
//         uint32_t payload_len = tlv->length - sizeof(tlvHeader);

//         switch (tlv->type)
//         {
//         case 7:
//             printf("TLV %d — Type: %u, Length: %u\n", i, tlv->type, tlv->length);
//             parse_tracked_obj(payload, payload_len);
//             break;
//         default:
//             break;
//         }

//         offset += tlv->length;
//     }
// }

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

    // Set baud rate
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

int main()
{
    uint8_t uart_buffer[UART_BUFFER_SIZE];
    int uart_fd = setup_uart(UART_PORT);
    if (uart_fd < 0)
        return 1;

    printf("Listening on %s...\n", UART_PORT);

    size_t buffer_len = 0;

    while (1)
    {
        // Read into buffer
        ssize_t n = read(uart_fd, uart_buffer + buffer_len, UART_BUFFER_SIZE - buffer_len);
        if (n > 0)
        {
            buffer_len += n;

            // Look for magic word
            int idx = find_magic_word(uart_buffer, buffer_len);
            if (idx >= 0 && buffer_len >= idx + 40)
            {
                mmwHeader header;
                parse_header(&uart_buffer[idx + 8], &header);

                // If full frame is received
                if (buffer_len >= idx + header.totalPacketLen)
                {
                    printf("\n---- Frame %u ----\n", header.frameNumber);
                    parse_tlv(uart_buffer, header.numTLVs, idx + 8 + sizeof(mmwHeader), buffer_len);

                    // Remove parsed data from buffer
                    size_t remaining = buffer_len - (idx + header.totalPacketLen);
                    memmove(uart_buffer, uart_buffer + idx + header.totalPacketLen, remaining);
                    buffer_len = remaining;
                }
            }

            // Prevent overflow
            if (buffer_len >= UART_BUFFER_SIZE - 512)
            {
                buffer_len = 0; // flush buffer if nothing parsable
                printf("Warning: UART buffer flushed\n");
            }
        }
        else if (n < 0)
        {
            perror("UART read error");
            break;
        }

        // Sleep optional if needed
        usleep(1000);
    }

    close(uart_fd);
    return 0;
}
