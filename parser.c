#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "parser.h"

#define UART_BUFFER_SIZE 8192

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