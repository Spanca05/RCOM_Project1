// Application layer protocol implementation

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "application_layer.h"
#include "link_layer.h"

long int getFileSize(FILE *file)
{
    long int size;
    long int current = ftell(file);
    fseek(file, 0, SEEK_END);
    size = ftell(file);
    fseek(file, current, SEEK_SET);
    return size;
}

unsigned char *getData(FILE *file, long int fileLength)
{
    unsigned char *content = (unsigned char *)malloc(sizeof(unsigned char) * fileLength);
    fread(content, sizeof(unsigned char), fileLength, file);
    return content;
}

unsigned char *getControlPacket(int controlField, const char *filename, long int fileSize, size_t *packetSize)
{
    size_t index = 0;
    size_t size = 1 + (1 + 1 + sizeof(long)) + (1 + 1 + strlen(filename));

    unsigned char *packet = malloc(size);
    if (packet == NULL)
    {
        printf("Error: Insufficient memory to allocate %zu bytes\n", size);
        return NULL;
    }

    packet[index++] = controlField;

    // Parameter size
    packet[index++] = 0;
    packet[index++] = sizeof(long);
    memcpy(&packet[index], &fileSize, sizeof(long));
    index += sizeof(long);

    // Parameter filename
    packet[index++] = 1;
    packet[index++] = strlen(filename);
    memcpy(&packet[index], filename, strlen(filename));
    index += strlen(filename);

    *packetSize = size;
    return packet;
}

unsigned char *getDataPacket(unsigned char *buf, const size_t bufSize, size_t *packetSize)
{
    size_t index = 0;
    size_t size = 1 + 2 + bufSize;

    unsigned char *packet = malloc(size);
    if (packet == NULL)
    {
        printf("Error: Insufficient memory to allocate %zu bytes\n", size);
        return NULL;
    }

    packet[index++] = 2;

    packet[index++] = (bufSize >> 8) & 0xFF; // L2: byte mais significativo
    packet[index++] = bufSize & 0xFF;        // L1: byte menos significativo

    memcpy(&packet[index], buf, bufSize);
    index += bufSize;

    *packetSize = size;
    return packet;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole llrole = (strcmp(role, "tx") == 0) ? LlTx : LlRx;

    LinkLayer llayer;
    strncpy(llayer.serialPort, serialPort, sizeof(llayer.serialPort) - 1);
    llayer.serialPort[sizeof(llayer.serialPort) - 1] = '\0';
    llayer.role = llrole;
    llayer.baudRate = baudRate;
    llayer.nRetransmissions = nTries;
    llayer.timeout = timeout;

    if (llopen(llayer) == -1)
    {
        printf("Error establishing connection\n");
        return;
    }

    if (llrole == LlTx)
    {
        FILE *file = fopen(filename, "rb");
        if (!file)
        {
            perror("fopen (tx)");
            llclose();
            return;
        }

        long int fileSize = getFileSize(file);
        unsigned char *content = getData(file, fileSize);

        // Send START Control Packet
        size_t startControlPacketSize;
        unsigned char *startControlPacket = getControlPacket(1, filename, fileSize, &startControlPacketSize);
        llwrite(startControlPacket, (int)startControlPacketSize);
        free(startControlPacket);

        unsigned char buf[MAX_PAYLOAD_SIZE];
        size_t size;
        unsigned char *dataPacket;
        size_t dataPacketSize;
        long int bytesLeft = fileSize;

        unsigned char *contentPtr = content;

        while (bytesLeft > 0)
        {
            size = (bytesLeft > MAX_PAYLOAD_SIZE) ? MAX_PAYLOAD_SIZE : (size_t)bytesLeft;

            memcpy(buf, contentPtr, size);

            contentPtr += size;
            bytesLeft -= (long)size;

            dataPacket = getDataPacket(buf, size, &dataPacketSize);

            llwrite(dataPacket, (int)dataPacketSize);
            free(dataPacket);
        }

        free(content);

        // Send END Control Packet
        size_t endControlPacketSize;
        unsigned char *endControlPacket = getControlPacket(3, filename, fileSize, &endControlPacketSize);
        llwrite(endControlPacket, (int)endControlPacketSize);
        free(endControlPacket);

        fclose(file);
    }
    else
    {
        /* Receiver: wait for START control packet, then DATA packets, then END*/
        unsigned char packetBuf[MAX_PAYLOAD_SIZE + 32];
        int received;

        FILE *outFile = NULL;
        long expectedFileSize = 0;
        long totalWritten = 0;
        char remoteFilename[256] = {0};

        while (1)
        {
            received = llread(packetBuf);
            if (received < 0)
            {
                fprintf(stderr, "llread error\n");
                if (outFile)
                    fclose(outFile);
                llclose();
                return;
            }

            if (received == 0)
                continue;

            unsigned char type = packetBuf[0];

            if (type == 1)
            {
                /* START control packet: parse TLV parameters */
                size_t idx = 1;
                while (idx + 1 < (size_t)received)
                {
                    unsigned char paramType = packetBuf[idx++];
                    unsigned char paramLen = packetBuf[idx++];
                    if (idx + paramLen > (size_t)received)
                        break; /* malformed */

                    if (paramType == 0)
                    {
                        /* file size (long) */
                        long fsz = 0;
                        if (paramLen == sizeof(long))
                        {
                            memcpy(&fsz, &packetBuf[idx], sizeof(long));
                            expectedFileSize = fsz;
                        }
                    }
                    else if (paramType == 1)
                    {
                        /* filename */
                        size_t nameLen = paramLen;
                        if (nameLen >= sizeof(remoteFilename))
                            nameLen = sizeof(remoteFilename) - 1;
                        memcpy(remoteFilename, &packetBuf[idx], nameLen);
                        remoteFilename[nameLen] = '\0';
                    }

                    idx += paramLen;
                }

                /* open output file (use remote filename if provided, else use provided filename)
                   If sender provided a filename like "penguin.gif", create a local name
                   by appending "-received" before the extension: "penguin-received.gif" */
                char outNameBuf[512] = {0};
                if (remoteFilename[0] != '\0')
                {
                    /* insert "-received" before last dot if present */
                    char *dot = strrchr(remoteFilename, '.');
                    if (dot != NULL && dot != remoteFilename)
                    {
                        size_t baseLen = (size_t)(dot - remoteFilename);
                        if (baseLen + strlen("-received") + strlen(dot) + 1 <= sizeof(outNameBuf))
                            snprintf(outNameBuf, sizeof(outNameBuf), "%.*s-received%s", (int)baseLen, remoteFilename, dot);
                        else
                            /* fallback to safe copy */
                            strncpy(outNameBuf, remoteFilename, sizeof(outNameBuf) - 1);
                    }
                    else
                    {
                        /* no extension: just append suffix */
                        if (strlen(remoteFilename) + strlen("-received") + 1 <= sizeof(outNameBuf))
                            snprintf(outNameBuf, sizeof(outNameBuf), "%s-received", remoteFilename);
                        else
                            strncpy(outNameBuf, remoteFilename, sizeof(outNameBuf) - 1);
                    }
                }
                else
                {
                    strncpy(outNameBuf, filename, sizeof(outNameBuf) - 1);
                }

                outFile = fopen(outNameBuf, "wb");
                if (!outFile)
                {
                    perror("fopen (rx)");
                    llclose();
                    return;
                }
                totalWritten = 0;
                fprintf(stderr, "Receiving file '%s' size=%ld bytes\n", outNameBuf, expectedFileSize);
            }
            else if (type == 2)
            {
                /* DATA packet: 1 byte type + 2 bytes length + payload */
                if (received < 3)
                    continue; /* malformed */

                unsigned char l2 = packetBuf[1];
                unsigned char l1 = packetBuf[2];
                int dataLen = ((int)l2 << 8) | (int)l1;
                if (dataLen < 0 || dataLen > MAX_PAYLOAD_SIZE)
                    continue; /* invalid */

                if (received < 3 + dataLen)
                    continue; /* truncated */

                if (!outFile)
                {
                    /* Haven't received START; open with provided filename */
                    outFile = fopen(filename, "wb");
                    if (!outFile)
                    {
                        perror("fopen (rx fallback)");
                        llclose();
                        return;
                    }
                }

                size_t written = fwrite(&packetBuf[3], 1, dataLen, outFile);
                fflush(outFile);
                totalWritten += (long)written;
                (void)written;
            }
            else if (type == 3)
            {
                /* END control packet: optionally parse and finish */
                /* Could parse TLV to verify size/filename */
                if (outFile)
                {
                    fclose(outFile);
                    outFile = NULL;
                }
                fprintf(stderr, "File transfer complete, %ld bytes written\n", totalWritten);
                break;
            }
            else
            {
                /* unknown packet type: ignore */
            }
        }
        if(outFile)
            fclose(outFile);
    }

    if (llclose() == 0)
    {
        fprintf(stderr, "Previous opened connection is now closed.\n");
    }
    else
    {
        fprintf(stderr, "Could not close previous opened connection.\n");
    }
}
