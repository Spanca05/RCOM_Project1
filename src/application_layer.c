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
        FILE *file = fopen(filename, "wb");
        if (!file)
        {
            perror("fopen (rx)");
            llclose();
            return;
        }

        unsigned char buf[BUF_SIZE];

        int got;
        while ((got = llread(buf)) > 0)
        {
            if (fwrite(buf, 1, got, file) != got)
            {
                perror("fwrite");
                break;
            }
        }

        if (got < 0)
        {
            printf("llread: error receiving block\n");
        }

        fclose(file);
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
