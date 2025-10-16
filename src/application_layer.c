// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 512

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

    if (llopen(llayer) == -1) {
        printf("Error establishing connection\n");
        return;
    }

    if (llrole == LlTx) {
        FILE *file = fopen(filename, "rb");
        if(!file) {
            perror("fopen (tx)");
            llclose();
            return;
        }

        unsigned char buf[BUFFER_SIZE];
        size_t n;

        while((n = fread(buf, 1, sizeof buf, file)) > 0) {
            if(llwrite(buf, n) < 0) {
                printf("Error writing\n");
                break;
            }
        }

        fclose(file);

    }
    else {
        FILE *file = fopen(filename, "wb");
        if(!file) {
            perror("fopen (rx)");
            llclose();
            return;
        }

        unsigned char buf[BUFFER_SIZE];

        int got;
        while ((got = llread(buf)) > 0) {
            if (fwrite(buf, 1, got, file) != got) {
                perror("fwrite");
                break;
            }
        }

        if (got < 0) {
            printf("llread: error receiving block\n");
        }

        fclose(file);
    }

    if(llclose() == 0) {
        fprintf(stderr, "Previous opened connection is now closed.\n");
    }
    else {
        fprintf(stderr, "Could not close previous opened connection.\n");
    }
}
