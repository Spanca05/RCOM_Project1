// Link layer protocol implementation

#include <stdio.h>

#include "link_layer.h"
#include "serial_port.h"
#include "alarm.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define FLAG 0x7E
#define ADDRESS_SENDER 0x03
#define ADDRESS_RECEIVER 0x01
#define SET 0x03
#define UA 0x07
#define RR0 0xAA
#define RR1 0xAB
#define REJ0 0x54
#define REJ1 0x55
#define DISC 0x0B
#define BCC1(a, c) ((a) ^ (c))

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} State;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // TODO: Implement this function
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) == -1)
    {
        fprintf(stderr, "Could not open serial port. Try again.\n");
        return -1;
    }

    if (connectionParameters.role == LlTx)
    {
        setupAlarm();

        unsigned char setFrame[5] = {0};

        setFrame[0] = FLAG;
        setFrame[1] = ADDRESS_SENDER;
        setFrame[2] = SET;
        setFrame[3] = BCC1(setFrame[1], setFrame[2]);
        setFrame[4] = FLAG;

        int tries = 0;
        int gotUA = 0;

        while (!gotUA && tries < connectionParameters.nRetransmissions)
        {
            if (writeBytesSerialPort(setFrame, 5) != 5) {
                printf("write SET failed\n");
            }

            startAlarm(connectionParameters.timeout);

            State state = START;

            while (state != STOP && alarmEnabled)
            {
                unsigned char byte;
                int r = readByteSerialPort(&byte);
                
                if (r < 0) continue;
                if (r == 0) continue;

                printf("Byte received: 0x%02x\n", byte);

                switch (state)
                {
                case START:
                    if (byte == FLAG)
                        state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == ADDRESS_RECEIVER)
                        state = A_RCV;
                    else if (byte != FLAG)
                        state = START;
                    break;

                case A_RCV:
                    if (byte == UA)
                        state = C_RCV;
                    else if (byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;

                case C_RCV:
                    if (byte == BCC1(ADDRESS_RECEIVER, UA))
                        state = BCC_OK;
                    else
                        state = START;
                    break;

                case BCC_OK:
                    if (byte == FLAG)
                        state = STOP;
                    else
                        state = START;
                    break;

                default:
                    break;
                }
            }

            if (state == STOP)
            {
                gotUA = 1;
                stopAlarm();
            }
            else
                tries++;
        }

        if(!gotUA) {
            printf("llopen (Tx): UA not received after %d retries\n", connectionParameters.nRetransmissions);
            closeSerialPort();
            return -1;
        }

        return 0;
    }
    else
    {
        State state = START;

        while (state != STOP)
        {
            unsigned char byte;
            int r = readByteSerialPort(&byte);
            
            if(r < 0) continue;
            if (r == 0) continue;

            printf("Byte received: 0x%02X\n", byte);

            switch (state)
            {
            case START:
                if (byte == FLAG)
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == ADDRESS_SENDER)
                    state = A_RCV;
                else if (byte != FLAG)
                    state = START;
                break;

            case A_RCV:
                if (byte == SET)
                    state = C_RCV;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                break;

            case C_RCV:
                if (byte == BCC1(ADDRESS_SENDER, SET))
                    state = BCC_OK;
                else
                    state = START;
                break;

            case BCC_OK:
                if (byte == FLAG)
                    state = STOP;
                else
                    state = START;
                break;

            default:
                break;
            }
        }


        unsigned char ua[5];

        ua[0] = FLAG;
        ua[1] = ADDRESS_RECEIVER;
        ua[2] = UA;
        ua[3] = BCC1(ADDRESS_RECEIVER, UA);
        ua[4] = FLAG;

        if (writeBytesSerialPort(ua, 5) != 5)
        {
            printf("Erro ao enviar UA\n");
            closeSerialPort();
            return -1;
        }

        return 0;
    }

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
