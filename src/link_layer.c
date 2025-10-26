// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "alarm.h"

int nTries = 3;
int timeout = 3;
static unsigned char s_ns = 0;
static LinkLayerRole storedRole = LlTx;

static unsigned char bcc2Calc(const unsigned char *data, int len)
{
    unsigned char bcc2 = 0x00;
    for (int i = 0; i < len; i++)
        bcc2 ^= data[i];
    return bcc2;
}

// Safer stuffing: escreve no buffer de saída mantendo controle de capacidade.
// Retorna o novo outLen (>=0) ou -1 em caso de overflow.
static int stuffFunc(unsigned char inputByte, unsigned char *outBuffer, size_t *outLen, size_t outCap)
{
    if (inputByte == FLAG || inputByte == ESC)
    {
        if ((*outLen) + 2 > outCap)
            return -1;
        outBuffer[(*outLen)++] = 0x7D;
        outBuffer[(*outLen)++] = (unsigned char)(inputByte ^ 0x20); // O XOR retorna um signed char?
    }
    else
    {
        if ((*outLen) + 1 > outCap)
            return -1;
        outBuffer[(*outLen)++] = inputByte;
    }

    return 0;
}

// Un-stuffing: converte sequências escapadas (0x7D 0x5E -> 0x7E, 0x7D 0x5D -> 0x7D)
// in: buffer de entrada (possivelmente com escapes), inLen: tamanho da entrada
// out: buffer destino, outCap: capacidade de out
// Retorna número de bytes escritos em out, ou -1 em caso de erro (escape truncado ou overflow)
static int unStuffFunc(const unsigned char *inBuffer, size_t inLen, unsigned char *outBuffer, size_t outCap)
{
    // TODO: Fazer eu a função e percebê-la
    int outIndex = 0;

    for (int inIndex = 0; inIndex < inLen; inIndex++)
    {
        unsigned char currByte = inBuffer[inIndex];

        if (currByte == ESC)
        {
            // precisa existir um próximo byte
            if (inIndex + 1 >= inLen)
                return -1; // escape truncado

            unsigned char nextByte = inBuffer[++inIndex];
            //! Inverso do stuffing (XOR seguido de XOR cancela-se?)
            currByte = (unsigned char)(nextByte ^ 0x20);
        }

        if (outIndex + 1 > outCap)
            return -1; // overflow
        outBuffer[outIndex++] = currByte;
    }

    return outIndex;
}

static int unStuffByteFromSerial(unsigned char *out, UnStuffState *state)
{
    unsigned char rx;
    int r = readByteSerialPort(&rx);
    if (r <= 0)
        return -1;

    if (state->escPending)
    {
        state->escPending = 0;
        *out = (unsigned char)(rx ^ 0x20);
        return 1;
    }

    if (rx == ESC)
    {
        state->escPending = 1;
        return 0;
    }

    // If rx == FLAG, it's still FLAG
    *out = rx;
    return 1;
}

void resetUnStuffState(UnStuffState *state)
{
    state->escPending = 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    nTries = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    storedRole = connectionParameters.role;

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) == -1)
    {
        fprintf(stderr, "Could not open serial port. Try again.\n");
        return -1;
    }

    if (storedRole == LlTx)
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

        while (!gotUA && tries < nTries)
        {
            if (writeBytesSerialPort(setFrame, 5) != 5)
            {
                printf("write SET failed\n");
            }

            startAlarm(timeout);

            State state = START;

            while (state != STOP && alarmEnabled)
            {
                unsigned char byte;
                int res = readByteSerialPort(&byte);

                if (res < 0)
                    continue;
                if (res == 0)
                    continue;

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

        if (!gotUA)
        {
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

            if (r < 0)
                continue;
            if (r == 0)
                continue;

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
    // Validate input
    if (bufSize <= 0)
        return -1;

    const unsigned char controlField = (s_ns == 0) ? 0x00 : 0x80;
    const unsigned char addressField = ADDRESS_SENDER;
    const unsigned char bcc1 = BCC1(addressField, controlField);
    const unsigned char bcc2 = bcc2Calc(buf, bufSize);

    // Worst case: FLAG + 2 * (A, C, BCC1, payload, BCC2) + FLAG
    const unsigned maxFrameSize = 1 + 2 * (4 + bufSize) + 1;
    unsigned char frame[maxFrameSize];
    int frameLen = 0;

    frame[frameLen++] = FLAG;
    TRY(stuffFunc(addressField, frame, &frameLen, maxFrameSize));
    TRY(stuffFunc(controlField, frame, &frameLen, maxFrameSize));
    TRY(stuffFunc(bcc1, frame, &frameLen, maxFrameSize));

    for (size_t payloadIndex = 0; payloadIndex < bufSize; payloadIndex++)
        TRY(stuffFunc(buf[payloadIndex], frame, &frameLen, maxFrameSize));

    TRY(stuffFunc(bcc2, frame, &frameLen, maxFrameSize));

    if (frameLen + 1 > maxFrameSize)
        return -1;
    frame[frameLen++] = FLAG;

    setupAlarm();
    int attemptCount = 0;

    UnStuffState unStuffState;
    resetUnStuffState(&unStuffState);

    /* follow the same retry semantics as llopen (attemptCount < nTries) */
    while (attemptCount < nTries)
    {
        resetUnStuffState(&unStuffState);
        if (writeBytesSerialPort(frame, frameLen) != frameLen)
        {
            // write failure -> try again without consuming a retry (optional)
            continue;
        }

        startAlarm(timeout);

        // RR confirms the next Ns (Nr = s_ns ^ 1)
        unsigned char rrExpected = (s_ns == 0) ? RR1 : RR0;
        // REJ asks retransmission for current Ns (Nr = s_ns)
        unsigned char rejExpected = (s_ns == 0) ? REJ0 : REJ1;

        // read bytes and run S/U state machine (FLAG A C BCC1 FLAG)
        State state = START;
        unsigned char rxByte = 0;
        unsigned char lastAddress = 0; /* last A read */
        unsigned char lastControl = 0; /* last C read */

        while (alarmEnabled)
        {
            // Destuffing
            unsigned char rxByteDeStuffed;
            int unStuffRes = unStuffByteFromSerial(&rxByteDeStuffed, &unStuffState);
            if (unStuffRes <= 0)
                continue;
            rxByte = rxByteDeStuffed;

            switch (state)
            {
            case START:
                if (rxByte == FLAG)
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (rxByte == ADDRESS_RECEIVER)
                {
                    lastAddress = rxByte;
                    state = A_RCV;
                }
                else if (rxByte != FLAG)
                    state = START;
                break;

            case A_RCV:
                lastControl = rxByte; // may be RR*/REJ*/DISC
                if (rxByte == rrExpected || rxByte == rejExpected || rxByte == DISC)
                    state = C_RCV;
                else if (rxByte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case C_RCV:
                // validate BCC1 = A ^ C
                if (rxByte == BCC1(lastAddress, lastControl))
                    state = BCC_OK;
                else if (rxByte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case BCC_OK:
                if (rxByte == FLAG)
                    state = STOP;
                else if (rxByte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            default:
                break;
            }

            if (state == STOP)
                break;
        }

        stopAlarm();

        if (state == STOP)
        {
            if (lastControl == rrExpected)
            {
                // success -> toggle Ns and return bytes sent
                s_ns ^= 1;
                return bufSize;
            }
            if (lastControl == rejExpected)
            {
                // receiver detected error -> retransmit (consume attempt)
                attemptCount++;
                continue;
            }
            if (lastControl == DISC)
            {
                // receiver closing unexpectedly
                return -1;
            }
            // other control (e.g., UA or unexpected RR): treat as non-useful response
            attemptCount++;
            continue;
        }
        else
        {
            // timeout
            attemptCount++;
            continue;
        }
    }

    // exceeded retransmissions
    return -1;
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
