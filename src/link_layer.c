// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "alarm.h"

int nTries = 3;
int timeout = 3;
static unsigned char s_ns = 0;
static unsigned char expected_ns = 0;
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
        outBuffer[(*outLen)++] = ESC;
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

// TODO: Implementar sendSupervisionFrame para código ficar mais modular e se perceber melhor

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
        const unsigned char addressField = ADDRESS_SENDER;
        const unsigned char controlField = SET;
        const unsigned char bcc1 = BCC1(addressField, controlField);

        unsigned char setFrame[8];
        size_t setLen = 0;

        setFrame[setLen++] = FLAG;
        TRY(stuffFunc(addressField, setFrame, &setLen, sizeof(setFrame)));
        TRY(stuffFunc(controlField, setFrame, &setLen, sizeof(setFrame)));
        TRY(stuffFunc(bcc1, setFrame, &setLen, sizeof(setFrame)));
        setFrame[setLen++] = FLAG;

        int tries = 0;
        int gotUA = 0;

        UnStuffState unStuffState;
        setupAlarm();

        while (!gotUA && tries < nTries)
        {
            if (writeBytesSerialPort(setFrame, setLen) != setLen)
            {
                printf("write SET failed\n");
                tries++;
                continue;
            }

            resetUnStuffState(&unStuffState);
            startAlarm(timeout);

            State state = START;
            unsigned char lastA = 0, lastC = 0;

            while (state != STOP && alarmEnabled)
            {
                unsigned char byte;
                int res = unStuffByteFromSerial(&byte, &unStuffState);

                if (res <= 0)
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
                    {
                        lastA = byte;
                        state = A_RCV;
                    }
                    else if (byte != FLAG)
                        state = START;
                    break;

                case A_RCV:
                    lastC = byte;
                    if (byte == UA)
                        state = C_RCV;
                    else if (byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;

                case C_RCV:
                    if (byte == BCC1(lastA, lastC))
                        state = BCC1_OK;
                    else
                        state = START;
                    break;

                case BCC1_OK:
                    if (byte == FLAG)
                        state = STOP;
                    else
                        state = START;
                    break;

                default:
                    break;
                }
            }

            if (state == STOP && lastA == ADDRESS_RECEIVER && lastC == UA)
            {
                gotUA = 1;
                stopAlarm();
                break;
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
        UnStuffState unStuffState;
        resetUnStuffState(&unStuffState);

        State state = START;
        unsigned char lastA = 0, lastC = 0;

        while (state != STOP)
        {
            unsigned char byte;
            int res = unStuffByteFromSerial(&byte, &unStuffState);

            if (res <= 0)
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
                {
                    lastA = byte;
                    state = A_RCV;
                }
                else if (byte != FLAG)
                    state = START;
                break;

            case A_RCV:
                lastC = byte;
                if (byte == SET)
                    state = C_RCV;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case C_RCV:
                if (byte == BCC1(lastA, lastC))
                    state = BCC1_OK;
                else
                    state = START;
                break;

            case BCC1_OK:
                if (byte == FLAG)
                    state = STOP;
                else
                    state = START;
                break;

            default:
                break;
            }
        }

        const unsigned char addressField = ADDRESS_RECEIVER;
        const unsigned char controlField = UA;
        const unsigned char bcc1 = BCC1(addressField, controlField);

        unsigned char uaFrame[8];
        size_t uaLen = 0;

        uaFrame[uaLen++] = FLAG;
        TRY(stuffFunc(addressField, uaFrame, &uaLen, sizeof(uaFrame)));
        TRY(stuffFunc(controlField, uaFrame, &uaLen, sizeof(uaFrame)));
        TRY(stuffFunc(bcc1, uaFrame, &uaLen, sizeof(uaFrame)));
        uaFrame[uaLen++] = FLAG;

        if (writeBytesSerialPort(uaFrame, uaLen) != uaLen)
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
    size_t frameLen = 0;

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
                    state = BCC1_OK;
                else if (rxByte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case BCC1_OK:
                if (rxByte == FLAG)
                    state = STOP;
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
    // TODO: Ver se está correto
    UnStuffState unStuffState;
    resetUnStuffState(&unStuffState);

    State state = START;
    unsigned char rxByte;
    unsigned char lastA = 0, lastC = 0;
    unsigned char dataBuf[MAX_PAYLOAD_SIZE + 1]; // payload + bcc2
    size_t dataIndex = 0;

    while (1)
    {
        int r = unStuffByteFromSerial(&rxByte, &unStuffState);
        if (r <= 0)
            continue;

        switch (state)
        {
        case START:
            if (rxByte == FLAG)
                state = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (rxByte == ADDRESS_SENDER)
            {
                lastA = rxByte;
                state = A_RCV;
            }
            else if (rxByte != FLAG)
                state = START;
            break;

        case A_RCV:
            lastC = rxByte; // control
            state = C_RCV;
            break;

        case C_RCV:
            if (rxByte == BCC1(lastA, lastC))
            {
                /* I-frame? control 0x00 or 0x80 */
                if (lastC == 0x00 || lastC == 0x80)
                {
                    dataIndex = 0;
                    state = D_RCV;
                }
                else
                {
                    /* supervisory - ignore for llread */
                    state = START;
                }
            }
            else if (rxByte == FLAG)
                state = FLAG_RCV;
            else
                state = START;
            break;

        case D_RCV:
            if (rxByte == FLAG)
            {
                /* end of I-frame: last byte is BCC2 */
                if (dataIndex < 1)
                {
                    state = FLAG_RCV;
                    break;
                }

                unsigned char recv_bcc2 = dataBuf[dataIndex - 1];
                size_t payloadLen = dataIndex - 1;
                unsigned char calc_bcc2 = bcc2Calc(dataBuf, (int)payloadLen);
                unsigned char ns = (lastC >> 7) & 0x01;

                if (calc_bcc2 == recv_bcc2 && ns == expected_ns)
                {
                    /* good frame */
                    if (payloadLen > MAX_PAYLOAD_SIZE)
                        return -1;

                    memcpy(packet, dataBuf, payloadLen);
                    expected_ns ^= 1; /* accept -> expect next */

                    /* send RR acknowledging next expected (Nr = expected_ns) */
                    unsigned char rrToSend = (expected_ns == 0) ? RR0 : RR1;
                    unsigned char rrFrame[8];
                    size_t rrLen = 0;
                    rrFrame[rrLen++] = FLAG;
                    TRY(stuffFunc(ADDRESS_RECEIVER, rrFrame, &rrLen, sizeof(rrFrame)));
                    TRY(stuffFunc(rrToSend, rrFrame, &rrLen, sizeof(rrFrame)));
                    TRY(stuffFunc(BCC1(ADDRESS_RECEIVER, rrToSend), rrFrame, &rrLen, sizeof(rrFrame)));
                    rrFrame[rrLen++] = FLAG;
                    if (writeBytesSerialPort(rrFrame, rrLen) != rrLen)
                        return -1;

                    return (int)payloadLen;
                }
                else if (calc_bcc2 != recv_bcc2)
                {
                    /* BCC2 error: request retransmission (REJ for current expected_ns) */
                    unsigned char rejToSend = (expected_ns == 0) ? REJ0 : REJ1;
                    unsigned char rejFrame[8];
                    size_t rejLen = 0;
                    rejFrame[rejLen++] = FLAG;
                    TRY(stuffFunc(ADDRESS_RECEIVER, rejFrame, &rejLen, sizeof(rejFrame)));
                    TRY(stuffFunc(rejToSend, rejFrame, &rejLen, sizeof(rejFrame)));
                    TRY(stuffFunc(BCC1(ADDRESS_RECEIVER, rejToSend), rejFrame, &rejLen, sizeof(rejFrame)));
                    rejFrame[rejLen++] = FLAG;
                    if (writeBytesSerialPort(rejFrame, rejLen) != rejLen)
                        return -1;

                    state = START; /* wait for retransmission */
                }
                else
                {
                    /* duplicate frame (ns != expected_ns): re-ack with RR(expected_ns) */
                    unsigned char rrToSend = (expected_ns == 0) ? RR0 : RR1;
                    unsigned char rrFrame[8];
                    size_t rrLen = 0;
                    rrFrame[rrLen++] = FLAG;
                    TRY(stuffFunc(ADDRESS_RECEIVER, rrFrame, &rrLen, sizeof(rrFrame)));
                    TRY(stuffFunc(rrToSend, rrFrame, &rrLen, sizeof(rrFrame)));
                    TRY(stuffFunc(BCC1(ADDRESS_RECEIVER, rrToSend), rrFrame, &rrLen, sizeof(rrFrame)));
                    rrFrame[rrLen++] = FLAG;
                    if (writeBytesSerialPort(rrFrame, rrLen) != rrLen)
                        return -1;

                    state = START;
                }
            }
            else
            {
                /* regular data byte */
                if (dataIndex + 1 > sizeof(dataBuf))
                {
                    /* overflow: discard frame */
                    state = START;
                    break;
                }
                dataBuf[dataIndex++] = rxByte;
            }
            break;

        default:
            break;
        }
    }

    /* unreachable */
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    if (storedRole == LlTx)
    {
        const unsigned char addressField = ADDRESS_SENDER;
        const unsigned char controlField = DISC;
        const unsigned char bcc1 = BCC1(addressField, controlField);

        unsigned char discFrame[8];
        size_t discLen = 0;
        discFrame[discLen++] = FLAG;
        TRY(stuffFunc(addressField, discFrame, &discLen, sizeof(discFrame)));
        TRY(stuffFunc(controlField, discFrame, &discLen, sizeof(discFrame)));
        TRY(stuffFunc(bcc1, discFrame, &discLen, sizeof(discFrame)));
        discFrame[discLen++] = FLAG;

        int tries = 0;
        int gotDISC = 0;

        UnStuffState unStuffState;
        setupAlarm();

        while (!gotDISC && tries < nTries)
        {
            if (writeBytesSerialPort(discFrame, discLen) != discLen)
            {
                tries++;
                continue;
            }

            resetUnStuffState(&unStuffState);
            startAlarm(timeout);

            State state = START;
            unsigned char lastA = 0, lastC = 0;

            while (state != STOP && alarmEnabled)
            {
                unsigned char byte;
                int res = unStuffByteFromSerial(&byte, &unStuffState);
                if (res <= 0)
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
                    {
                        lastA = byte;
                        state = A_RCV;
                    }
                    else if (byte != FLAG)
                        state = START;
                    break;

                case A_RCV:
                    lastC = byte; //! Porque é que é aqui e não dentro do if?
                    if (byte == DISC)
                        state = C_RCV;
                    else if (byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;

                case C_RCV:
                    if (byte == BCC1(lastA, lastC))
                        state = BCC1_OK;
                    else
                        state = START;
                    break;

                case BCC1_OK:
                    if (byte == FLAG)
                        state = STOP;
                    else
                        state = START;
                    break;

                default:
                    break;
                }
            }

            if (state == STOP && lastA == ADDRESS_RECEIVER && lastC == DISC)
            {
                gotDISC = 1;
                stopAlarm();
                break;
            }

            tries++;
        }

        if (!gotDISC)
        {
            printf("llclose (Tx): DISC not received after %d retries\n", tries);
            closeSerialPort();
            return -1;
        }

        // Envia UA final (stuffed)
        const unsigned char C_UA = UA;
        const unsigned char B1_UA = BCC1(addressField, C_UA);

        unsigned char uaFrame[8];
        size_t uaLen = 0;

        uaFrame[uaLen++] = FLAG;
        TRY(stuffFunc(addressField, uaFrame, &uaLen, sizeof(uaFrame)));
        TRY(stuffFunc(C_UA, uaFrame, &uaLen, sizeof(uaFrame)));
        TRY(stuffFunc(B1_UA, uaFrame, &uaLen, sizeof(uaFrame)));
        uaFrame[uaLen++] = FLAG;

        writeBytesSerialPort(uaFrame, uaLen);

        closeSerialPort();
        return 0;
    }
    else
    {
        UnStuffState unStuffState;
        resetUnStuffState(&unStuffState);

        State state = START;
        unsigned char lastA = 0, lastC = 0;

        while (state != STOP)
        {
            unsigned char byte;
            int res = unStuffByteFromSerial(&byte, &unStuffState);

            if (res <= 0)
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
                {
                    lastA = byte;
                    state = A_RCV;
                }
                else if (byte != FLAG)
                    state = START;
                break;

            case A_RCV:
                lastC = byte;
                if (byte == DISC)
                    state = C_RCV;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case C_RCV:
                if (byte == BCC1(lastA, lastC))
                    state = BCC1_OK;
                else
                    state = START;
                break;

            case BCC1_OK:
                if (byte == FLAG)
                    state = STOP;
                else
                    state = START;
                break;

            default:
                break;
            }
        }

        /* If we didn't receive a proper DISC from transmitter, bail out */
        if (!(lastA == ADDRESS_SENDER && lastC == DISC))
        {
            closeSerialPort();
            return -1;
        }

        /* Send DISC back (stuffed), using ADDRESS_RECEIVER */
        const unsigned char addressField = ADDRESS_RECEIVER;
        const unsigned char controlField = DISC;
        const unsigned char bcc1 = BCC1(addressField, controlField);

        unsigned char discFrame[8];
        size_t discLen = 0;

        discFrame[discLen++] = FLAG;
        TRY(stuffFunc(addressField, discFrame, &discLen, sizeof(discFrame)));
        TRY(stuffFunc(controlField, discFrame, &discLen, sizeof(discFrame)));
        TRY(stuffFunc(bcc1, discFrame, &discLen, sizeof(discFrame)));
        discFrame[discLen++] = FLAG;

        if (writeBytesSerialPort(discFrame, discLen) != discLen)
        {
            printf("Erro ao enviar UA\n");
            closeSerialPort();
            return -1;
        }

        return 0;
    }
    return 0;
}
