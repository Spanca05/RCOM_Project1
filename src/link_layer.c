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

// Track whether the last returned byte from unStuffByteFromSerial
// was produced from an ESC-escaped sequence (i.e., data), so that
// receivers can distinguish a data 0x7E from a real frame FLAG 0x7E.
static int g_lastFromEscape = 0;

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
/* static int unStuffFunc(const unsigned char *inBuffer, size_t inLen, unsigned char *outBuffer, size_t outCap)
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
} */

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
        // This output byte originated from an escape sequence
        g_lastFromEscape = 1;
        return 1;
    }

    if (rx == ESC)
    {
        state->escPending = 1;
        // No output yet, retain lastFromEscape unchanged
        return 0;
    }

    // If rx == FLAG, it's still FLAG
    *out = rx;
    // Direct byte from the line (not from escape)
    g_lastFromEscape = 0;
    return 1;
}

void resetUnStuffState(UnStuffState *state)
{
    state->escPending = 0;
    g_lastFromEscape = 0;
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

    printf("llwrite called with bufSize=%d, s_ns=%d\n", bufSize, s_ns);

    const unsigned char controlField = (s_ns == 0) ? 0x00 : 0x80;
    const unsigned char addressField = ADDRESS_SENDER;
    const unsigned char bcc1 = BCC1(addressField, controlField);
    const unsigned char bcc2 = bcc2Calc(buf, bufSize);

    printf("Calculated BCC2=0x%02X for %d bytes\n", bcc2, bufSize);

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
    UnStuffState unStuffState;
    resetUnStuffState(&unStuffState);

    State state = START;
    unsigned char rxByte;
    unsigned char lastA = 0, lastC = 0;
    unsigned char dataBuf[MAX_PAYLOAD_SIZE + 10]; // Extra space for BCC2 and margin
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

        case BCC2_OK:
            if (rxByte == FLAG)
                state = STOP;
            else
                state = START;
            break;

        case STOP:
            // We shouldn't reach this state in the switch
            // as we handle STOP conditions before entering the switch
            state = START;
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
            lastC = rxByte;
            if (rxByte == FLAG)
                state = FLAG_RCV;
            else if (rxByte == 0x00 || rxByte == 0x80 || rxByte == DISC) // Information frame or DISC
                state = C_RCV;
            else
                state = START;
            break;

        case C_RCV:
            if (rxByte == BCC1(lastA, lastC))
            {
                if (lastC == DISC)
                {
                    // Handle disconnection separately
                    state = BCC1_OK;
                }
                else if (lastC == 0x00 || lastC == 0x80) // Information frame
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

        case BCC1_OK:
            if (rxByte == FLAG)
            {
                if (lastC == DISC)
                {
                    // Valid DISC frame received
                    // Send DISC response
                    unsigned char responseFrame[5] = {FLAG};
                    size_t responseLen = 1;
                    stuffFunc(ADDRESS_RECEIVER, responseFrame, &responseLen, sizeof(responseFrame));
                    stuffFunc(DISC, responseFrame, &responseLen, sizeof(responseFrame));
                    stuffFunc(BCC1(ADDRESS_RECEIVER, DISC), responseFrame, &responseLen, sizeof(responseFrame));
                    responseFrame[responseLen++] = FLAG;
                    writeBytesSerialPort(responseFrame, responseLen);
                    return -1; // Signal disconnection to upper layer
                }
            }
            state = START;
            break;

        case D_RCV:
            // Only consider FLAG as end-of-frame if it's a real flag from the line,
            // not a data byte produced by an ESC sequence (stuffed 0x7E)
            if (rxByte == FLAG && g_lastFromEscape == 0)
            {
                if (dataIndex < 1) // Need at least BCC2
                {
                    printf("Frame too short, dataIndex=%zu\n", dataIndex);
                    state = START;
                    break;
                }

                // Last byte is BCC2
                unsigned char receivedBcc2 = dataBuf[dataIndex - 1];
                unsigned char calculatedBcc2 = bcc2Calc(dataBuf, dataIndex - 1);

                // Create response frame
                unsigned char responseFrame[8]; // Increased size for safety
                size_t responseLen = 0;

                // Get frame sequence number
                unsigned char ns = (lastC == 0x00) ? 0 : 1;

                printf("Received I-frame: ns=%d, expected_ns=%d, dataIndex=%zu, BCC2: recv=0x%02X calc=0x%02X\n",
                       ns, expected_ns, dataIndex, receivedBcc2, calculatedBcc2);

                // Debug: print first few bytes of data
                printf("First 10 data bytes: ");
                for (size_t i = 0; i < (dataIndex < 10 ? dataIndex : 10); i++)
                {
                    printf("%02X ", dataBuf[i]);
                }
                printf("\n");

                if (receivedBcc2 == calculatedBcc2 && ns == expected_ns)
                {
                    // Valid frame and correct sequence number
                    memcpy(packet, dataBuf, dataIndex - 1);

                    // Send RR for next frame
                    expected_ns ^= 1; // Toggle expected sequence number
                    unsigned char rr = (expected_ns == 0) ? RR0 : RR1;

                    // Build RR frame
                    responseFrame[responseLen++] = FLAG;
                    stuffFunc(ADDRESS_RECEIVER, responseFrame, &responseLen, sizeof(responseFrame));
                    stuffFunc(rr, responseFrame, &responseLen, sizeof(responseFrame));
                    stuffFunc(BCC1(ADDRESS_RECEIVER, rr), responseFrame, &responseLen, sizeof(responseFrame));
                    responseFrame[responseLen++] = FLAG;

                    // Send RR
                    printf("Sending RR(%d), frame length=%zu\n", expected_ns, responseLen);
                    printf("RR frame bytes: ");
                    for (size_t i = 0; i < responseLen; i++)
                    {
                        printf("%02X ", responseFrame[i]);
                    }
                    printf("\n");
                    int written = writeBytesSerialPort(responseFrame, responseLen);
                    printf("Bytes written: %d\n", written);

                    return dataIndex - 1; // Return payload size (excluding BCC2)
                }
                else
                {
                    // Either BCC2 error or wrong sequence number - reject frame
                    unsigned char rej = (expected_ns == 0) ? REJ0 : REJ1;

                    // Build REJ frame
                    responseFrame[responseLen++] = FLAG;
                    stuffFunc(ADDRESS_RECEIVER, responseFrame, &responseLen, sizeof(responseFrame));
                    stuffFunc(rej, responseFrame, &responseLen, sizeof(responseFrame));
                    stuffFunc(BCC1(ADDRESS_RECEIVER, rej), responseFrame, &responseLen, sizeof(responseFrame));
                    responseFrame[responseLen++] = FLAG;

                    // Send REJ
                    printf("Sending REJ(%d), frame length=%zu (BCC2 error or duplicate)\n", expected_ns, responseLen);
                    printf("REJ frame bytes: ");
                    for (size_t i = 0; i < responseLen; i++)
                    {
                        printf("%02X ", responseFrame[i]);
                    }
                    printf("\n");
                    int written = writeBytesSerialPort(responseFrame, responseLen);
                    printf("Bytes written: %d\n", written);
                }

                state = START;
                break;
            }
            else
            {
                // Regular data byte
                if (dataIndex < sizeof(dataBuf))
                    dataBuf[dataIndex++] = rxByte;
                else
                {
                    // Buffer overflow - discard frame
                    printf("WARNING: Buffer overflow in D_RCV, dataIndex=%zu\n", dataIndex);
                    state = START;
                }
            }
            break;
        }
    }
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
