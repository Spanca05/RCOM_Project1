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

// Note: whether the last returned byte was produced from an ESC-escaped
// sequence is now stored per-UnStuffState (see link_layer.h). This avoids
// interference when nested state machines call unStuffByteFromSerial.

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
        state->lastFromEscape = 1;
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
    state->lastFromEscape = 0;
    return 1;
}

void resetUnStuffState(UnStuffState *state)
{
    state->escPending = 0;
    state->lastFromEscape = 0;
}

// TODO: Implementar sendSupervisionFrame para código ficar mais modular e se perceber melhor

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    nTries = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    // Allow runtime override via environment for stress tests
    const char *envRetries = getenv("LL_RETRIES");
    if (envRetries && *envRetries)
    {
        char *endp = NULL;
        long v = strtol(envRetries, &endp, 10);
        if (endp != envRetries && v > 0 && v <= 1000)
        {
            nTries = (int)v;
            fprintf(stderr, "[LL] Using LL_RETRIES=%d (overriding default %d)\n", nTries, connectionParameters.nRetransmissions);
        }
    }
    const char *envTimeout = getenv("LL_TIMEOUT");
    if (envTimeout && *envTimeout)
    {
        char *endp = NULL;
        long v = strtol(envTimeout, &endp, 10);
        if (endp != envTimeout && v > 0 && v <= 60)
        {
            timeout = (int)v;
            fprintf(stderr, "[LL] Using LL_TIMEOUT=%d seconds (overriding default %d)\n", timeout, connectionParameters.timeout);
        }
    }

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

        // Supervision frames are NOT stuffed
        unsigned char setFrame[5] = {FLAG, addressField, controlField, bcc1, FLAG};
        size_t setLen = 5;

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

        // Supervision frames are NOT stuffed
        unsigned char uaFrame[5] = {FLAG, addressField, controlField, bcc1, FLAG};
        size_t uaLen = 5;

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
        printf("[llwrite] Attempt %d (attemptCount=%d, nTries=%d)\n", attemptCount + 1, attemptCount, nTries);
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
                // Receiver detected error -> retransmit WITHOUT consuming attempt
                // REJ means communication is working, just data was corrupted
                // Only timeouts should consume retry attempts
                printf("[llwrite] REJ received, retransmitting without consuming retry\n");
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
            // timeout - no response received, consume a retry attempt
            printf("[llwrite] Timeout on attempt %d, incrementing retry counter\n", attemptCount + 1);
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
            {
                printf("BCC1 error: expected 0x%02X, got 0x%02X (A=0x%02X, C=0x%02X)\n",
                       BCC1(lastA, lastC), rxByte, lastA, lastC);
                state = START;
            }
            break;

        case BCC1_OK:
            if (rxByte == FLAG)
            {
                if (lastC == DISC)
                {
                    // Valid DISC frame received - handle the entire disconnect handshake
                    // Send DISC response
                    unsigned char discFrame[5] = {FLAG, ADDRESS_RECEIVER, DISC, BCC1(ADDRESS_RECEIVER, DISC), FLAG};
                    writeBytesSerialPort(discFrame, 5);

                    // Wait for UA from transmitter
                    UnStuffState uaState;
                    resetUnStuffState(&uaState);
                    State uaWaitState = START;
                    unsigned char uaA = 0, uaC = 0;

                    while (uaWaitState != STOP)
                    {
                        unsigned char uaByte;
                        int uaRes = unStuffByteFromSerial(&uaByte, &uaState);
                        if (uaRes <= 0)
                            continue;

                        switch (uaWaitState)
                        {
                        case START:
                            if (uaByte == FLAG)
                                uaWaitState = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (uaByte == ADDRESS_SENDER)
                            {
                                uaA = uaByte;
                                uaWaitState = A_RCV;
                            }
                            else if (uaByte != FLAG)
                                uaWaitState = START;
                            break;
                        case A_RCV:
                            uaC = uaByte;
                            if (uaByte == UA)
                                uaWaitState = C_RCV;
                            else if (uaByte == FLAG)
                                uaWaitState = FLAG_RCV;
                            else
                                uaWaitState = START;
                            break;
                        case C_RCV:
                            if (uaByte == BCC1(uaA, uaC))
                                uaWaitState = BCC1_OK;
                            else if (uaByte == FLAG)
                                uaWaitState = FLAG_RCV;
                            else
                                uaWaitState = START;
                            break;
                        case BCC1_OK:
                            if (uaByte == FLAG)
                                uaWaitState = STOP;
                            else
                                uaWaitState = START;
                            break;
                        default:
                            break;
                        }
                    }

                    return 0; // Return 0 to signal end of transmission (not error)
                }
            }
            state = START;
            break;

        case D_RCV:
            // Only consider FLAG as end-of-frame if it's a real flag from the line,
            // not a data byte produced by an ESC sequence (stuffed 0x7E)
            if (rxByte == FLAG && unStuffState.lastFromEscape == 0)
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

                if (receivedBcc2 == calculatedBcc2)
                {
                    if (ns == expected_ns)
                    {
                        // Valid frame and correct sequence number
                        memcpy(packet, dataBuf, dataIndex - 1);
                        // Send RR for next frame
                        expected_ns ^= 1; // Toggle expected sequence number
                        unsigned char rr = (expected_ns == 0) ? RR0 : RR1;

                        // Supervision frames are NOT stuffed
                        unsigned char responseFrame[5] = {FLAG, ADDRESS_RECEIVER, rr, BCC1(ADDRESS_RECEIVER, rr), FLAG};

                        printf("Sending RR(%d), frame length=5\n", expected_ns);
                        writeBytesSerialPort(responseFrame, 5);

                        return (int)(dataIndex - 1);
                    }
                    else
                    {
                        unsigned char rr_dup = (expected_ns == 0) ? RR0 : RR1;

                        // Supervision frames are NOT stuffed
                        unsigned char responseFrame[5] = {FLAG, ADDRESS_RECEIVER, rr_dup, BCC1(ADDRESS_RECEIVER, rr_dup), FLAG};

                        printf("Dup frame: sending RR(%d)\n", expected_ns);
                        writeBytesSerialPort(responseFrame, 5);

                        state = START;
                        break;
                    }
                }
                else
                {
                    // Either BCC2 error or wrong sequence number - reject frame
                    unsigned char rej = (expected_ns == 0) ? REJ0 : REJ1;

                    // Supervision frames are NOT stuffed
                    unsigned char responseFrame[5] = {FLAG, ADDRESS_RECEIVER, rej, BCC1(ADDRESS_RECEIVER, rej), FLAG};

                    // Send REJ
                    printf("Sending REJ(%d), frame length=5 (BCC2 error or duplicate)\n", expected_ns);
                    printf("REJ frame bytes: ");
                    for (size_t i = 0; i < 5; i++)
                    {
                        printf("%02X ", responseFrame[i]);
                    }
                    printf("\n");
                    int written = writeBytesSerialPort(responseFrame, 5);
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

        // Supervision frames are NOT stuffed
        unsigned char discFrame[5] = {FLAG, addressField, controlField, bcc1, FLAG};
        size_t discLen = 5;

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

        // Send UA final - supervision frames are NOT stuffed
        const unsigned char C_UA = UA;
        const unsigned char B1_UA = BCC1(addressField, C_UA);

        unsigned char uaFrame[5] = {FLAG, addressField, C_UA, B1_UA, FLAG};

        writeBytesSerialPort(uaFrame, 5);

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

        /* Send DISC back - supervision frames are NOT stuffed */
        const unsigned char addressField = ADDRESS_RECEIVER;
        const unsigned char controlField = DISC;
        const unsigned char bcc1 = BCC1(addressField, controlField);

        unsigned char discFrame[5] = {FLAG, addressField, controlField, bcc1, FLAG};
        size_t discLen = 5;

        if (writeBytesSerialPort(discFrame, discLen) != discLen)
        {
            printf("Erro ao enviar DISC\n");
            closeSerialPort();
            return -1;
        }

        UnStuffState unStuffState2;
        setupAlarm();

        int tries2 = 0;
        int gotUA = 0;

        while (!gotUA && tries2 < nTries)
        {
            resetUnStuffState(&unStuffState2);
            startAlarm(timeout);

            State st = START;
            unsigned char a2 = 0, c2 = 0;

            while (st != STOP && alarmEnabled)
            {
                unsigned char b;
                int res = unStuffByteFromSerial(&b, &unStuffState2);
                if (res <= 0)
                    continue;

                switch (st)
                {
                case START:
                    if (b == FLAG)
                        st = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (b == ADDRESS_SENDER)
                    {
                        a2 = b;
                        st = A_RCV;
                    }
                    else if (b != FLAG)
                        st = START;
                    break;
                case A_RCV:
                    c2 = b;
                    if (b == UA)
                        st = C_RCV;
                    else if (b == FLAG)
                        st = FLAG_RCV;
                    else
                        st = START;
                    break;
                case C_RCV:
                    if (b == BCC1(a2, c2))
                        st = BCC1_OK;
                    else if (b == FLAG)
                        st = FLAG_RCV;
                    else
                        st = START;
                    break;
                case BCC1_OK:
                    if (b == FLAG)
                        st = STOP;
                    else
                        st = START;
                    break;
                default:
                    break;
                }
            }

            if (st == STOP && a2 == ADDRESS_SENDER && c2 == UA)
            {
                gotUA = 1;
                stopAlarm();
                break;
            }

            stopAlarm();
            tries2++;

            writeBytesSerialPort(discFrame, discLen);
        }

        if (!gotUA)
        {
            printf("llclose (Rx): UA não recebido após %d tentativas\n", tries2);
            closeSerialPort();
            return -1;
        }

        closeSerialPort();
        return 0;
    }
}
