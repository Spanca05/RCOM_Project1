# Serial Port File Transfer — Data-Link Protocol in C

Reliable file transfer between two computers over an RS-232 serial port, using a data-link protocol implemented from scratch. Developed for the Computer Networks (RCOM) course at FEUP (University of Porto), 2025/2026.

## How it works
- **Application layer:** splits the file into data packets, with control packets carrying the file name and size at the start and end of the transfer
- **Link layer** (`llopen`, `llwrite`, `llread`, `llclose`):
  - framing with flag delimiters and **byte stuffing**
  - header (BCC1) and data (BCC2) **error detection**
  - **Stop-and-Wait ARQ** with alternating sequence numbers and RR/REJ acknowledgements
  - **timeouts and retransmission** using alarms, with a configurable number of tries
  - a **state machine** to parse incoming frames
- **Serial port layer:** port configuration and byte-level reads and writes

## Testing
A virtual cable (based on `socat`) connects two virtual serial ports and can inject noise and disconnections, to check that the protocol recovers from errors.

## Build and run
```bash
make                # build the application and the virtual cable
sudo make run_cable # start the virtual cable (requires socat)
make run_rx         # terminal 2: receiver
make run_tx         # terminal 3: transmitter
make check_files    # check the received file matches the original
```

## Authors
Vasco Vieira · Leonardo Carvalho
