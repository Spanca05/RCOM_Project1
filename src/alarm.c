// Alarm example using sigaction.
// This example shows how to configure an alarm using the sigaction function.
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]
//              Rui Prior [rcprior@fc.up.pt]

#define _POSIX_SOURCE 1 // POSIX compliant source
#include "alarm.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define FALSE 0
#define TRUE 1

volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d received\n", alarmCount);
}

void setupAlarm()
{
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }
}

void startAlarm(unsigned seconds)
{
    alarmEnabled = TRUE;
    alarm(seconds);
}

void stopAlarm(void)
{
    alarm(0);
    alarmEnabled = FALSE;
}
