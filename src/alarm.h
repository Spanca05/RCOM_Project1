#ifndef ALARM_H
#define ALARM_H

extern volatile int alarmEnabled;
extern volatile int alarmCount;

void setupAlarm();
void startAlarm(unsigned seconds);
void stopAlarm();

#endif