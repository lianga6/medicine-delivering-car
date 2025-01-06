#ifndef __PID_LOCATION_H
#define __PID_LOCATION_H

extern unsigned char location_control_count;
extern uint16_t delay_time2;

void car_spinctl(void);
void car_location(int32_t location_cm);
float location1_pid_control(void);
float location2_pid_control(void);
#endif
