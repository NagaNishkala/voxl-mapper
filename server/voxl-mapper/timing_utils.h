#ifndef TIMING_UTILS_H_
#define TIMING_UTILS_H_

#include <stdint.h>

int64_t rc_nanos_monotonic_time();
int loop_sleep(double rate_hz, int64_t* next_time);
void nanosleep_for(int64_t ns);



#endif