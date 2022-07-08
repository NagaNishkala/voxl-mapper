#include "timing_utils.h"
#include <time.h>
#include <errno.h>
#include <stdio.h>

int64_t rc_nanos_monotonic_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((int64_t)ts.tv_sec * 1000000000) + ts.tv_nsec;
}

int loop_sleep(double rate_hz)
{
    // static variable so we remember when we last woke up
    static int64_t next_time = 0;

    int64_t current_time = rc_nanos_monotonic_time();

    if (next_time <= 0)
        next_time = current_time;

    // try to maintain output data rate
    next_time += (1000000000.0 / rate_hz);

    // uh oh, we fell behind, warn and get back on track
    if (next_time <= current_time)
    {
        fprintf(stderr, "WARNING loop_sleep fell behind\n");
        return -1;
    }

    nanosleep_for(next_time - current_time);
    return 0;
}

void nanosleep_for(int64_t ns)
{
    struct timespec req, rem;
    req.tv_sec = ns / 1000000000;
    req.tv_nsec = ns % 1000000000;
    // loop untill nanosleep sets an error or finishes successfully
    errno = 0; // reset errno to avoid false detection
    while (nanosleep(&req, &rem) && errno == EINTR)
    {
        req.tv_sec = rem.tv_sec;
        req.tv_nsec = rem.tv_nsec;
    }
    return;
}