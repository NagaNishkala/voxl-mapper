#ifndef TIMER_H
#define TIMER_H

#include <time.h>
#include <unordered_map>

class Timer
{
public:
    void start(std::string timer_name)
    {
        TimeStatistics &cur = named_timers_[timer_name];

        if (cur.running)
        {
            fprintf(stderr, "TIMER ERROR: Timer already started\n");
            return;
        }

        cur.start_time = rc_nanos_monotonic_time();
        cur.running = true;
    }

    void stop(std::string timer_name)
    {
        if (!checkExists(timer_name))
        {
            fprintf(stderr, "TIMER ERROR: No such timer exists to stop\n");
            return;
        }

        TimeStatistics &cur = named_timers_[timer_name];

        uint64_t diff = rc_nanos_monotonic_time() - cur.start_time;

        cur.call_count += 1;
        cur.total_time += diff;

        cur.max_time = diff > cur.max_time ? diff : cur.max_time;
        cur.min_time = diff < cur.min_time ? diff : cur.min_time;

        cur.running = false;
    }

    void stopAll(bool reset_timers = false)
    {
        for (const std::pair<std::string, TimeStatistics> &it : named_timers_)
        {
            stop(it.first);

            if (reset_timers)
                reset(it.first);
        }
    }

    void reset(std::string timer_name)
    {
        if (!checkExists(timer_name))
        {
            fprintf(stderr, "TIMER ERROR: No such timer exists to stop\n");
            return;
        }

        TimeStatistics &cur = named_timers_[timer_name];

        cur = TimeStatistics();
    }

    void printTimerStats(std::string timer_name, bool print_header = true)
    {
        if (!checkExists(timer_name))
        {
            fprintf(stderr, "TIMER ERROR: No such timer exists to print stats for\n");
            return;
        }

        TimeStatistics cur = named_timers_[timer_name];
        int64_t average_time = cur.total_time / (uint64_t)cur.call_count;

        // Unless we are timing on the order of hundreds of years this check is fine
        int64_t true_min_time = cur.min_time < std::numeric_limits<uint64_t>::max() ? cur.min_time : 0;

        if (print_header)
        {
            printf("\n------------------------------------------\n");
            printf("TIMING STATS\n");
            printf("------------------------------------------\n");
        }
        printf("Timing stats for %s timer\n", timer_name.c_str());
        printf("Number of times called ->          %d\n", cur.call_count);
        printf("Total time             ->          %6.2fms\n", (double)(cur.total_time) / 1000000.0);
        printf("Average time           ->          %6.2fms\n", (double)(average_time) / 1000000.0);
        printf("Minimum time           ->          %6.2fms\n", (double)(true_min_time) / 1000000.0);
        printf("Maximum time           ->          %6.2fms\n", (double)(cur.max_time) / 1000000.0);
        printf("\n------------------------------------------\n");
    }

    void printAllTimers(bool reset_timers = true)
    {

        // We want to print the header only on the first timer (makes output neater)
        bool print_header = true;
        for (const std::pair<std::string, TimeStatistics> &it : named_timers_)
        {
            printTimerStats(it.first, print_header);

            print_header = false;

            if (reset_timers)
                reset(it.first);
        }
    }

private:
    typedef struct TimeStatistics
    {
        int call_count = 0;
        uint64_t min_time = std::numeric_limits<uint64_t>::max();
        uint64_t max_time = std::numeric_limits<uint64_t>::min();
        uint64_t total_time = 0;

        bool running = false;
        uint64_t start_time = 0;
    } TimeStatistics;

    uint64_t rc_nanos_monotonic_time()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ((uint64_t)ts.tv_sec * 1000000000) + ts.tv_nsec;
    };

    bool checkExists(std::string timer_name)
    {
        return named_timers_.count(timer_name) == 1;
    }

    std::unordered_map<std::string, TimeStatistics> named_timers_;
};

#endif