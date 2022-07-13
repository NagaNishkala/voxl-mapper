#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <mav_trajectory_generation/trajectory.h>
#include "planner_utils.h"
#include "pthread.h"

/**
 * @brief Base class for local planners to adhere to
 *
 */
class LocalPlanner
{
public:
    LocalPlanner(){};

    /**
     * @brief Method to setup any additional resources needed for planning
     */
    virtual void setup(){};

    /**
     * @brief Method to tear down resources used during planning
     */
    virtual void tearDown(){};

    /**
     * @brief Set the global plan that the local planner should track
     *
     * @param waypoints The waypoints from a global planner
     */
    virtual void setPlan(const Point3fVector &waypoints) = 0;

    /**
     * @brief Start local planner and any associated threads
     *
     */
    virtual void start() = 0;

    /**
     * @brief Stop local planner and any associated threads
     *
     */
    virtual void stop() = 0;

    /**
     * @brief Set the current segment id and evaluation time. This should be done
     * whenever we receive an EVALUATED trajectory protocol message
     * 
     * @param segment_id id of current segment
     * @param segment_t eval time of current segment
     */
    void setCurrentSegment(int segment_id, double segment_t)
    {
        pthread_mutex_lock(&segment_mutex);
        cur_segment_id_ = segment_id;
        cur_segment_t_ = segment_t;
        pthread_mutex_unlock(&segment_mutex);
    }
    
    virtual ~LocalPlanner(){};

protected:
    pthread_mutex_t segment_mutex = PTHREAD_MUTEX_INITIALIZER;
    int cur_segment_id_ = 0;
    double cur_segment_t_ = 0;

};

#endif