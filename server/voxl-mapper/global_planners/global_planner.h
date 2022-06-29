#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <mav_trajectory_generation/trajectory.h>

/**
 * @brief Base class for global planners to adhere to
 *
 */
class GlobalPlanner
{
public:
    GlobalPlanner() {};

    /**
     * @brief Method to setup any additional resources needed for planning
     */
    virtual void setup(){};

    /**
     * @brief Method to tear down resources used during planning
     */
    virtual void tearDown(){};

    /**
     * @brief Run the planner to solve for a path through the map
     *
     * @param start_pos Starting position for planner
     * @param end_pos Goal position for planner
     * @param trajectory The returned trajectory that is collision free through the map
     * @return true If a path was found
     * @return false If no path was found
     */
    virtual bool createPlan(const Eigen::Vector3d &start_pos,
                            const Eigen::Vector3d &end_pos,
                            mav_trajectory_generation::Trajectory &trajectory) = 0;

    virtual ~GlobalPlanner() {};
};

#endif