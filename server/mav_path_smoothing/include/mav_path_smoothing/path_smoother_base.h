#ifndef MAV_PATH_SMOOTHING_PATH_SMOOTHER_BASE_H_
#define MAV_PATH_SMOOTHING_PATH_SMOOTHER_BASE_H_

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_common/physical_constraints.h>

namespace mav_planning {

class PathSmootherBase {
   public:
    PathSmootherBase() : verbose_(true) {}
    virtual ~PathSmootherBase() {}

    virtual void setParameters(PhysicalConstraints& constraints, bool verbose);
    void setPhysicalConstraints(const PhysicalConstraints& constraints);
    const PhysicalConstraints& getPhysicalConstraints() const;

    void setVerbose(bool verbose) { verbose_ = verbose; }
    bool getVerbose() const { return verbose_; }

    // By default, getPathBetweenWaypoints just calls getPathBetweenTwoPoints
    // on consecutive waypoints.
    virtual bool getPathBetweenWaypoints(
        const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
        mav_msgs::EigenTrajectoryPoint::Vector* path) const;

    virtual bool getPathBetweenTwoPoints(
        __attribute__((unused)) const mav_msgs::EigenTrajectoryPoint& start,
        __attribute__((unused)) const mav_msgs::EigenTrajectoryPoint& goal,
        __attribute__((unused)) mav_msgs::EigenTrajectoryPoint::Vector* path) const {
        return false;
    }

   protected:
    PhysicalConstraints constraints_;

    bool verbose_;
};

}  // namespace mav_planning

#endif  // MAV_PATH_SMOOTHING_PATH_SMOOTHER_BASE_H_
