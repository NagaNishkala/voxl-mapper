#ifndef MAV_PLANNING_COMMON_PHYSICAL_CONSTRAINTS_H_
#define MAV_PLANNING_COMMON_PHYSICAL_CONSTRAINTS_H_

#include <math.h>

namespace mav_planning {

// Physical constraints for all planners.
struct PhysicalConstraints {
 public:
  PhysicalConstraints()
      : v_max(1.0),
        a_max(2.0),
        yaw_rate_max(M_PI / 4.0),
        robot_radius(0.35), // turi
        sampling_dt(0.1) {}

  double v_max;  // Meters/second
  double a_max;  // Meters/second^2
  double yaw_rate_max;  // Rad/second
  double robot_radius;  // Meters
  double sampling_dt;  // Seconds
};

}  // namespace mav_planning

#endif  // MAV_PLANNING_COMMON_PHYSICAL_CONSTRAINTS_H_
