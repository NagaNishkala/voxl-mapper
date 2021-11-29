#ifndef MAV_PLANNING_MSGS_H_
#define MAV_PLANNING_MSGS_H_

#include <stdint.h>
#include <voxblox/Eigen/Core>
#include <vector>

#include "mav_msgs/mav_msgs.h"

namespace mav_planning_msgs {

typedef struct EigenPolynomialSegment {
  EigenPolynomialSegment() : segment_time_ns(0), num_coeffs(0) {};

  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd z;
  Eigen::VectorXd yaw;
  Eigen::VectorXd rx;
  Eigen::VectorXd ry;
  Eigen::VectorXd rz;
  uint64_t segment_time_ns;
  int num_coeffs;
}EigenPolynomialSegment;

typedef std::vector<EigenPolynomialSegment> EigenPolynomialTrajectory;


typedef struct PolynomialSegment {
    uint64_t ts;
    uint32_t num_coeffs;     //order of the polynomial + 1, should match size of x*
    uint64_t segment_time;   // duration of the segment
    std::vector<double> x;                // coefficients for the x-axis, INCREASING order
    std::vector<double> y;                // coefficients for the y-axis, INCREASING order
    std::vector<double> z;                // coefficients for the z-axis, INCREASING order
    std::vector<double> rx;               // coefficients for the rotation x-vector, INCREASING order
    std::vector<double> ry;               // coefficients for the rotation y-vector, INCREASING order
    std::vector<double> rz;               // coefficients for the rotation z-vector, INCREASING order
    // For backwards compatibility with underactuated (4DOF) commands):
    std::vector<double> yaw;              // coefficients for the yaw, INCREASING order
} PolynomialSegment;

typedef struct PolynomialSegment4D {
    uint64_t ts;
    uint32_t num_coeffs;     //order of the polynomial + 1, should match size of x*
    uint64_t segment_time;   // duration of the segment
    std::vector<double> x;                // coefficients for the x-axis, INCREASING order
    std::vector<double> y;                // coefficients for the y-axis, INCREASING order
    std::vector<double> z;                // coefficients for the z-axis, INCREASING order
    std::vector<double> yaw;              // coefficients for the yaw, INCREASING order
} PolynomialSegment4D;

typedef struct PolynomialTrajectory {
    uint64_t ts;
    int num_segments;
    PolynomialSegment* segments;
} PolynomialTrajectory;

typedef struct PolynomialTrajectory4D {
    uint64_t ts;
    int num_segments;
    std::vector<PolynomialSegment4D> segments;
} PolynomialTrajectory4D;


}
#endif //MAV_PLANNING_MSGS_H_
