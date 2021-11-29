/*
 * Copyright (c) 2016, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Rik Bähnemann, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mav_local_planner/conversions.h"

namespace mav_trajectory_generation {

/// Converts an EigenPolynomialSegment to a PolynomialSegment message. Does NOT
/// set the header!
// inline void polynomialSegmentMsgFromEigen(const mav_planning_msgs::EigenPolynomialSegment& segment,
//                                           mav_planning_msgs::PolynomialSegment4D* msg) {
//     //assert(msg != NULL);
//     msg->x.resize(segment.x.size());
//     Eigen::VectorXd::Map(&msg->x[0], segment.x.size()) = segment.x;

//     msg->y.resize(segment.y.size());
//     Eigen::VectorXd::Map(&msg->y[0], segment.y.size()) = segment.y;

//     msg->z.resize(segment.z.size());
//     Eigen::VectorXd::Map(&msg->z[0], segment.z.size()) = segment.z;

//     msg->yaw.resize(segment.yaw.size());
//     Eigen::VectorXd::Map(&msg->yaw[0], segment.yaw.size()) = segment.yaw;

//     msg->segment_time = segment.segment_time_ns;
//     msg->num_coeffs = segment.num_coeffs;
// }


 inline void msgArrayFromVector(const Eigen::VectorXd& x,
                                std::vector<double>* array) {
   array->resize(x.size());
   Eigen::Map<Eigen::VectorXd> map =
       Eigen::Map<Eigen::VectorXd>(&((*array)[0]), array->size());
   map = x;
 }

  inline void polynomialSegmentMsgFromEigen(const mav_planning_msgs::EigenPolynomialSegment& segment,
                                           mav_planning_msgs::PolynomialSegment4D* msg) {
   msgArrayFromVector(segment.x, &(msg->x));
   msgArrayFromVector(segment.y, &(msg->y));
   msgArrayFromVector(segment.z, &(msg->z));
   msgArrayFromVector(segment.yaw, &(msg->yaw));

   msg->segment_time = (segment.segment_time_ns);
   msg->num_coeffs = segment.num_coeffs;
 }

bool trajectoryToPolynomialTrajectoryMsg(
    const Trajectory& trajectory,
    mav_planning_msgs::PolynomialTrajectory4D* msg) {
    if (msg == nullptr) {
        fprintf(stderr, "msg is null: %s\n", __FUNCTION__);
        return false;
    }
    msg->segments.clear();

    bool success = true;

    Segment::Vector segments;
    trajectory.getSegments(&segments);

    msg->segments.reserve(segments.size());
    printf("%lu SEGMENTS\n", segments.size());
    for (size_t i = 0; i < segments.size(); ++i) {
        const Segment& segment = segments[i];

        if (segment.D() != 3 && segment.D() != 4) {
            fprintf(stderr, "Dimension of position segment has to be 3 or 4, but is %d\n", segment.D());
            success = false;
            break;
        }

        mav_planning_msgs::PolynomialSegment4D segment_msg;
        mav_planning_msgs::EigenPolynomialSegment eigen_segment;
        eigen_segment.x = segment[0].getCoefficients();
        eigen_segment.y = segment[1].getCoefficients();
        eigen_segment.z = segment[2].getCoefficients();
        if (segment.D() == 4) {
            eigen_segment.yaw = segment[3].getCoefficients();
        }
        eigen_segment.num_coeffs = segment.N();
        eigen_segment.segment_time_ns = segment.getTimeNSec();

        polynomialSegmentMsgFromEigen(eigen_segment, &segment_msg);
        msg->segments.push_back(segment_msg);
    }

    if (!success) msg->segments.clear();
    return success;
}

}  // namespace mav_trajectory_generation
