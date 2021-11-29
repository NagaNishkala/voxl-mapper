/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Conversion functions between Eigen types and MAV ROS message types.

#ifndef MAV_MSGS_CONVERSIONS_H
#define MAV_MSGS_CONVERSIONS_H


#include "mav_msgs/mav_msgs.h"
#include "mav_msgs/common.h"
#include "mav_msgs/default_values.h"
#include "mav_msgs/eigen_mav_msgs.h"

namespace mav_msgs {

inline void eigenAttitudeThrustFromMsg(const AttitudeThrust& msg, EigenAttitudeThrust* attitude_thrust) {
  assert(attitude_thrust != NULL);

  attitude_thrust->attitude = quaternionFromMsg(msg.attitude);
  attitude_thrust->thrust = vector3FromMsg(msg.thrust);
}

inline void eigenActuatorsFromMsg(const Actuators& msg, EigenActuators* actuators) {
  assert(actuators != NULL);

  // Angle of the actuator in [rad].
  actuators->angles.resize(msg.angles.size());
  for (unsigned int i = 0; i < msg.angles.size(); ++i) {
    actuators->angles[i] = msg.angles[i];
  }

  // Angular velocities of the actuator in [rad/s].
  actuators->angular_velocities.resize(msg.angular_velocities.size());
  for (unsigned int i = 0; i < msg.angular_velocities.size(); ++i) {
    actuators->angular_velocities[i] = msg.angular_velocities[i];
  }

  // Normalized: Everything that does not fit the above, normalized
  // between [-1 ... 1].
  actuators->normalized.resize(msg.normalized.size());
  for (unsigned int i = 0; i < msg.normalized.size(); ++i) {
    actuators->normalized[i] = msg.normalized[i];
  }
}

inline void eigenRateThrustFromMsg(const RateThrust& msg, EigenRateThrust* rate_thrust) {
  assert(rate_thrust != NULL);

  rate_thrust->angular_rates = vector3FromMsg(msg.angular_rates);
  rate_thrust->thrust = vector3FromMsg(msg.thrust);
}

inline void eigenTorqueThrustFromMsg(const TorqueThrust& msg, EigenTorqueThrust* torque_thrust) {
  assert(torque_thrust != NULL);

  torque_thrust->torque = vector3FromMsg(msg.torque);
  torque_thrust->thrust = vector3FromMsg(msg.thrust);
}

inline void eigenRollPitchYawrateThrustFromMsg(
    const RollPitchYawrateThrust& msg,
    EigenRollPitchYawrateThrust* roll_pitch_yawrate_thrust) {
  assert(roll_pitch_yawrate_thrust != NULL);

  roll_pitch_yawrate_thrust->roll = msg.roll;
  roll_pitch_yawrate_thrust->pitch = msg.pitch;
  roll_pitch_yawrate_thrust->yaw_rate = msg.yaw_rate;
  roll_pitch_yawrate_thrust->thrust = vector3FromMsg(msg.thrust);
}

}  // end namespace mav_msgs

#endif  // MAV_MSGS_CONVERSIONS_H
