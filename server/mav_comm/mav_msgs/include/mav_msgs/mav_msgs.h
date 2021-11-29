#ifndef MAV_MSGS_H_
#define MAV_MSGS_H_

#include <vector>

namespace mav_msgs {

typedef struct Actuators {
    std::vector<float> angles;
    std::vector<float> angular_velocities;
    std::vector<float> normalized;
} Actuators;

typedef struct Quaternion{
    float x;
    float y;
    float z;
    float w;
} Quaternion;

typedef struct Vector3 {
    float x;
    float y;
    float z;
} Vector3;

typedef struct AttitudeThrust {
    Quaternion attitude;
    Vector3 thrust;
} AltitudeThrust;

typedef struct FilteredSensorData {
    Vector3 accelerometer;
    Vector3 gyroscope;
    Vector3 magnetometer;
    float barometer;
} FilteredSensorData;

typedef struct GpsWaypoint {
    float latitude; // latitude in degree
    float longitud; // longitude in degree
    float altitude; // above start-up point
    float heading; // GPS heading
    float maxSpeed; // maximum approach speed
    float maxAcc; // maximum approach accelerations
} GpsWaypoint;

typedef struct RateThrust {
    Vector3 angular_rates;
    Vector3 thrust;
} RateThrust;

typedef struct RollPitchYawrateThrust{
    float roll;
    float pitch;
    float yaw_rate;
    Vector3 thrust;
} RollPitchYawrateThrust;

typedef struct TorqueThrust {
    Vector3 torque;
    Vector3 thrust;
} TorqueThrust;

} //mav_msgs

#endif //MAV_MSGS_H_
