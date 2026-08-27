#pragma once

#include "config.h"

#if IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF
#include <SensorFusion.h>
#endif

#if IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF
struct EulerDeg {
    float yaw, pitch, roll;
};

struct UnitVector {
    float x, y, z;
};

float unwrapToPrev(float current, float prev);
EulerDeg quaternionToEulerZYX_unwrapped(const Quaternion& q);
UnitVector quaternionToUnitVector(const Quaternion& q);

float unitVectorToSphericalTheta(const UnitVector& uv);
float unitVectorToSphericalPhi(const UnitVector& uv);
float normalizedQuaternionToSphericalTheta(const Quaternion& q);
float normalizedQuaternionToSphericalPhi(const Quaternion& q);

#endif

#define DEG2RAD     0.017453292519943295f
#define RAD2DEG     57.295779513082322864f

class LegacyIMU;

struct ImuImpactFrame {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};

ImuImpactFrame imu_impact_frame(const LegacyIMU& imu);

#if IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF && IMU_ROTATE

xyz_t imu_rot_apply_accel(xyz_t accel);
xyz_t imu_rot_apply_gyro_rad(xyz_t gyro);

// void imu_rot_apply_accel(float x, float y, float z,
//                          float& ox, float& oy, float& oz);
// void imu_rot_apply_gyro_rad(float gx, float gy, float gz,
//                             float& ogx, float& ogy, float& ogz);

#endif