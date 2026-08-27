#include <Arduino.h>
#include "utils/imu_helper.h"
#include "IMS/ims.h"

ImuImpactFrame imu_impact_frame(const LegacyIMU& imu) {
    ImuImpactFrame frame;
#if (IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF) && IMU_ROTATE
    xyz_t rotatedAccel = imu.rotatedAccel();
    xyz_t rotatedGyro = imu.rotatedGyro();
    frame.ax = rotatedAccel.y;
    frame.ay = rotatedAccel.x;
    frame.az = rotatedAccel.z;
    frame.gx = -rotatedGyro.y;
    frame.gy = -rotatedGyro.x;
    frame.gz = rotatedGyro.z;
#else
    frame.ax = imu.accY();
    frame.ay = imu.accX();
    frame.az = imu.accZ();
    frame.gx = -imu.gyrY();
    frame.gy = -imu.gyrX();
    frame.gz = imu.gyrZ();
#endif
    return frame;
}

#if IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF && IMU_ROTATE

xyz_t imu_rot_apply_accel(xyz_t accel)
{
    return xyz_t{
        IMU_R00 * accel.x + IMU_R01 * accel.y + IMU_R02 * accel.z,
        IMU_R10 * accel.x + IMU_R11 * accel.y + IMU_R12 * accel.z,
        IMU_R20 * accel.x + IMU_R21 * accel.y + IMU_R22 * accel.z
    };
}

xyz_t imu_rot_apply_gyro_rad(xyz_t gyro)
{
    return xyz_t{
        IMU_R00 * gyro.x + IMU_R01 * gyro.y + IMU_R02 * gyro.z,
        IMU_R10 * gyro.x + IMU_R11 * gyro.y + IMU_R12 * gyro.z,
        IMU_R20 * gyro.x + IMU_R21 * gyro.y + IMU_R22 * gyro.z
    };
}

// void imu_rot_apply_accel(float x, float y, float z,
//                          float& ox, float& oy, float& oz)
// {
//     ox = IMU_R00 * x + IMU_R01 * y + IMU_R02 * z;
//     oy = IMU_R10 * x + IMU_R11 * y + IMU_R12 * z;
//     oz = IMU_R20 * x + IMU_R21 * y + IMU_R22 * z;
// }

// void imu_rot_apply_gyro_rad(float gx, float gy, float gz,
//                             float& ogx, float& ogy, float& ogz)
// {
//     ogx = IMU_R00 * gx + IMU_R01 * gy + IMU_R02 * gz;
//     ogy = IMU_R10 * gx + IMU_R11 * gy + IMU_R12 * gz;
//     ogz = IMU_R20 * gx + IMU_R21 * gy + IMU_R22 * gz;
// }

#endif

#if IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF

float unwrapToPrev(float current, float prev) {
  float diff = current - prev;
  // Bring diff into [-180, 180)
  while (diff > 180.0f)  diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return prev + diff;
}

EulerDeg quaternionToEulerZYX_unwrapped(const Quaternion& q) {
  static bool initialized = false;
  static EulerDeg prev{0,0,0};

  float w = q.w, x = q.x, y = q.y, z = q.z;

  // Standard Z-Y-X extraction (one of the common formulas)
  float sinp = 2.0f * (w*y - z*x);
  float pitch_rad;
  if (fabsf(sinp) >= 1.0f)
      pitch_rad = copysignf((float)PI/2.0f, sinp);   // clamp at ±90° if needed
  else
      pitch_rad = asinf(sinp);

  float roll_rad =
      atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
  float yaw_rad =
      atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));

  EulerDeg raw;
  raw.roll  = roll_rad  * 180.0f / (float)PI;
  raw.pitch = pitch_rad * 180.0f / (float)PI;
  raw.yaw   = yaw_rad   * 180.0f / (float)PI;

  if (!initialized) {
      prev = raw;
      initialized = true;
      return raw;
  }

  EulerDeg cont;
  // cont.roll  = unwrapToPrev(raw.roll,  prev.roll);
  // cont.pitch = unwrapToPrev(raw.pitch, prev.pitch);
  // cont.yaw   = unwrapToPrev(raw.yaw,   prev.yaw);

  cont.roll = raw.roll;
  cont.pitch = raw.pitch;
  cont.yaw = raw.yaw;

  prev = cont;
  return cont;
}

// Extract unit vector representing the direction of the bot's top (Z-axis)
// in world coordinates from the orientation quaternion.
// The initial "up" vector [0, 0, 1] in bot frame is rotated to world frame.
UnitVector quaternionToUnitVector(const Quaternion& q) {
  float w = q.w, x = q.x, y = q.y, z = q.z;

  // Rotate vector by quaternion using alternative formula
  // vx = 2(xz + wy)
  // vy = 2(yz - wx)
  // vz = w² - x² - y² + z²
  float vx = 2.0f * (x*z + w*y);
  float vy = 2.0f * (y*z - w*x);
  float vz = w*w - x*x - y*y + z*z;

  // Normalize to ensure it's a unit vector (should already be normalized, but safety check)
  float mag = sqrtf(vx*vx + vy*vy + vz*vz);
  if (mag > 0.0001f) {  // Avoid division by zero
    vx /= mag;
    vy /= mag;
    vz /= mag;
  } else {
    // Fallback to upright position if quaternion is invalid
    vx = 0.0f;
    vy = 0.0f;
    vz = 1.0f;
  }

  UnitVector result;
  result.x = vx;
  result.y = vy;
  result.z = vz;
  return result;
}

// Calculate spherical theta + phi.
// unitVectorOrientation is already normalised (r == 1), so theta = acos(z)
// directly - no magnitude/sqrt/divide needed. Clamp z for acosf domain safety.
float unitVectorToSphericalTheta(const UnitVector& uv) {
  const float cz = uv.z > 1.0f ? 1.0f : (uv.z < -1.0f ? -1.0f : uv.z);
  return acosf(cz) * (float)RAD2DEG;
}

float unitVectorToSphericalPhi(const UnitVector& uv) {
  return atan2f(uv.y, uv.x) * (float)RAD2DEG;
}

// Calculate without unit vector // if orintentation is normalised. -- Untested
float normalizedQuaternionToSphericalTheta(const Quaternion& q) {
    float cz = 1.0f - 2.0f*(q.x*q.x + q.y*q.y);
    return acosf(cz);
}

// Untested
float normalizedQuaternionToSphericalPhi(const Quaternion& q) {
  return atan2f(q.y*q.z - q.w*q.x, q.x*q.z + q.w*q.y);
}

// Calculate spherical theta + phi
// const UnitVector uv = result.unitVectorOrientation;
// float sphericalR = sqrtf(uv.x * uv.x + uv.y * uv.y + uv.z * uv.z);
// if (sphericalR > 0.0001f) {
//     result.sphericalTheta = acosf(uv.z / sphericalR) * (180.0f / (float)PI);
// } else {
//     result.sphericalTheta = 0.0f;
// }
// result.sphericalPhi = atan2f(uv.y, uv.x) * 180.0f / PI;

#endif