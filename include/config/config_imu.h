// Defined Values -- Do not change these values
#define IMU_SENSOR_FUSION_MODE_OFF          0
#define IMU_SENSOR_FUSION_MODE_MADGWICK     1
#define IMU_SENSOR_FUSION_MODE_VQF          2


#ifndef __DEFINES_ONLY
// IMU Sensor Fusion Mode
// Default to VQF mode - this provides what seems to be the best accuracy at the cost of performance
#if !defined(IMU_SENSOR_FUSION_MODE)
#define IMU_SENSOR_FUSION_MODE            IMU_SENSOR_FUSION_MODE_VQF
#endif

#if !defined(IMU_ROTATE)
#define IMU_ROTATE                        false
#endif

#endif // __DEFINES_ONLY