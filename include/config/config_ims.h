// Defined Values -- Do not change these values

#ifndef __DEFINES_ONLY


#if !defined(IMS_IMU_SAMPLE_RATE_HZ)
#define IMS_IMU_SAMPLE_RATE_HZ             500
#endif


// IMS_GYRO_Z_LP_ORDER // this controls how tight the cutoff is for the gyro z low pass filter
// 0 = Passthru
// 1 = First-order Low-pass
// 2 = Second-order Low-pass
// 4 = Fourth-order Low-pass
#if !defined(IMS_GYRO_Z_LP_ORDER)
#define IMS_GYRO_Z_LP_ORDER                  1
#endif

#if !defined(IMS_GYRO_Z_LP_CUTOFF_HZ)
#define IMS_GYRO_Z_LP_CUTOFF_HZ              100.0f
#endif


#if !defined(IMS_SPHERICAL_THETA_LP_SAMPLE_RATE_HZ)
#define IMS_SPHERICAL_THETA_LP_SAMPLE_RATE_HZ          200
#endif

#if !defined(IMS_SPHERICAL_THETA_LP_CUTOFF_HZ)
#define IMS_SPHERICAL_THETA_LP_CUTOFF_HZ               13.3f
#endif

#endif // __DEFINES_ONLY