#pragma once

#include <stdint.h>

#include "OrbtICM42688.h"
#include "OrbtDSP.h"

#include "config.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "utils/imu_helper.h"

#if IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF
#include <SensorFusion.h>
#endif


struct IMSSample {
    float accX = 0.0f;
    float accY = 0.0f;
    float accZ = 0.0f;
    float gyrX = 0.0f;
    float gyrY = 0.0f;
    float gyrZ = 0.0f;
    float temp = 0.0f;
    uint32_t sequence = 0;
    uint32_t deltaTimeUs = 0;
};

class IMSComputeResult {
public:
    float accX = 0.0f;
    float accY = 0.0f;
    float accZ = 0.0f;
    float gyrX = 0.0f;
    float gyrY = 0.0f;
    float gyrZ = 0.0f;
    float imuTemp = 0.0f;

    uint32_t sequence = 0;
    uint32_t deltaTimeUs = 0;

    // Quaternion values from sensor fusion
    Quaternion orientation;

    // Rotated raw values
    float rot_accX = 0.0f;
    float rot_accY = 0.0f;
    float rot_accZ = 0.0f;
    float rot_gyrX = 0.0f;
    float rot_gyrY = 0.0f;
    float rot_gyrZ = 0.0f;

    // Gyro Z Low Pass Filter
    float gyrZ_LP = 0.0f;

    // Theta Smoothed
    float sphericalTheta_LP = 0.0f;

    // Gyro Magnitude Mean & Variance EMA
    float gyroMagnitudeMean = 0.0f;
    float gyroMagnitudeVariance = 0.0f;

    // Euler orientation
    bool _eulerOrientationValid = false;
    EulerDeg _eulerOrientation = { 0.0f, 0.0f, 0.0f };

    EulerDeg eulerOrientation() {
        if(!_eulerOrientationValid) {
            _eulerOrientation = quaternionToEulerZYX_unwrapped(orientation);
            _eulerOrientationValid = true;
        }
        return _eulerOrientation;
    }

    // Unit vector orientation
    bool _unitVectorOrientationValid = false;
    UnitVector _unitVectorOrientation = { 0.0f, 0.0f, 1.0f };

    UnitVector unitVectorOrientation() {
        if(!_unitVectorOrientationValid) {
            _unitVectorOrientation = quaternionToUnitVector(orientation);
            _unitVectorOrientationValid = true;
        }
        return _unitVectorOrientation;
    }

    // Spherical coordinates
    bool _sphericalThetaValid = false;
    float _sphericalTheta = 0.0f;
    bool _sphericalPhiValid = false;
    float _sphericalPhi = 0.0f;


    float sphericalTheta() {
        if(!_sphericalThetaValid) {
            _sphericalTheta = unitVectorToSphericalTheta(unitVectorOrientation());
            _sphericalThetaValid = true;
        }
        return _sphericalTheta;
    }

    float sphericalPhi() {
        if(!_sphericalPhiValid) {
            _sphericalPhi = unitVectorToSphericalPhi(unitVectorOrientation());
            _sphericalPhiValid = true;
        }
        return _sphericalPhi;
    }

    // GyroMagitude^2
    float _gyroMagnitudeSquaredValid = false;
    float _gyroMagnitudeSquared = 0.0f;
    float gyroMagnitudeSquared() {
        if(!_gyroMagnitudeSquaredValid) {
            _gyroMagnitudeSquared = gyrX * gyrX + gyrY * gyrY + gyrZ * gyrZ;
            _gyroMagnitudeSquaredValid = true;
        }
        return _gyroMagnitudeSquared;
    }
};

struct IMSCommand {
    enum Command {
        COMMAND_NONE,
        COMMAND_CALIBRATE,
    };
    Command command;
};

class LegacyIMU {
public:
    void update(const IMSComputeResult& result);

    float accX() const { return _computeResult.accX; }
    float accY() const { return _computeResult.accY; }
    float accZ() const { return _computeResult.accZ; }
    float gyrX() const { return _computeResult.gyrX; }
    float gyrY() const { return _computeResult.gyrY; }
    float gyrZ() const { return _computeResult.gyrZ; }
    float temp() const { return _computeResult.imuTemp; }

    Quaternion orientation() const { return _computeResult.orientation; }

    EulerDeg eulerOrientation() { return _computeResult.eulerOrientation(); }

    UnitVector unitVectorOrientation() { return _computeResult.unitVectorOrientation(); }
    float sphericalTheta() { return _computeResult.sphericalTheta(); }
    float sphericalPhi() { return _computeResult.sphericalPhi(); }

    xyz_t rotatedAccel() const { return { _computeResult.rot_accX, _computeResult.rot_accY, _computeResult.rot_accZ }; }
    xyz_t rotatedGyro() const { return { _computeResult.rot_gyrX, _computeResult.rot_gyrY, _computeResult.rot_gyrZ }; }

    float gyrZ_LP() const { return _computeResult.gyrZ_LP; }
    float sphericalTheta_LP() const { return _computeResult.sphericalTheta_LP; }

    float gyroMagnitudeSquared() { return _computeResult.gyroMagnitudeSquared(); }
    float gyroMagnitudeMean() const { return _computeResult.gyroMagnitudeMean; }
    float gyroMagnitudeVariance() const { return _computeResult.gyroMagnitudeVariance; }

    IMSComputeResult getComputeResult() const { return _computeResult; }

private:
    IMSComputeResult _computeResult;
};

class IMS {
public:
    bool begin();
    void end();
    void registerLegacy(LegacyIMU* legacyIMU);
    bool update();
    bool isRunning() const;
    IMSComputeResult getComputeResult() const;
    bool getComputeResult(IMSComputeResult& result) const;
    bool getLatestComputeResult(IMSComputeResult& result) const;
    LegacyIMU* getLegacyIMU();

    const LegacyIMU* getLegacyIMU() const;
    uint32_t sampleCount() const;

    // IMU commands
    void imuCalibrateGyro();

protected:
    static constexpr uint32_t TIMER_RESOLUTION_HZ = 1000000;
    static constexpr uint64_t TIMER_TICKS_PER_SAMPLE = TIMER_RESOLUTION_HZ / IMS_IMU_SAMPLE_RATE_HZ;
    static constexpr uint32_t IMU_ODR_HZ = 1000;
    static constexpr uint8_t IMU_SAMPLES_PER_FIFO_READ = (IMU_ODR_HZ / IMS_IMU_SAMPLE_RATE_HZ) + 1;
    static constexpr uint8_t QUEUE_DEPTH = IMU_SAMPLES_PER_FIFO_READ * 2;

    static void IMUTaskStub(void* args);
    static void IMUComputeTaskStub(void* args);
    static bool AlarmIsr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);

    void _imuTaskLoop();
    void _computeTaskLoop();
    bool _configureTimer();
    void _clearLatestSample();
    void _diagnostics_sample_frequency(IMSSample& oldSample, IMSSample& newSample);
    void _sendIMUCommand(IMSCommand::Command command);
    bool _computeSample(const IMSSample& sample, IMSComputeResult& result);

    gptimer_handle_t _gptimer = nullptr;
    TaskHandle_t _imuTaskHandle = nullptr;
    QueueHandle_t _IMUSampleQueue = nullptr;
    QueueHandle_t _IMUCommandQueue = nullptr;

    TaskHandle_t _computeTaskHandle = nullptr;
    QueueHandle_t _computeToForegroundQueue = nullptr;

    IMSComputeResult _foregroundComputeResult;
    LegacyIMU* _legacyIMU = nullptr;
    uint32_t _foregroundSampleCount = 0;
    uint32_t _backgroundSampleCount = 0;
    volatile bool _running = false;

    // Sensor Fusion
    #if IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF
    Quaternion _orientation;

    #if IMU_SENSOR_FUSION_MODE == IMU_SENSOR_FUSION_MODE_MADGWICK
    //Madgwick Filter
    MadgwickFilter *_sensorFusionFilter;

    #elif IMU_SENSOR_FUSION_MODE == IMU_SENSOR_FUSION_MODE_VQF
    //VQF Filter
    VQF *_sensorFusionFilter;
    #endif
    #endif

    // IIR Filters
#if IMS_GYRO_Z_LP_ORDER == 0
    OrbtDSP_PassthruF _gyrZ_LP = OrbtDSP_PassthruF();
#elif IMS_GYRO_Z_LP_ORDER == 1
    OrbtDSP_IIR_1LPf _gyrZ_LP = OrbtDSP_IIR_1LPf(IMU_ODR_HZ, IMS_GYRO_Z_LP_CUTOFF_HZ);
#elif IMS_GYRO_Z_LP_ORDER == 2
    OrbtDSP_IIR_2LPf _gyrZ_LP = OrbtDSP_IIR_2LPf(IMU_ODR_HZ, IMS_GYRO_Z_LP_CUTOFF_HZ);
#elif IMS_GYRO_Z_LP_ORDER == 4
    OrbtDSP_IIR_4LPf _gyrZ_LP = OrbtDSP_IIR_4LPf(IMU_ODR_HZ, IMS_GYRO_Z_LP_CUTOFF_HZ);
#endif

    // Theta Smoothed
    OrbtDSP_IIR_1LPf _sphericalTheta_LP = OrbtDSP_IIR_1LPf(IMS_SPHERICAL_THETA_LP_SAMPLE_RATE_HZ, IMS_SPHERICAL_THETA_LP_CUTOFF_HZ);

    // Gyro Magnitude Mean & Variance EMA
    float _gyroMagnitudeMean = 0.0f;
    float _gyroMagnitudeVariance = 0.0f;
};
