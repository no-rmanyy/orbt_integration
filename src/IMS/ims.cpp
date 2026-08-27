#include "IMS/ims.h"
#include "utils/imu_helper.h"

#include "Arduino.h"
#include "esp_timer.h"
#include "config_pins.h"

#include <type_traits>
#include <cmath>

// These types are passed by value through FreeRTOS queues, which byte-copy
// (memcpy) the item without running any constructors/destructors. That is only
// valid for trivially-copyable types. If one of these structs ever gains a
// member that owns a resource (std::string, std::vector, unique_ptr, etc.),
// these asserts will fail the build instead of silently corrupting memory.
static_assert(std::is_trivially_copyable_v<IMSSample>,
              "IMSSample is queued by value and must be trivially copyable");
static_assert(std::is_trivially_copyable_v<IMSComputeResult>,
              "IMSComputeResult is queued by value and must be trivially copyable");
static_assert(std::is_trivially_copyable_v<IMSCommand>,
              "IMSCommand is queued by value and must be trivially copyable");

// 4 Wire Mode
static OrbtICM42688_FIFO _imu((gpio_num_t)PIN_IMU_CS, (gpio_num_t)PIN_IMU_SCK, (gpio_num_t)PIN_IMU_MISO, (gpio_num_t)PIN_IMU_MOSI);

// 3 Wire Mode
//static OrbtICM42688_FIFO _imu((gpio_num_t)PIN_IMU_CS, (gpio_num_t)PIN_IMU_SCK, (gpio_num_t)PIN_IMU_MOSI, (gpio_num_t)GPIO_NUM_NC);


void LegacyIMU::update(const IMSComputeResult& result)
{
    _computeResult = result;
}

void IMS::IMUTaskStub(void* args)
{
    IMS* self = static_cast<IMS*>(args);
    self->_imuTaskLoop();
}

void IMS::IMUComputeTaskStub(void* args)
{
    IMS* self = static_cast<IMS*>(args);
    self->_computeTaskLoop();
}

bool ARDUINO_ISR_ATTR IMS::AlarmIsr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx)
{
    IMS* self = static_cast<IMS*>(user_ctx);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    gptimer_alarm_config_t alarm_config = {
        //.alarm_count = edata->alarm_value + TIMER_TICKS_PER_SAMPLE,
        .alarm_count = TIMER_TICKS_PER_SAMPLE,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true
        }
    };
    gptimer_set_alarm_action(timer, &alarm_config);

    if(self != nullptr && self->_running && self->_imuTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(self->_imuTaskHandle, &xHigherPriorityTaskWoken);
    }

    return xHigherPriorityTaskWoken == pdTRUE;
}

bool IMS::begin()
{
    if(_running) {
        return true;
    }

    // Initialize the IMU
    int status = _imu.begin();
    if (status < 0)
    {
      //Disable as TCPPIP probably isn't up yet: LOG_INFO("IMU initialization unsuccessful. Check IMU wiring or try cycling power.");
      Serial.println("IMU initialization unsuccessful. Check IMU wiring or try cycling power");
      Serial.printf("Status is %d", status);

      // stall catastrophically forever??
      while(1) {}
    }

    // Clear the latest sample
    _clearLatestSample();

    if(_IMUSampleQueue == nullptr) {
        _IMUSampleQueue = xQueueCreate(QUEUE_DEPTH, sizeof(IMSSample));
        if(_IMUSampleQueue == nullptr) {
            return false;
        }
    } else {
        xQueueReset(_IMUSampleQueue);
    }

    if(_IMUCommandQueue == nullptr) {
        _IMUCommandQueue = xQueueCreate(1, sizeof(IMSCommand));
        if(_IMUCommandQueue == nullptr) {
            return false;
        }
    } else {
        xQueueReset(_IMUCommandQueue);
    }

    if(_computeToForegroundQueue == nullptr) {
        _computeToForegroundQueue = xQueueCreate(1, sizeof(IMSComputeResult));
        if(_computeToForegroundQueue == nullptr) {
            return false;
        }
    } else {
        xQueueReset(_computeToForegroundQueue);
    }

    // Sensor Fusion
    #if IMU_SENSOR_FUSION_MODE == IMU_SENSOR_FUSION_MODE_MADGWICK
    //Madgwick Filter
    _sensorFusionFilter = new MadgwickFilter();
    _sensorFusionFilter.setBeta(0.5F);

    #elif IMU_SENSOR_FUSION_MODE == IMU_SENSOR_FUSION_MODE_VQF
    //VQF Filter
    //_sensorFusionFilter = new VQF(0.01f, 0.01f, 0.01f,
    _sensorFusionFilter = new VQF(0.001f, 0.001f, 0.001f,
        true,   // restBiasEstimationEnabled
        true,   // motionBiasEstimationEnabled
        false); // magDisturbanceRejectionEnabled
    #endif

    // Ready to start tasks
    _running = true;

    // 24 priority is higher than wifi, thus the ims task will run before the wifi task -- this really cleans up timing jitter
    // 19 is higher than the network tcp/ip tasks
    // 11 is higher than all of our other tasks
    BaseType_t taskCreated = xTaskCreate(IMUTaskStub, "IMS_IMU", 4096, this, 11, &_imuTaskHandle);
    if(taskCreated != pdPASS) {
        _imuTaskHandle = nullptr;
        _running = false;
        return false;
    }

    taskCreated = xTaskCreate(IMUComputeTaskStub, "IMS_COMPUTE", 4096, this, 5, &_computeTaskHandle);
    if(taskCreated != pdPASS) {
        _running = false;
        xTaskNotifyGive(_imuTaskHandle); // wake up the imu task to stop

        _computeTaskHandle = nullptr;
        return false;
    }

    if(!_configureTimer()) {
        end();
        return false;
    }

    return true;
}

void IMS::end()
{
    if(!_running && _gptimer == nullptr && _imuTaskHandle == nullptr) {
        return;
    }

    _running = false;

    if(_gptimer != nullptr) {
        gptimer_stop(_gptimer);
        gptimer_disable(_gptimer);
        gptimer_del_timer(_gptimer);
        _gptimer = nullptr;
    }

    TaskHandle_t imuTaskHandle = _imuTaskHandle;
    _imuTaskHandle = nullptr;
    if(imuTaskHandle != nullptr) {
        xTaskNotifyGive(imuTaskHandle);
    }

    TaskHandle_t computeTaskHandle = _computeTaskHandle;
    _computeTaskHandle = nullptr;
    if(computeTaskHandle != nullptr) {
        xTaskNotifyGive(computeTaskHandle);
    }

    _imu.stopStreamingToFifo();
}

void IMS::registerLegacy(LegacyIMU* legacyIMU)
{
    _legacyIMU = legacyIMU;
    if(_legacyIMU != nullptr && _foregroundSampleCount > 0) {
        _legacyIMU->update(_foregroundComputeResult);
    }
}

static volatile uint32_t backgroundDeltaTimeUs = 0;
static float backgroundDeltaTimeUsAcc = 0.0f;
void IMS::_diagnostics_sample_frequency(IMSSample& oldSample, IMSSample& newSample)
{
    float frequencyAlpha = 0.001f;

    if(oldSample.sequence == 0 || newSample.sequence == 0) {
        return;
    }

    backgroundDeltaTimeUsAcc = backgroundDeltaTimeUsAcc * (1.0f - frequencyAlpha) + backgroundDeltaTimeUs * frequencyAlpha;
    //Serial.printf("bdts: %d -- %d / %2.2f us \n", newSample.sequence, backgroundDeltaTimeUs, backgroundDeltaTimeUsAcc);

    if(oldSample.sequence != (newSample.sequence - 1)) {
        Serial.printf("Missing sample: %d\n", newSample.sequence);
        return;
    }
}

bool IMS::update()
{
    if(_computeToForegroundQueue == nullptr) {
        return false;
    }

    bool updated = false;
    IMSComputeResult result;

    while(xQueueReceive(_computeToForegroundQueue, &result, 0) == pdTRUE) {
        _foregroundComputeResult = result;
        if(_legacyIMU != nullptr) {
            _legacyIMU->update(result);
        }
        _foregroundSampleCount = result.sequence;
        updated = true;
    }

    return updated;
}

bool IMS::isRunning() const
{
    return _running;
}

IMSComputeResult IMS::getComputeResult() const
{
    return _foregroundComputeResult;
}

bool IMS::getComputeResult(IMSComputeResult& result) const
{
    return getLatestComputeResult(result);
}

bool IMS::getLatestComputeResult(IMSComputeResult& result) const
{
    if(_foregroundSampleCount > 0) {
        result = _foregroundComputeResult;
        return true;
    }

    return false;
}

LegacyIMU* IMS::getLegacyIMU()
{
    return _legacyIMU;
}

const LegacyIMU* IMS::getLegacyIMU() const
{
    return _legacyIMU;
}

uint32_t IMS::sampleCount() const
{
    return _foregroundSampleCount;
}

void IMS::_imuTaskLoop()
{
    // Start streaming to fifo
    _imu.enableFifo(true);
    _imu.startStreamingToFifo();

    while(_running) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t startTime = esp_timer_get_time();

        if(!_running) {
            continue;
        }

        //uint64_t startTime = esp_timer_get_time();
        //int ret = _imu->getAGTInterrupt();
        //int ret = _imu->getAGT();
        int ret = _imu.readFifo(IMU_SAMPLES_PER_FIFO_READ);
        // TOOD: test using interrupts instead of polling, now that we are doing bigger reads maybe the interrupt overhead is less significant?

        // // Calculate the start time of the fifo read
        // const uint32_t imuSamplePeriod_us = 1000000 / IMU_ODR_HZ;
        // uint64_t fifoStartTime = esp_timer_get_time() - (ret * imuSamplePeriod_us);

        do {
            IMSSample sample;
            sample.accX = _imu.accX();
            sample.accY = _imu.accY();
            sample.accZ = _imu.accZ();
            sample.gyrX = _imu.gyrX();
            sample.gyrY = _imu.gyrY();
            sample.gyrZ = _imu.gyrZ();
            sample.temp = _imu.temp();
            //sample.timestamp = _imu.timestamp();
            sample.sequence = ++_backgroundSampleCount;

            if(_IMUSampleQueue != nullptr) {
                uint8_t err = xQueueSend(_IMUSampleQueue, &sample, 0);
                // if(err != pdTRUE) {
                //     Serial.printf("IMS Sample Queue send error: %d\n", err);
                // }
            }

        } while(_imu.nextFifoSample());

        IMSCommand command;
        if(xQueueReceive(_IMUCommandQueue, &command, 0) == pdTRUE) {
            switch(command.command) {
                case IMSCommand::COMMAND_CALIBRATE:
                    _imu.calibrateGyro();
                    // Clear fifo to ensure we have a clean start
                    while(_imu.readFifo(IMU_SAMPLES_PER_FIFO_READ) != 0);
                    break;
                default:
                    Serial.printf("IMS Command: Unknown command: %d\n", command.command);
                    break;
            }
        }

        backgroundDeltaTimeUs = esp_timer_get_time() - startTime;
    }

    vTaskDelete(nullptr);
}

bool IMS::_computeSample(const IMSSample& sample, IMSComputeResult& result)
{
    result.accX = sample.accX;
    result.accY = sample.accY;
    result.accZ = sample.accZ;
    result.gyrX = sample.gyrX;
    result.gyrY = sample.gyrY;
    result.gyrZ = sample.gyrZ;
    result.imuTemp = sample.temp;

#if IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF
    float deltaT = 1.0f / IMU_ODR_HZ;

    xyz_t accelerometer = { sample.accY, sample.accX, sample.accZ };
    xyz_t gyroRPS = { -1 * sample.gyrY, -1 * sample.gyrX, sample.gyrZ };
    gyroRPS *= DEG2RAD;

#if IMU_ROTATE
    gyroRPS = imu_rot_apply_gyro_rad(gyroRPS);
    accelerometer = imu_rot_apply_accel(accelerometer);
#endif

    {
        Quaternion newOrientation = _sensorFusionFilter->updateOrientation(gyroRPS, accelerometer, deltaT);
        bool orientationFinite = isfinite(newOrientation.w) && isfinite(newOrientation.x) &&
                                  isfinite(newOrientation.y) && isfinite(newOrientation.z);
        if (orientationFinite) {
            _orientation = newOrientation;
        } else {
            // Sensor fusion diverged (e.g. a zero/invalid accelerometer sample
            // right at startup can make the filter's internal gravity
            // reference divide-by-zero). Recreate the filter so it doesn't
            // stay latched on NaN forever; keep reporting the last known-good
            // orientation in the meantime.
            delete _sensorFusionFilter;
#if IMU_SENSOR_FUSION_MODE == IMU_SENSOR_FUSION_MODE_MADGWICK
            _sensorFusionFilter = new MadgwickFilter();
            _sensorFusionFilter->setBeta(0.5F);
#elif IMU_SENSOR_FUSION_MODE == IMU_SENSOR_FUSION_MODE_VQF
            _sensorFusionFilter = new VQF(0.001f, 0.001f, 0.001f, true, true, false);
#endif
        }
    }
    //_orientation.normalize();
    result.orientation = _orientation;

    result._eulerOrientationValid = false;
    result._unitVectorOrientationValid = false;
    result._sphericalThetaValid = false;
    result._sphericalPhiValid = false;
    result._gyroMagnitudeSquaredValid = false;
#endif

#if IMU_ROTATE
    result.rot_accX = accelerometer.x;
    result.rot_accY = accelerometer.y;
    result.rot_accZ = accelerometer.z;
    gyroRPS *= (float)RAD2DEG;
    result.rot_gyrX = gyroRPS.x;
    result.rot_gyrY = gyroRPS.y;
    result.rot_gyrZ = gyroRPS.z;
#endif

    //Serial.printf("LegacyIMU is trivially copyable: %d\n", std::is_trivially_copyable_v<LegacyIMU>);

    // Update sphericalTheta_LP detection every 200hz
    // 1khz dt: 1ms alpha: 0.08 // original
    // 200hz dt: 5ms alpha: 0.341 // same cutoff as original
    // 100hz dt: 10ms alpha: 0.565 // same cutoff as original
    uint32_t updateInterval = IMU_ODR_HZ / IMS_SPHERICAL_THETA_LP_SAMPLE_RATE_HZ;
    if(sample.sequence % updateInterval == 0) {
        float theta_inst = result.sphericalTheta();
        result.sphericalTheta_LP = _sphericalTheta_LP.process(theta_inst);
    } else {
        result.sphericalTheta_LP = _sphericalTheta_LP.output();
    }

    // Gyro Magnitude Mean & Variance EMA
    float alpha = 0.0952; // Equivalent to 20 sample sliding window: alpha = 2 / (N + 1)
    float error = result.gyroMagnitudeSquared() - _gyroMagnitudeMean;
    _gyroMagnitudeMean = _gyroMagnitudeMean + (error * alpha);
    float error2 = result.gyroMagnitudeSquared() - _gyroMagnitudeMean;
    _gyroMagnitudeVariance = _gyroMagnitudeVariance + (alpha * (error * error2 - _gyroMagnitudeVariance));

    result.gyroMagnitudeMean = _gyroMagnitudeMean;
    result.gyroMagnitudeVariance = _gyroMagnitudeVariance;

    // Gyro Z Low Pass Filter
    result.gyrZ_LP = _gyrZ_LP.process(sample.gyrZ);

    return true;
}

void IMS::_computeTaskLoop()
{
    while(_running) {
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        IMSSample sample;
        static IMSSample lastSample;
        if(xQueueReceive(_IMUSampleQueue, &sample, portMAX_DELAY) == pdTRUE) {
            IMSComputeResult result;

            _diagnostics_sample_frequency(lastSample, sample);
            lastSample = sample;

            bool ret = _computeSample(sample, result);
            if(!ret) {
                // compute sample failed, skip writing it.
                Serial.printf("IMS Compute Sample error\n");
                continue;
            }

            if(xQueueOverwrite(_computeToForegroundQueue, &result) != pdTRUE) {
                Serial.printf("IMS Compute Queue send error\n");
            }
        }
    }
    vTaskDelete(nullptr);
}

bool IMS::_configureTimer()
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };

    if(gptimer_new_timer(&timer_config, &_gptimer) != ESP_OK) {
        _gptimer = nullptr;
        return false;
    }

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_TICKS_PER_SAMPLE,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true
        }
    };
    if(gptimer_set_alarm_action(_gptimer, &alarm_config) != ESP_OK) {
        gptimer_del_timer(_gptimer);
        _gptimer = nullptr;
        return false;
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = AlarmIsr,
    };
    if(gptimer_register_event_callbacks(_gptimer, &cbs, this) != ESP_OK) {
        gptimer_del_timer(_gptimer);
        _gptimer = nullptr;
        return false;
    }

    if(gptimer_enable(_gptimer) != ESP_OK) {
        gptimer_del_timer(_gptimer);
        _gptimer = nullptr;
        return false;
    }

    if(gptimer_start(_gptimer) != ESP_OK) {
        gptimer_disable(_gptimer);
        gptimer_del_timer(_gptimer);
        _gptimer = nullptr;
        return false;
    }

    return true;
}

void IMS::_clearLatestSample()
{
    _foregroundComputeResult = {};
    _foregroundSampleCount = 0;
    _backgroundSampleCount = 0;
}

void IMS::_sendIMUCommand(IMSCommand::Command command)
{
    IMSCommand cmd = {
        .command = command,
    };
    xQueueSend(_IMUCommandQueue, &cmd, 0);
}

void IMS::imuCalibrateGyro()
{
    _sendIMUCommand(IMSCommand::COMMAND_CALIBRATE);
}