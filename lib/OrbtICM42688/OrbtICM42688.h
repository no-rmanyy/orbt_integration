#ifndef ORBT_ICM42688_H
#define ORBT_ICM42688_H

#include "Arduino.h"
#include "SPI.h"     // SPI library

#include "driver/spi_master.h"

class OrbtICM42688
{
  public:

    enum GyroFS : uint8_t {
      dps2000 = 0x00,
      dps1000 = 0x01,
      dps500 = 0x02,
      dps250 = 0x03,
      dps125 = 0x04,
      dps62_5 = 0x05,
      dps31_25 = 0x06,
      dps15_625 = 0x07
    };

    enum AccelFS : uint8_t {
      gpm16 = 0x00,
      gpm8 = 0x01,
      gpm4 = 0x02,
      gpm2 = 0x03
    };

    enum ODR : uint8_t {
      odr32k = 0x01, // LN mode only
      odr16k = 0x02, // LN mode only
      odr8k = 0x03, // LN mode only
      odr4k = 0x04, // LN mode only
      odr2k = 0x05, // LN mode only
      odr1k = 0x06, // LN mode only
      odr200 = 0x07,
      odr100 = 0x08,
      odr50 = 0x09,
      odr25 = 0x0A,
      odr12_5 = 0x0B,
      odr6a25 = 0x0C, // LP mode only (accel only)
      odr3a125 = 0x0D, // LP mode only (accel only)
      odr1a5625 = 0x0E, // LP mode only (accel only)
      odr500 = 0x0F,
    };

    enum GyroNFBWsel : uint8_t {
      nfBW1449Hz = 0x00,
      nfBW680Hz = 0x01,
      nfBW329Hz = 0x02,
      nfBW162Hz = 0x03,
      nfBW80Hz = 0x04,
      nfBW40Hz = 0x05,
      nfBW20Hz = 0x06,
      nfBW10Hz = 0x07,
    };

    enum UIFiltOrd : uint8_t {
      first_order = 0x00,
      second_order = 0x01,
      third_order = 0x02,
    };

    /**
     * @brief      Constructor for SPI communication
     *
     * @param      bus    SPI bus
     * @param[in]  csPin  Chip Select pin
     */
    //OrbtICM42688(SPIClass &bus, uint8_t csPin, uint32_t SPI_HS_CLK=8000000);

    /**
     * @brief      Constructor for SPI communication
     * @param[in]  csPin  Chip Select pin
     * @param[in]  sckPin  Clock pin
     * @param[in]  misoPin  Master In Slave Out pin
     * @param[in]  mosiPin  Master Out Slave In pin
     */
    OrbtICM42688(gpio_num_t csPin, gpio_num_t sckPin, gpio_num_t misoPin, gpio_num_t mosiPin);

    /**
     * @brief      Initialize the device.
     *
     * @return     ret < 0 if error
     */
    int begin();

    /**
     * @brief      Sets the full scale range for the accelerometer
     *
     * @param[in]  fssel  Full scale selection
     *
     * @return     ret < 0 if error
     */
    int setAccelFS(AccelFS fssel);

    /**
     * @brief      Sets the full scale range for the gyro
     *
     * @param[in]  fssel  Full scale selection
     *
     * @return     ret < 0 if error
     */
    int setGyroFS(GyroFS fssel);

    /**
     * @brief      Set the ODR for accelerometer
     *
     * @param[in]  odr   Output data rate
     *
     * @return     ret < 0 if error
     */
    int setAccelODR(ODR odr);

    /**
     * @brief      Set the ODR for gyro
     *
     * @param[in]  odr   Output data rate
     *
     * @return     ret < 0 if error
     */
    int setGyroODR(ODR odr);

    int setFilters(bool gyroFilters, bool accFilters);

    /**
     * @brief      Enables the data ready interrupt.
     *
     *             - routes UI data ready interrupt to INT1
     *             - push-pull, pulsed, active HIGH interrupts
     *
     * @return     ret < 0 if error
     */
    int enableDataReadyInterrupt();

    /**
     * @brief      Masks the data ready interrupt
     *
     * @return     ret < 0 if error
     */
    int disableDataReadyInterrupt();

    /**
     * @brief      Transfers data from ICM 42688-p to microcontroller.
     *             Must be called to access new measurements.
     *
     * @return     ret < 0 if error
     */
    int getAGT();

    /**
     * @brief      Get accelerometer data, per axis
     *
     * @return     Acceleration in g's
     */
    float accX() const {
      return _acc[0];
    }

    float accY() const {
#if ORBT_PCB_VERSION == 50
      return -1 * _acc[1];
#else
      return _acc[1];
#endif
    }

    float accZ() const {
#if ORBT_PCB_VERSION == 50
      return -1 * _acc[2];
#else
      return _acc[2];
#endif
    }

    /**
     * @brief      Get gyro data, per axis
     *
     * @return     Angular velocity in dps
     */
    float gyrX() const {
      return _gyr[0];
    }

    float gyrY() const {
#if ORBT_PCB_VERSION == 50
      return -1 * _gyr[1];
#else
      return _gyr[1];
#endif
    }

    float gyrZ() const {
#if ORBT_PCB_VERSION == 50
      return -1 * _gyr[2];
#else
      return _gyr[2];
#endif
    }

    /**
     * @brief      Get temperature of gyro die
     *
     * @return     Temperature in Celsius
     */
    float temp() const { return _t; }

    int calibrateGyro();
    float getGyroBiasX();
    float getGyroBiasY();
    float getGyroBiasZ();
    void setGyroBiasX(float bias);
    void setGyroBiasY(float bias);
    void setGyroBiasZ(float bias);
    int calibrateAccel();
    float getAccelBiasX_mss();
    float getAccelScaleFactorX();
    float getAccelBiasY_mss();
    float getAccelScaleFactorY();
    float getAccelBiasZ_mss();
    float getAccelScaleFactorZ();
    void setAccelCalX(float bias,float scaleFactor);
    void setAccelCalY(float bias,float scaleFactor);
    void setAccelCalZ(float bias,float scaleFactor);
  
    int getAGTInterrupt();

  protected:
    float _accX() const { return _acc[0]; }
    float _accY() const { return _acc[1]; }
    float _accZ() const { return _acc[2]; }
    float _gyrX() const { return _gyr[0]; }
    float _gyrY() const { return _gyr[1]; }
    float _gyrZ() const { return _gyr[2]; }

    ///\brief IDF SPI Communication
    static constexpr uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
    uint32_t SPI_HS_CLOCK = 24000000; // 24 MHz

    gpio_num_t _csPin;
    gpio_num_t _sckPin;
    gpio_num_t _misoPin;
    gpio_num_t _mosiPin;

    bool _usingHS;

    spi_device_interface_config_t dev_ls_config;
    spi_device_interface_config_t dev_hs_config;
    spi_device_handle_t _spi_device_handle;
    bool _spi_device_acquired = false;

    // buffer for reading from sensor
    uint8_t _buffer[15] = {};

    uint8_t _interruptTransferInprogress = false;
    uint8_t _interruptResultReady = false;
    spi_transaction_t _t1, _t2;

    // data buffer
    float _t = 0.0f;
    float _acc[3] = {};
    float _gyr[3] = {};

    ///\brief Full scale resolution factors
    float _accelScale = 0.0f;
    float _gyroScale = 0.0f;

    ///\brief Full scale selections
    AccelFS _accelFS = gpm16;
    GyroFS _gyroFS = dps2000;

    ///\brief Accel calibration
    float _accBD[3] = {};
    float _accB[3] = {};
    float _accS[3] = {1.0f, 1.0f, 1.0f};
    float _accMax[3] = {};
    float _accMin[3] = {};

    ///\brief Gyro calibration
    float _gyroBD[3] = {};
    float _gyrB[3] = {};

    ///\brief Constants
    static constexpr uint8_t WHO_AM_I = 0x47; ///< expected value in UB0_REG_WHO_AM_I reg
    static constexpr int NUM_CALIB_SAMPLES = 1000; ///< for gyro/accel bias calib

    ///\brief Conversion formula to get temperature in Celsius (Sec 4.13)
    static constexpr float TEMP_DATA_REG_SCALE_2B = 132.48f;
    static constexpr float TEMP_DATA_REG_SCALE_1B = 2.07;
    static constexpr float TEMP_OFFSET = 25.0f;

    uint8_t _bank = 0; ///< current user bank

    const uint8_t FIFO_EN = 0x5F;
    const uint8_t FIFO_HIRES_EN = 0x10;
    const uint8_t FIFO_TEMP_EN = 0x04;
    const uint8_t FIFO_GYRO = 0x02;
    const uint8_t FIFO_ACCEL = 0x01;
    // const uint8_t FIFO_COUNT = 0x2E;
    // const uint8_t FIFO_DATA = 0x30;

    // BANK 1
    // const uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
    const uint8_t GYRO_NF_ENABLE = 0x00;
    const uint8_t GYRO_NF_DISABLE = 0x01;
    const uint8_t GYRO_AAF_ENABLE = 0x00;
    const uint8_t GYRO_AAF_DISABLE = 0x02;

    // BANK 2
    // const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
    const uint8_t ACCEL_AAF_ENABLE = 0x00;
    const uint8_t ACCEL_AAF_DISABLE = 0x01;

    // private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int readRegisters3Wire(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int readRegisters4Wire(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int setBank(uint8_t bank);
    void useHS(bool useHS);

    void threeWireSPIWrite(bool write);
    static void pre_cb(spi_transaction_t *trans);
    static void post_cb(spi_transaction_t *trans);
    int readRegistersInterrupt(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int readRegisters3WireInterrupt(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int readRegisters4WireInterrupt(uint8_t subAddress, uint8_t count, uint8_t* dest);

    bool spi_device_acquire();
    void spi_device_release();

    /**
     * @brief      Software reset of the device
     */
    void reset();

    /**
     * @brief      Read the WHO_AM_I register
     *
     * @return     Value of WHO_AM_I register
     */
    uint8_t whoAmI();
};

class OrbtICM42688_FIFO: public OrbtICM42688 {
  public:
    using OrbtICM42688::OrbtICM42688;

    typedef struct {
      uint8_t header;
      uint8_t accel[6];
      uint8_t gyro[6];
      uint8_t temp[1];
      uint8_t timestamp[2];
    } fifo_frame_t;

    typedef struct {
      uint8_t header;
      uint8_t accel[6];
      uint8_t gyro[6];
      uint8_t temp[2];
      uint8_t timestamp[2];
      uint8_t highres_extension[3];
    } fifo_frame_highres_t;

    int enableFifo(bool hires = false);
    int startStreamingToFifo();
    int stopStreamingToFifo();

    uint8_t readFifo(uint8_t sampleCount);
    uint8_t getFifoSampleCount();
    bool setFifoSample(uint8_t sampleIdx);
    bool nextFifoSample();

    uint16_t getFifoTimestamp() const { return _fifoTimestamp; }

  protected:
    const uint8_t FIFO_HEADER_MSG = 0x80;

    // fifo
    uint16_t _fifoTimestamp = 0;
    size_t _hires_en = 0;
    size_t _fifoFrameSize = 0;

    uint8_t *_fifoBuffer = nullptr;
    size_t _fifoBufferSize = 0;
    uint8_t _fifoBufferSampleCount = 0;
    uint8_t _fifoBufferSampleIdx = 0;
    uint8_t *_allocateFifoBuffer(uint8_t sampleCount);
    void _updateFifoSample();
    static int32_t _signExtend20(uint32_t value);
};

#endif // ICM42688_H
