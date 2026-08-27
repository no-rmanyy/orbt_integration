#include "Arduino.h"
#include "OrbtICM42688.h"
#include "registers.h"

#include "esp_rom_gpio.h"
#include "soc/spi_periph.h"

using namespace ICM42688reg;

// For 3 wire SPI: MOSI to -1
OrbtICM42688::OrbtICM42688(gpio_num_t csPin, gpio_num_t sckPin, gpio_num_t misoPin, gpio_num_t mosiPin) {
  _csPin = csPin;
  _sckPin = sckPin;
  _misoPin = misoPin;
  _mosiPin = mosiPin;
}

/* starts communication with the ICM42688 */
int OrbtICM42688::begin() {
  esp_err_t esp_ret;

  spi_bus_config_t bus_config = {
    .mosi_io_num = _mosiPin,
    .miso_io_num = _misoPin,
    .sclk_io_num = _sckPin,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 32,
    .flags = SPICOMMON_BUSFLAG_GPIO_PINS
  };

  dev_hs_config = {
    .mode = 0,                            //SPI mode 0/3 default
    .clock_speed_hz = (int)SPI_HS_CLOCK,  //Clock out at 8 MHz
    .spics_io_num = _csPin,               //CS pin
    .queue_size = 7                       //We want to be able to queue 7 transactions at a time  
  };
  dev_hs_config.pre_cb = pre_cb;
  dev_hs_config.post_cb = post_cb;

  dev_ls_config = {
    .mode = 0,                            //SPI mode 0/3 default
    .clock_speed_hz = SPI_LS_CLOCK,       //Clock out at 1 MHz
    .spics_io_num = _csPin,               //CS pin
    .queue_size = 7                       //We want to be able to queue 7 transactions at a time  

  };
  dev_ls_config.pre_cb = pre_cb;
  dev_ls_config.post_cb = post_cb;

  esp_ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
  if(esp_ret != ESP_OK) {
    return -1;
  }

  // use low speed SPI for register setting
  _usingHS = false;
  esp_ret = spi_bus_add_device(SPI2_HOST, &dev_ls_config, &_spi_device_handle);
  if(esp_ret != ESP_OK) {
    return -2;
  }

  // reset the ICM42688
  reset();

  // if we're using three wire SPI, configure it.
  if(_mosiPin == GPIO_NUM_NC) {
    setBank(1);
    writeRegister(UB1_REG_INTF_CONFIG4, 0);
  }

  // check the WHO AM I byte
  if(whoAmI() != WHO_AM_I) {
    return -4;
  }

  // turn on accel and gyro in Low Noise (LN) Mode
  if(writeRegister(UB0_REG_PWR_MGMT0, 0x0F) < 0) {
    return -5;
  }

  // 16G is default -- do this to set up accel resolution scaling
  int ret = setAccelFS(gpm16);
  if (ret < 0) return ret;

  // 2000DPS is default -- do this to set up gyro resolution scaling
  ret = setGyroFS(dps2000);
  if (ret < 0) return ret;

  // // disable inner filters (Notch filter, Anti-alias filter, UI filter block)
  // if (setFilters(false, false) < 0) {
  //   return -6;
  // }

  // estimate gyro bias
  if (calibrateGyro() < 0) {
    return -7;
  }
  // successful init, return 1
  return 1;
}

/* sets the accelerometer full scale range to values other than default */
int OrbtICM42688::setAccelFS(AccelFS fssel) {
  // use low speed SPI for register setting
  useHS(false);

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;

  _accelScale = static_cast<float>(1 << (4 - fssel)) / 32768.0f;
  _accelFS = fssel;

  return 1;
}

/* sets the gyro full scale range to values other than default */
int OrbtICM42688::setGyroFS(GyroFS fssel) {
  // use low speed SPI for register setting
  useHS(false);

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;

  _gyroScale = (2000.0f / static_cast<float>(1 << fssel)) / 32768.0f;
  _gyroFS = fssel;

  return 1;
}

int OrbtICM42688::setAccelODR(ODR odr) {
  // use low speed SPI for register setting
  useHS(false);

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;

  return 1;
}

int OrbtICM42688::setGyroODR(ODR odr) {
  // use low speed SPI for register setting
  useHS(false);

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;

  return 1;
}

int OrbtICM42688::setFilters(bool gyroFilters, bool accFilters) {
  if (setBank(1) < 0) return -1;

  if (gyroFilters == true) {
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE) < 0) {
      return -2;
    }
  }
  else {
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0) {
      return -3;
    }
  }
  
  if (setBank(2) < 0) return -4;

  if (accFilters == true) {
    if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE) < 0) {
      return -5;
    }
  }
  else {
    if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE) < 0) {
      return -6;
    }
  }
  if (setBank(0) < 0) return -7;
  return 1;
}

int OrbtICM42688::enableDataReadyInterrupt() {
  // use low speed SPI for register setting
  useHS(false);

  // push-pull, pulsed, active HIGH interrupts
  if (writeRegister(UB0_REG_INT_CONFIG, 0x18 | 0x03) < 0) return -1;

  // need to clear bit 4 to allow proper INT1 and INT2 operation
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) return -2;
  reg &= ~0x10;
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) return -3;

  // route UI data ready interrupt to INT1
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x18) < 0) return -4;

  return 1;
}

int OrbtICM42688::disableDataReadyInterrupt() {
  // use low speed SPI for register setting
  useHS(false);

  // set pin 4 to return to reset value
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) return -1;
  reg |= 0x10;
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) return -2;

  // return reg to reset value
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x10) < 0) return -3;

  return 1;
}

int OrbtICM42688::getAGTInterrupt() 
{
  int ret = 0;

  // Check if previous transaction completed and copy data.
  if(_interruptResultReady) {
    // 3 & 4 wire modes have at least one transaction
    spi_transaction_t* trans1 = &_t1;
    spi_device_get_trans_result(_spi_device_handle, &trans1, portMAX_DELAY);

    if(_mosiPin == GPIO_NUM_NC) {
      spi_transaction_t* trans2 = &_t2;
      spi_device_get_trans_result(_spi_device_handle, &trans2, portMAX_DELAY);
    }

    //spi_device_release();
    _interruptResultReady = false;

    int16_t rawMeas[7]; // temp, accel xyz, gyro xyz
    for (size_t i=0; i<7; i++) {
      rawMeas[i] = ((int16_t)_buffer[i*2] << 8) | _buffer[i*2+1];
    }

    _t = (static_cast<float>(rawMeas[0]) / TEMP_DATA_REG_SCALE_2B) + TEMP_OFFSET;

    _acc[0] = ((rawMeas[1] * _accelScale) - _accB[0]) * _accS[0];
    _acc[1] = ((rawMeas[2] * _accelScale) - _accB[1]) * _accS[1];
    _acc[2] = ((rawMeas[3] * _accelScale) - _accB[2]) * _accS[2];
  
    _gyr[0] = (rawMeas[4] * _gyroScale) - _gyrB[0];
    _gyr[1] = (rawMeas[5] * _gyroScale) - _gyrB[1];
    _gyr[2] = (rawMeas[6] * _gyroScale) - _gyrB[2];

    ret = 1;
  }

  if(!_interruptTransferInprogress) {
    // use the high speed SPI for data readout
    useHS(true);
    // grab the data from the ICM42688
    if (readRegistersInterrupt(UB0_REG_TEMP_DATA1, 14, _buffer) < 0) 
      return -1;
  }

  return ret;
}

/* reads the most current data from ICM42688 and stores in buffer */
int OrbtICM42688::getAGT() {
  // use the high speed SPI for data readout
  useHS(true);
  // grab the data from the ICM42688
  if (readRegisters(UB0_REG_TEMP_DATA1, 14, _buffer) < 0) return -1;

  // combine bytes into 16 bit values
  int16_t rawMeas[7]; // temp, accel xyz, gyro xyz
  for (size_t i=0; i<7; i++) {
    rawMeas[i] = ((int16_t)_buffer[i*2] << 8) | _buffer[i*2+1];
  }

  _t = (static_cast<float>(rawMeas[0]) / TEMP_DATA_REG_SCALE_2B) + TEMP_OFFSET;

  _acc[0] = ((rawMeas[1] * _accelScale) - _accB[0]) * _accS[0];
  _acc[1] = ((rawMeas[2] * _accelScale) - _accB[1]) * _accS[1];
  _acc[2] = ((rawMeas[3] * _accelScale) - _accB[2]) * _accS[2];

  _gyr[0] = (rawMeas[4] * _gyroScale) - _gyrB[0];
  _gyr[1] = (rawMeas[5] * _gyroScale) - _gyrB[1];
  _gyr[2] = (rawMeas[6] * _gyroScale) - _gyrB[2];

  return 1;
}

/* configures and enables the FIFO buffer
 Enforces best-fitting structure (1, 2, or 3) but cannot replicate Packet 4 as choosing between 3/4 requires
 additional argument for high-resolution

 See https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf, 6.1 for details
*/
int OrbtICM42688_FIFO::enableFifo(bool hires) {
  bool enFifoAccel = true;
  bool enFifoGyro  = true;
  bool enFifoTemp  = true;  // all structures have 1-byte temp, didn't return error to maintain backwards compatibility
  bool enFifoTimestamp = enFifoAccel && enFifoGyro; // can only read both accel and gyro in Structure 3 or 4, both have 2-byte timestamp
  bool enFifoHeader  = enFifoAccel || enFifoGyro;  // if neither sensor requested, FIFO will not send any more packets
  _hires_en = hires;

  // Standard frame size: 1 header + 6 accel + 6 gyro + 1 temp + 2 timestamp = 16 bytes
  // High-resolution frame size: 1 header + 6 accel + 6 gyro + 2 temp + 2 timestamp + 3 highres extension = 20 bytes
  _fifoFrameSize = enFifoHeader * 1 + enFifoAccel * 6 + enFifoGyro * 6 + (enFifoTemp + hires) + enFifoTimestamp * 2 + (hires * 3);

  // use low speed SPI for register setting
  useHS(false);
  if (writeRegister(FIFO_EN, (enFifoAccel * FIFO_ACCEL) | (enFifoGyro * FIFO_GYRO) | (enFifoTemp * FIFO_TEMP_EN) | (hires * FIFO_HIRES_EN)) < 0) {
    return -2;
  }

  return 1;
}

uint8_t *OrbtICM42688_FIFO::_allocateFifoBuffer(uint8_t sampleCount)
{
  if(sampleCount == _fifoBufferSize) {
    return _fifoBuffer;
  }

  if(_fifoBuffer != nullptr) {
    free(_fifoBuffer);
    _fifoBuffer = nullptr;
    _fifoBufferSize = 0;
  }

  _fifoBufferSize = sampleCount;

  Serial.printf("OrbtICM42688_FIFO Allocating %d bytes for %d samples\n", _fifoBufferSize * _fifoFrameSize, _fifoBufferSize);
  _fifoBuffer = (uint8_t *)calloc(_fifoBufferSize * _fifoFrameSize, sizeof(uint8_t));

  return _fifoBuffer;
}

/* Start streaming, required to read after enableFifo() under most sensor configurations */
int OrbtICM42688_FIFO::startStreamingToFifo() {
  useHS(false);

  uint8_t reg;

  if (readRegisters(ICM42688reg::UB0_REG_FIFO_CONFIG, 1, &reg) < 0) {
    return -1;
  }

  // Set Start Bit
  reg |= 1 << 6;
  if (writeRegister(ICM42688reg::UB0_REG_FIFO_CONFIG, reg) < 0) {
    return -2;
  }

  return 1;
}

int OrbtICM42688_FIFO::stopStreamingToFifo() {
  useHS(false);

  uint8_t reg;

  if (readRegisters(ICM42688reg::UB0_REG_FIFO_CONFIG, 1, &reg) < 0) {
    return -1;
  }

  // Clear Start Bit
  reg &= ~(1 << 6);
  if (writeRegister(ICM42688reg::UB0_REG_FIFO_CONFIG, reg) < 0) {
    return -2;
  }

  return 1;
}


/* Reads a batch of samples */
uint8_t OrbtICM42688_FIFO::readFifo(uint8_t sampleCount) {
  useHS(true); // use the high speed SPI for data readout

  uint8_t *sampleBuffer = _allocateFifoBuffer(sampleCount);
  if(sampleBuffer == nullptr) {
    return -1;
  }

  if(readRegisters(UB0_REG_FIFO_DATA, sampleCount * _fifoFrameSize, sampleBuffer) < 0) {
    Serial.printf("Failed to read FIFO data from IMU\n");
    return -1;
  }

  // Count how many valid samples were read
  if (_hires_en) {
    OrbtICM42688_FIFO::fifo_frame_highres_t *frame = (OrbtICM42688_FIFO::fifo_frame_highres_t *)sampleBuffer;

    _fifoBufferSampleCount = 0;
    _fifoBufferSampleIdx = 0;
    for(size_t i = 0; i < sampleCount; i++) {
      if(!(frame[i].header & FIFO_HEADER_MSG)) {
        _fifoBufferSampleCount++;
      }
    }
  } else {
    OrbtICM42688_FIFO::fifo_frame_t *frame = (OrbtICM42688_FIFO::fifo_frame_t *)sampleBuffer;

    _fifoBufferSampleCount = 0;
    _fifoBufferSampleIdx = 0;
    for(size_t i = 0; i < sampleCount; i++) {
      if(!(frame[i].header & FIFO_HEADER_MSG)) {
        _fifoBufferSampleCount++;
      }
    }
  }

  if(_fifoBufferSampleCount > 0) {
    _updateFifoSample();
  }

  return _fifoBufferSampleCount;
}

uint8_t OrbtICM42688_FIFO::getFifoSampleCount() {
  return _fifoBufferSampleCount;
}

int32_t OrbtICM42688_FIFO::_signExtend20(uint32_t value)
{
  return (value & 0x80000) ? (int32_t)(value | 0xFFF00000) : (int32_t)value;
}

void OrbtICM42688_FIFO::_updateFifoSample() {
  if(_hires_en) {
    OrbtICM42688_FIFO::fifo_frame_highres_t *frame = (OrbtICM42688_FIFO::fifo_frame_highres_t *)_fifoBuffer;
    OrbtICM42688_FIFO::fifo_frame_highres_t *sample = &frame[_fifoBufferSampleIdx];

    // in 20 bit mode, the accel and gyro scales are fixed
    float accelScale = static_cast<float>(1 << (4 - 2)) / 32768.0f;
    float gyroScale = (2000.0f / static_cast<float>(1 << 3)) / 32768.0f;

    // handle accelerometer data
    int32_t rawMeas[3];
    rawMeas[0] = _signExtend20((((uint32_t)sample->accel[0]) << 12) | (sample->accel[1] << 4) | (sample->highres_extension[0] >> 4));
    rawMeas[1] = _signExtend20((((uint32_t)sample->accel[2]) << 12) | (sample->accel[3] << 4) | (sample->highres_extension[1] >> 4));
    rawMeas[2] = _signExtend20((((uint32_t)sample->accel[4]) << 12) | (sample->accel[5] << 4) | (sample->highres_extension[2] >> 4));

    // FIXME: / TODO:note to self we don't use the _accS values, we can't probably remove them here.
    _acc[0] = (((rawMeas[0] >> 2) * accelScale) - _accB[0]) * _accS[0];
    _acc[1] = (((rawMeas[1] >> 2) * accelScale) - _accB[1]) * _accS[1];
    _acc[2] = (((rawMeas[2] >> 2) * accelScale) - _accB[2]) * _accS[2];

    // handle gyroscope data
    rawMeas[0] = _signExtend20((((uint32_t)sample->gyro[0]) << 12) | (sample->gyro[1] << 4) | (sample->highres_extension[0] & 0xF));
    rawMeas[1] = _signExtend20((((uint32_t)sample->gyro[2]) << 12) | (sample->gyro[3] << 4) | (sample->highres_extension[1] & 0xF));
    rawMeas[2] = _signExtend20((((uint32_t)sample->gyro[4]) << 12) | (sample->gyro[5] << 4) | (sample->highres_extension[2] & 0xF));
    _gyr[0] = (((rawMeas[0] >> 1) * gyroScale) - _gyrB[0]);
    _gyr[1] = (((rawMeas[1] >> 1) * gyroScale) - _gyrB[1]);
    _gyr[2] = (((rawMeas[2] >> 1) * gyroScale) - _gyrB[2]);

    // handle temperature data // TODO: / FIXME: we probably don't need this in production
    rawMeas[0] = sample->temp[0] << 8 | sample->temp[1];
    _t = (static_cast<float>(rawMeas[0]) / TEMP_DATA_REG_SCALE_2B) + TEMP_OFFSET;

    // handle timestamp data // TODO: / FIXME: we probably don't need this in production
    _fifoTimestamp = (sample->timestamp[0] << 8) | sample->timestamp[1];

  } else {
    OrbtICM42688_FIFO::fifo_frame_t *frame = (OrbtICM42688_FIFO::fifo_frame_t *)_fifoBuffer;
    OrbtICM42688_FIFO::fifo_frame_t *sample = &frame[_fifoBufferSampleIdx];

    // handle accelerometer data
    int16_t rawMeas[3];
    rawMeas[0] = (((int16_t)sample->accel[0]) << 8) | sample->accel[1];
    rawMeas[1] = (((int16_t)sample->accel[2]) << 8) | sample->accel[3];
    rawMeas[2] = (((int16_t)sample->accel[4]) << 8) | sample->accel[5];

    // FIXME: / TODO:note to self we don't use the _accS values, we can't probably remove them here.
    _acc[0] = ((rawMeas[0] * _accelScale) - _accB[0]) * _accS[0];
    _acc[1] = ((rawMeas[1] * _accelScale) - _accB[1]) * _accS[1];
    _acc[2] = ((rawMeas[2] * _accelScale) - _accB[2]) * _accS[2];

    // handle gyroscope data
    rawMeas[0] = (((int16_t)sample->gyro[0]) << 8) | sample->gyro[1];
    rawMeas[1] = (((int16_t)sample->gyro[2]) << 8) | sample->gyro[3];
    rawMeas[2] = (((int16_t)sample->gyro[4]) << 8) | sample->gyro[5];
    _gyr[0] = (rawMeas[0] * _gyroScale) - _gyrB[0];
    _gyr[1] = (rawMeas[1] * _gyroScale) - _gyrB[1];
    _gyr[2] = (rawMeas[2] * _gyroScale) - _gyrB[2];

    // handle temperature data
    // rawMeas[0] = sample->temp[0];
    _t = (static_cast<float>(rawMeas[0]) / TEMP_DATA_REG_SCALE_1B) + TEMP_OFFSET;

    // // handle timestamp data
    _fifoTimestamp = (sample->timestamp[0] << 8) | sample->timestamp[1];
  }

}

bool OrbtICM42688_FIFO::setFifoSample(uint8_t sampleIdx) {
  if(sampleIdx >= _fifoBufferSampleCount) {
    return false;
  }

  _fifoBufferSampleIdx = sampleIdx;
  _updateFifoSample();
  return true;
}

bool OrbtICM42688_FIFO::nextFifoSample() {
  _fifoBufferSampleIdx++;
  if(_fifoBufferSampleIdx >= _fifoBufferSampleCount) {
    _fifoBufferSampleIdx--;
    return false;
  }

  _updateFifoSample();
  return true;
}

/* estimates the gyro biases */
int OrbtICM42688::calibrateGyro() {
  // set at a lower range (more resolution) since IMU not moving
  const GyroFS current_fssel = _gyroFS;
  //if (setGyroFS(dps15_625) < 0) return -1; // experiment with highest resolution
  if (setGyroFS(dps250) < 0) return -1;

  // take samples and find bias
  double gyroBD[3] = {0, 0, 0};

  // reset gyro biases
  _gyrB[0] = 0;
  _gyrB[1] = 0;
  _gyrB[2] = 0;

  for (size_t i=0; i < NUM_CALIB_SAMPLES; i++) {
    getAGT();
    gyroBD[0] += _gyrX();
    gyroBD[1] += _gyrY();
    gyroBD[2] += _gyrZ();

    delay(1);
  }

  gyroBD[0] /= NUM_CALIB_SAMPLES;
  gyroBD[1] /= NUM_CALIB_SAMPLES;
  gyroBD[2] /= NUM_CALIB_SAMPLES;

  _gyrB[0] = gyroBD[0];
  _gyrB[1] = gyroBD[1];
  _gyrB[2] = gyroBD[2];

  Serial.printf("Gyro biases: %f, %f, %f\n", _gyrB[0], _gyrB[1], _gyrB[2]);

  // recover the full scale setting
  if (setGyroFS(current_fssel) < 0) return -4;
  return 1;
}

/* returns the gyro bias in the X direction, dps */
float OrbtICM42688::getGyroBiasX() {
  return _gyrB[0];
}

/* returns the gyro bias in the Y direction, dps */
float OrbtICM42688::getGyroBiasY() {
  return _gyrB[1];
}

/* returns the gyro bias in the Z direction, dps */
float OrbtICM42688::getGyroBiasZ() {
  return _gyrB[2];
}

/* sets the gyro bias in the X direction to bias, dps */
void OrbtICM42688::setGyroBiasX(float bias) {
  _gyrB[0] = bias;
}

/* sets the gyro bias in the Y direction to bias, dps */
void OrbtICM42688::setGyroBiasY(float bias) {
  _gyrB[1] = bias;
}

/* sets the gyro bias in the Z direction to bias, dps */
void OrbtICM42688::setGyroBiasZ(float bias) {
  _gyrB[2] = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int OrbtICM42688::calibrateAccel() {
  // set at a lower range (more resolution) since IMU not moving
  const AccelFS current_fssel = _accelFS;
  if (setAccelFS(gpm2) < 0) return -1;

  // take samples and find min / max
  _accBD[0] = 0;
  _accBD[1] = 0;
  _accBD[2] = 0;
  for (size_t i=0; i < NUM_CALIB_SAMPLES; i++) {
    getAGT();
    _accBD[0] += (_accX()/_accS[0] + _accB[0]) / NUM_CALIB_SAMPLES;
    _accBD[1] += (_accY()/_accS[1] + _accB[1]) / NUM_CALIB_SAMPLES;
    _accBD[2] += (_accZ()/_accS[2] + _accB[2]) / NUM_CALIB_SAMPLES;
    delay(1);
  }
  if (_accBD[0] > 0.9f) {
    _accMax[0] = _accBD[0];
  }
  if (_accBD[1] > 0.9f) {
    _accMax[1] = _accBD[1];
  }
  if (_accBD[2] > 0.9f) {
    _accMax[2] = _accBD[2];
  }
  if (_accBD[0] < -0.9f) {
    _accMin[0] = _accBD[0];
  }
  if (_accBD[1] < -0.9f) {
    _accMin[1] = _accBD[1];
  }
  if (_accBD[2] < -0.9f) {
    _accMin[2] = _accBD[2];
  }

  // find bias and scale factor
  if ((abs(_accMin[0]) > 0.9f) && (abs(_accMax[0]) > 0.9f)) {
    _accB[0] = (_accMin[0] + _accMax[0]) / 2.0f;
    _accS[0] = 1/((abs(_accMin[0]) + abs(_accMax[0])) / 2.0f);
  }
  if ((abs(_accMin[1]) > 0.9f) && (abs(_accMax[1]) > 0.9f)) {
    _accB[1] = (_accMin[1] + _accMax[1]) / 2.0f;
    _accS[1] = 1/((abs(_accMin[1]) + abs(_accMax[1])) / 2.0f);
  }
  if ((abs(_accMin[2]) > 0.9f) && (abs(_accMax[2]) > 0.9f)) {
    _accB[2] = (_accMin[2] + _accMax[2]) / 2.0f;
    _accS[2] = 1/((abs(_accMin[2]) + abs(_accMax[2])) / 2.0f);
  }

  // recover the full scale setting
  if (setAccelFS(current_fssel) < 0) return -4;
  return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float OrbtICM42688::getAccelBiasX_mss() {
  return _accB[0];
}

/* returns the accelerometer scale factor in the X direction */
float OrbtICM42688::getAccelScaleFactorX() {
  return _accS[0];
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float OrbtICM42688::getAccelBiasY_mss() {
  return _accB[1];
}

/* returns the accelerometer scale factor in the Y direction */
float OrbtICM42688::getAccelScaleFactorY() {
  return _accS[1];
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float OrbtICM42688::getAccelBiasZ_mss() {
  return _accB[2];
}

/* returns the accelerometer scale factor in the Z direction */
float OrbtICM42688::getAccelScaleFactorZ() {
  return _accS[2];
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void OrbtICM42688::setAccelCalX(float bias,float scaleFactor) {
  _accB[0] = bias;
  _accS[0] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void OrbtICM42688::setAccelCalY(float bias,float scaleFactor) {
  _accB[1] = bias;
  _accS[1] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void OrbtICM42688::setAccelCalZ(float bias,float scaleFactor) {
  _accB[2] = bias;
  _accS[2] = scaleFactor;
}

/* writes a byte to ICM42688 register given a register address and data */
int OrbtICM42688::writeRegister(uint8_t subAddress, uint8_t data) {
  esp_err_t ret;
  spi_transaction_t t;
  uint8_t cmd[2] = {subAddress, data};

  // Zero out the transaction
  memset(&t, 0, sizeof(t));

  threeWireSPIWrite(true);

  t.length = 16;                     //Command is 16 bits
  t.tx_buffer = &cmd;                //The data is the cmd itself
  t.user = (void*)0;                 //D/C needs to be set to 0

  ret = spi_device_polling_transmit(_spi_device_handle, &t);

  if(ret != ESP_OK) {
    return -1;
  }

  delay(10);

  readRegisters(subAddress, 1, _buffer);
  
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }  
}

void OrbtICM42688::useHS(bool useHS) {
  if(useHS != _usingHS) {
    // release the spi device if it is acquired
    spi_device_release();

    if(useHS) {
      _usingHS = true;
      esp_err_t esp_ret = spi_bus_remove_device(_spi_device_handle);
      if(esp_ret != ESP_OK) {
        return;
      }
      esp_ret = spi_bus_add_device(SPI2_HOST, &dev_hs_config, &_spi_device_handle);
      if(esp_ret != ESP_OK) {
        return;
      }
    } else {
      _usingHS = false;
      esp_err_t esp_ret = spi_bus_remove_device(_spi_device_handle);
      if(esp_ret != ESP_OK) {
        return;
      }
      esp_ret = spi_bus_add_device(SPI2_HOST, &dev_ls_config, &_spi_device_handle);
      if(esp_ret != ESP_OK) {
        return;
      }
    }
  }
}

void OrbtICM42688::threeWireSPIWrite(bool write) {
  if(_mosiPin == GPIO_NUM_NC) {
    esp_rom_gpio_connect_in_signal(_misoPin, SIG_GPIO_OUT_IDX, 0);
    esp_rom_gpio_connect_out_signal(_misoPin, SIG_GPIO_OUT_IDX, 0, 0);

    if(write) {
      esp_rom_gpio_connect_out_signal(_misoPin, spi_periph_signal[SPI2_HOST].spid_out, false, false);
    } else {
      esp_rom_gpio_connect_in_signal(_misoPin, spi_periph_signal[SPI2_HOST].spid_in, false);
    }   
  }
}

bool OrbtICM42688::spi_device_acquire() {
  if(!_spi_device_acquired) {
    esp_err_t ret = spi_device_acquire_bus(_spi_device_handle, portMAX_DELAY);
    if(ret != ESP_OK) {
      Serial.printf("SPI device acquire failed\n");
      return false;
    }
    _spi_device_acquired = true;
  }
  return true;
}

void OrbtICM42688::spi_device_release() {
  if(_spi_device_acquired) {
    spi_device_release_bus(_spi_device_handle);
    _spi_device_acquired = false;
  }
}

int OrbtICM42688::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
  if(_mosiPin == GPIO_NUM_NC) {
    return readRegisters3Wire(subAddress, count, dest);
  } else {
    return readRegisters4Wire(subAddress, count, dest);
  }
}

/* reads registers from ICM42688 given a starting register address, number of bytes, and a pointer to store data */
int OrbtICM42688::readRegisters3Wire(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));

  subAddress |= 0x80;
  uint8_t cmd[1] = {subAddress};

  if(!spi_device_acquire()) {
    return -1;
  }

  threeWireSPIWrite(true);

  t.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
  t.length = 8;                      //Command is 8 bits
  t.tx_buffer = cmd;                //The data is the cmd itself
  t.user = (void*)0;                 //D/C needs to be set to 0

  ret = spi_device_polling_transmit(_spi_device_handle, &t);

  if(ret != ESP_OK) {
    spi_device_release();
    return -1;
  }

  threeWireSPIWrite(false);

  memset(&t, 0, sizeof(t));
  t.tx_buffer = NULL;
  t.length = count * 8;
  t.rx_buffer = dest;
  
  ret = spi_device_polling_transmit(_spi_device_handle, &t);
  
  if(ret != ESP_OK) {
    spi_device_release();
    return -1;
  }

  spi_device_release();

  return 1;
}

int OrbtICM42688::readRegisters4Wire(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));

  subAddress |= 0x80;
  static uint8_t data[16 * 20];   // FIXME: / TODO: make dynamic and shared. 16 bytes per fifo frame, usually aiming for 10 frames, with extra space for fun.
  memset(data, 0, sizeof(data));

  data[0] = subAddress;

  if(!spi_device_acquire()) {
    return -1;
  }

  t.length = 8 + count * 8;         //Command is 8 bits + data is 8 bits per byte
  t.tx_buffer = data;                //The data is the cmd itself
  t.rx_buffer = data;                //The data is the destination buffer
  t.user = (void*)0;                 //D/C needs to be set to 0

  ret = spi_device_polling_transmit(_spi_device_handle, &t);

  if(ret != ESP_OK) {
    spi_device_release();
    return -1;
  }

  memcpy(dest, data+1, count);

  spi_device_release();

  return 1;
}

typedef struct {
  OrbtICM42688* imu;
  uint8_t write;
  uint8_t release_bus;
  uint8_t* dest;
  uint8_t length;
} transaction_data_t;

void OrbtICM42688::pre_cb(spi_transaction_t *trans) {
  if(trans->user != NULL) {
    transaction_data_t* data = (transaction_data_t*)trans->user;
    if(data->write) {
      data->imu->threeWireSPIWrite(true);
    } else {
      data->imu->threeWireSPIWrite(false);
    }
  }
}

void OrbtICM42688::post_cb(spi_transaction_t *trans) {
  if(trans->user != NULL) {
    transaction_data_t* data = (transaction_data_t*)trans->user;
    if(data->release_bus) {
      data->imu->_interruptResultReady = true;
      data->imu->_interruptTransferInprogress = false;

      // if in 4 wire mode, with single transaction remove the first byte (command) from the data
      if(data->imu->_mosiPin != GPIO_NUM_NC) {
        memcpy(data->dest, trans->rx_data+1, data->length);
      }
    }
  }
}

int OrbtICM42688::readRegistersInterrupt(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  if(_mosiPin == GPIO_NUM_NC) {
    return readRegisters3WireInterrupt(subAddress, count, dest);
  } else {
    return readRegisters4WireInterrupt(subAddress, count, dest);
  }
}

/* reads registers from ICM42688 given a starting register address, number of bytes, and a pointer to store data */
int OrbtICM42688::readRegisters3WireInterrupt(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  esp_err_t ret;
  static transaction_data_t th1, th2;

  subAddress |= 0x80;
  uint8_t cmd[1] = {subAddress};

  _interruptTransferInprogress = true;

  if(!spi_device_acquire()) {
    return -1;
  }

  th1.imu = this;
  th1.write = true;
  th1.release_bus = false;

  memset(&_t1, 0, sizeof(_t1));
  _t1.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
  _t1.length = 8;                      //Command is 8 bits
  _t1.tx_buffer = cmd;                //The data is the cmd itself
  _t1.user = (void*)&th1;

  ret = spi_device_queue_trans(_spi_device_handle, &_t1, portMAX_DELAY);

  if(ret != ESP_OK) {
    spi_device_release();
    return -1;
  }

  th2.imu = this;
  th2.write = false;
  th2.release_bus = true;

  memset(&_t2, 0, sizeof(_t2));
  _t2.tx_buffer = NULL;
  _t2.length = count * 8;
  _t2.rx_buffer = dest;
  _t2.user = (void*)&th2;
   
  ret = spi_device_queue_trans(_spi_device_handle, &_t2, portMAX_DELAY);
  
  if(ret != ESP_OK) {
    spi_device_release();
    return -1;
  }

  spi_device_release();

  return 1;
}

int OrbtICM42688::readRegisters4WireInterrupt(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  esp_err_t ret;
  static transaction_data_t th1;

  subAddress |= 0x80;
  static uint8_t cmd[32];
  static uint8_t data[32];
  memset(cmd, 0, sizeof(cmd));
  memset(data, 0, sizeof(data));

  cmd[0] = subAddress;

  _interruptTransferInprogress = true;

  if(!spi_device_acquire()) {
    return -1;
  }

  th1.imu = this;
  th1.write = false; // this should have no effect when in 4 wire mode
  th1.release_bus = true;
  th1.dest = dest;
  th1.length = count;

  memset(&_t1, 0, sizeof(_t1));
  _t1.length = 8 + count * 8;         //Command is 8 bits + data is 8 bits per byte
  _t1.tx_buffer = cmd;                //The data is the cmd itself
  _t1.rx_buffer = data;
  _t1.user = (void*)&th1;

  ret = spi_device_queue_trans(_spi_device_handle, &_t1, portMAX_DELAY);

  if(ret != ESP_OK) {
    spi_device_release();
    return -1;
  }

  spi_device_release();

  return 1;
}


int OrbtICM42688::setBank(uint8_t bank) {
  // if we are already on this bank, bail
  if (_bank == bank) return 1;

  _bank = bank;

  return writeRegister(REG_BANK_SEL, bank);
}

void OrbtICM42688::reset() {
  setBank(0);

  writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

  // wait for ICM42688 to come back up
  delay(1);
}

/* gets the ICM42688 WHO_AM_I register value */
uint8_t OrbtICM42688::whoAmI() {
  setBank(0);

  // read the WHO AM I register
  if (readRegisters(UB0_REG_WHO_AM_I, 1, _buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}
