#include "bno055.h"

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;

I2C_HandleTypeDef *_bno055_i2c_port;

void bno055_delay(int time) {
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}

void bno055_writeData(uint8_t reg, uint8_t data) {
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  status = HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR_LO << 1,
                                   txdata, sizeof(txdata), 10);
  if (status == HAL_OK) {
    return;
  }

  if (status == HAL_ERROR) {
    printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
  } else if (status == HAL_TIMEOUT) {
    printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
  } else if (status == HAL_BUSY) {
    printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
  } else {
    printf("Unknown status data %d", status);
  }

  uint32_t error = HAL_I2C_GetError(_bno055_i2c_port);
  if (error == HAL_I2C_ERROR_NONE) {
    return;
  } else if (error == HAL_I2C_ERROR_BERR) {
    printf("HAL_I2C_ERROR_BERR\r\n");
  } else if (error == HAL_I2C_ERROR_ARLO) {
    printf("HAL_I2C_ERROR_ARLO\r\n");
  } else if (error == HAL_I2C_ERROR_AF) {
    printf("HAL_I2C_ERROR_AF\r\n");
  } else if (error == HAL_I2C_ERROR_OVR) {
    printf("HAL_I2C_ERROR_OVR\r\n");
  } else if (error == HAL_I2C_ERROR_DMA) {
    printf("HAL_I2C_ERROR_DMA\r\n");
  } else if (error == HAL_I2C_ERROR_TIMEOUT) {
    printf("HAL_I2C_ERROR_TIMEOUT\r\n");
  }

  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(_bno055_i2c_port);
  if (state == HAL_I2C_STATE_RESET) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_READY) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_BUSY) {
    printf("HAL_I2C_STATE_BUSY\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX) {
    printf("HAL_I2C_STATE_BUSY_TX\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX) {
    printf("HAL_I2C_STATE_BUSY_RX\r\n");
  } else if (state == HAL_I2C_STATE_LISTEN) {
    printf("HAL_I2C_STATE_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_ABORT) {
    printf("HAL_I2C_STATE_ABORT\r\n");
  } else if (state == HAL_I2C_STATE_TIMEOUT) {
    printf("HAL_I2C_STATE_TIMEOUT\r\n");
  } else if (state == HAL_I2C_STATE_ERROR) {
    printf("HAL_I2C_STATE_ERROR\r\n");
  }
  // while (HAL_I2C_GetState(_bno055_i2c_port) != HAL_I2C_STATE_READY) {}
  // return;
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
  HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR_LO << 1, &reg, 1,
                          100);
  HAL_I2C_Master_Receive(_bno055_i2c_port, BNO055_I2C_ADDR_LO << 1, data, len,
                         100);
  // HAL_I2C_Mem_Read(_bno055_i2c_port, BNO055_I2C_ADDR_LO<<1, reg,
  // I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void bno055_setPage(uint8_t page) { bno055_writeData(BNO055_PAGE_ID, page); }

void bno055_setOperationMode(uint8_t mode) {
  bno055_writeData(BNO055_OPR_MODE, mode);
  bno055_delay(30);
}

void bno055_setOperationModeConfig() {
  bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF() {
  bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

void bno055_setExternalCrystalUse(bool state) {
  bno055_setPage(0);
  uint8_t tmp = 0;
  bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
  tmp |= (state == true) ? 0x80 : 0x0;
  bno055_writeData(BNO055_SYS_TRIGGER, tmp);
  bno055_delay(700);
}

void bno055_enableExternalCrystal() { bno055_setExternalCrystalUse(true); }
void bno055_disableExternalCrystal() { bno055_setExternalCrystalUse(false); }

void bno055_reset() {
  bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
  bno055_delay(700);
}

int8_t bno055_getTemp() {
  bno055_setPage(0);
  uint8_t t;
  bno055_readData(BNO055_TEMP, &t, 1);
  return t;
}

void bno055_initI2C(I2C_HandleTypeDef *hi2c_device) {
  _bno055_i2c_port = hi2c_device;
  bno055_reset();

  uint8_t id = 0;
  bno055_readData(BNO055_CHIP_ID, &id, 1);
  if (id != BNO055_ID) {
    printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
  }
  bno055_setPage(0);
  bno055_writeData(BNO055_SYS_TRIGGER, 0x0);

  // Select BNO055 config mode
  bno055_setOperationModeConfig();
  bno055_delay(10);
}

int16_t bno055_getSWRevision() {
  bno055_setPage(0);
  uint8_t buffer[2];
  bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

uint8_t bno055_getBootloaderRevision() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

uint8_t bno055_getSystemStatus() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

bno055_self_test_result_t bno055_getSelfTestResult() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_self_test_result_t res = {
      .mcuState = 0, .gyrState = 0, .magState = 0, .accState = 0};
  bno055_readData(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

uint8_t bno055_getSystemError() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

bno055_calibration_t bno055_getCalibration() {
  bno055_setPage(0);
  bno055_calibration_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
  uint8_t calData = 0;
  bno055_readData(BNO055_CALIB_STAT, &calData, 1);
  cal.sys = (calData >> 6) & 0x03;
  cal.gyro = (calData >> 4) & 0x03;
  cal.accel = (calData >> 2) & 0x03;
  cal.mag = calData & 0x03;
  return cal;
}

bno055_vector_t bno055_getVector(uint8_t vec) {
  bno055_setPage(0);
  uint8_t buffer[6];

  bno055_readData(vec, buffer, 6);

  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER) {
    scale = magScale;
  } else if (vec == BNO055_VECTOR_ACCELEROMETER ||
           vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
    scale = accelScale;
  } else if (vec == BNO055_VECTOR_GYROSCOPE) {
    scale = angularRateScale;
  } else if (vec == BNO055_VECTOR_EULER) {
    scale = eulerScale;
  }

  bno055_vector_t xyz = {.x = 0, .y = 0, .z = 0};
  xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
  xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
  xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;

  return xyz;
}

bno055_vector_t bno055_getVectorAccelerometer() {
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
bno055_vector_t bno055_getVectorMagnetometer() {
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
bno055_vector_t bno055_getVectorGyroscope() {
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
bno055_vector_t bno055_getVectorEuler() {
  return bno055_getVector(BNO055_VECTOR_EULER);
}
bno055_vector_t bno055_getVectorLinearAccel() {
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
bno055_vector_t bno055_getVectorGravity() {
  return bno055_getVector(BNO055_VECTOR_GRAVITY);
}
