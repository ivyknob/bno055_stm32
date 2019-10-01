#include "bno055.h"

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;

void bno055_setPage(uint8_t page) { bno055_writeData(BNO055_PAGE_ID, page); }

bno055_opmode_t bno055_getOperationMode() {
  bno055_opmode_t mode;
  bno055_readData(BNO055_OPR_MODE, &mode, 1);
  return mode;
}

void bno055_setOperationMode(bno055_opmode_t mode) {
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

void bno055_setup() {
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

bno055_calibration_state_t bno055_getCalibrationState() {
  bno055_setPage(0);
  bno055_calibration_state_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
  uint8_t calState = 0;
  bno055_readData(BNO055_CALIB_STAT, &calState, 1);
  cal.sys = (calState >> 6) & 0x03;
  cal.gyro = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag = calState & 0x03;
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
