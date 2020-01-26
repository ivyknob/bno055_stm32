#ifndef BNO055_H_
#define BNO055_H_

#ifdef __cplusplus
  extern "C" {
#endif
// #define FREERTOS_ENABLED true

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define START_BYTE 0xAA
#define RESPONSE_BYTE 0xBB
#define ERROR_BYTE 0xEE

#define BNO055_I2C_ADDR_HI 0x29
#define BNO055_I2C_ADDR_LO 0x28
#define BNO055_I2C_ADDR    BNO055_I2C_ADDR_LO

#define BNO055_READ_TIMEOUT 100
#define BNO055_WRITE_TIMEOUT 10

#define ERROR_WRITE_SUCCESS 0x01  // Everything working as expected
#define ERROR_WRITE_FAIL                                                       \
  0x03  // Check connection, protocol settings and operation more of BNO055
#define ERROR_REGMAP_INV_ADDR 0x04   // Invalid register address
#define ERROR_REGMAP_WRITE_DIS 0x05  // Register is read-only
#define ERROR_WRONG_START_BYTE 0x06  // Check if the first byte
#define ERROR_BUS_OVERRUN_ERR                                                  \
  0x07  // Resend the command, BNO055 was not able to clear the receive buffer
#define ERROR_MAX_LEN_ERR                                                      \
  0x08  // Split the command, max fire size can be up to 128 bytes
#define ERROR_MIN_LEN_ERR 0x09  // Min length of data is less than 1
#define ERROR_RECV_CHAR_TIMEOUT                                                \
  0x0A  // Decrease the waiting time between sending of two bytes of one frame

#define REG_WRITE 0x00
#define REG_READ 0x01

// Page 0
#define BNO055_ID (0xA0)
#define BNO055_CHIP_ID 0x00        // value: 0xA0
#define BNO055_ACC_ID 0x01         // value: 0xFB
#define BNO055_MAG_ID 0x02         // value: 0x32
#define BNO055_GYRO_ID 0x03        // value: 0x0F
#define BNO055_SW_REV_ID_LSB 0x04  // value: 0x08
#define BNO055_SW_REV_ID_MSB 0x05  // value: 0x03
#define BNO055_BL_REV_ID 0x06      // N/A
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_DATA_X_LSB 0x08
#define BNO055_ACC_DATA_X_MSB 0x09
#define BNO055_ACC_DATA_Y_LSB 0x0A
#define BNO055_ACC_DATA_Y_MSB 0x0B
#define BNO055_ACC_DATA_Z_LSB 0x0C
#define BNO055_ACC_DATA_Z_MSB 0x0D
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_MAG_DATA_X_MSB 0x0F
#define BNO055_MAG_DATA_Y_LSB 0x10
#define BNO055_MAG_DATA_Y_MSB 0x11
#define BNO055_MAG_DATA_Z_LSB 0x12
#define BNO055_MAG_DATA_Z_MSB 0x13
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_GYR_DATA_X_MSB 0x15
#define BNO055_GYR_DATA_Y_LSB 0x16
#define BNO055_GYR_DATA_Y_MSB 0x17
#define BNO055_GYR_DATA_Z_LSB 0x18
#define BNO055_GYR_DATA_Z_MSB 0x19
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_EUL_HEADING_MSB 0x1B
#define BNO055_EUL_ROLL_LSB 0x1C
#define BNO055_EUL_ROLL_MSB 0x1D
#define BNO055_EUL_PITCH_LSB 0x1E
#define BNO055_EUL_PITCH_MSB 0x1F
#define BNO055_QUA_DATA_W_LSB 0x20
#define BNO055_QUA_DATA_W_MSB 0x21
#define BNO055_QUA_DATA_X_LSB 0x22
#define BNO055_QUA_DATA_X_MSB 0x23
#define BNO055_QUA_DATA_Y_LSB 0x24
#define BNO055_QUA_DATA_Y_MSB 0x25
#define BNO055_QUA_DATA_Z_LSB 0x26
#define BNO055_QUA_DATA_Z_MSB 0x27
#define BNO055_LIA_DATA_X_LSB 0x28
#define BNO055_LIA_DATA_X_MSB 0x29
#define BNO055_LIA_DATA_Y_LSB 0x2A
#define BNO055_LIA_DATA_Y_MSB 0x2B
#define BNO055_LIA_DATA_Z_LSB 0x2C
#define BNO055_LIA_DATA_Z_MSB 0x2D
#define BNO055_GRV_DATA_X_LSB 0x2E
#define BNO055_GRV_DATA_X_MSB 0x2F
#define BNO055_GRV_DATA_Y_LSB 0x30
#define BNO055_GRV_DATA_Y_MSB 0x31
#define BNO055_GRV_DATA_Z_LSB 0x32
#define BNO055_GRV_DATA_Z_MSB 0x33
#define BNO055_TEMP 0x34
#define BNO055_CALIB_STAT 0x35
#define BNO055_ST_RESULT 0x36
#define BNO055_INT_STATUS 0x37
#define BNO055_SYS_CLK_STATUS 0x38
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_UNIT_SEL 0x3B
#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
#define BNO055_TEMP_SOURCE 0x40
#define BNO055_AXIS_MAP_CONFIG 0x41
#define BNO055_AXIS_MAP_SIGN 0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB 0x67
#define BNO055_ACC_RADIUS_MSB 0x68
#define BNO055_MAG_RADIUS_LSB 0x69
#define BNO055_MAG_RADIUS_MSB 0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_CONFIG 0x08
#define BNO055_MAG_CONFIG 0x09
#define BNO055_GYRO_CONFIG_0 0x0A
#define BNO055_GYRO_CONFIG_1 0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK 0x0F
#define BNO055_INT_EN 0x10
#define BNO055_ACC_AM_THRES 0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION 0x13
#define BNO055_ACC_HG_THRESH 0x14
#define BNO055_ACC_NM_THRESH 0x15
#define BNO055_ACC_NM_SET 0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET 0x18
#define BNO055_GYR_DUR_X 0x19
#define BNO055_GYR_HR_Y_SET 0x1A
#define BNO055_GYR_DUR_Y 0x1B
#define BNO055_GYR_HR_Z_SET 0x1C
#define BNO055_GYR_DUR_Z 0x1D
#define BNO055_GYR_AM_THRESH 0x1E
#define BNO055_GYR_AM_SET 0x1F

enum bno055_system_status_t {
  BNO055_SYSTEM_STATUS_IDLE = 0x00,
  BNO055_SYSTEM_STATUS_SYSTEM_ERROR = 0x01,
  BNO055_SYSTEM_STATUS_INITIALIZING_PERIPHERALS = 0x02,
  BNO055_SYSTEM_STATUS_SYSTEM_INITIALIZATION = 0x03,
  BNO055_SYSTEM_STATUS_EXECUTING_SELF_TEST = 0x04,
  BNO055_SYSTEM_STATUS_FUSION_ALGO_RUNNING = 0x05,
  BNO055_SYSTEM_STATUS_FUSION_ALOG_NOT_RUNNING = 0x06
};

typedef enum {  // BNO-55 operation modes
  BNO055_OPERATION_MODE_CONFIG = 0x00,
  // Sensor Mode
  BNO055_OPERATION_MODE_ACCONLY,
  BNO055_OPERATION_MODE_MAGONLY,
  BNO055_OPERATION_MODE_GYRONLY,
  BNO055_OPERATION_MODE_ACCMAG,
  BNO055_OPERATION_MODE_ACCGYRO,
  BNO055_OPERATION_MODE_MAGGYRO,
  BNO055_OPERATION_MODE_AMG,  // 0x07
                              // Fusion Mode
  BNO055_OPERATION_MODE_IMU,
  BNO055_OPERATION_MODE_COMPASS,
  BNO055_OPERATION_MODE_M4G,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  BNO055_OPERATION_MODE_NDOF  // 0x0C
} bno055_opmode_t;

typedef struct {
  uint8_t mcuState;
  uint8_t gyrState;
  uint8_t magState;
  uint8_t accState;
} bno055_self_test_result_t;

typedef struct {
  uint8_t sys;
  uint8_t gyro;
  uint8_t mag;
  uint8_t accel;
} bno055_calibration_state_t;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} bno055_vector_xyz_int16_t;

typedef struct {
  bno055_vector_xyz_int16_t gyro;
  bno055_vector_xyz_int16_t mag;
  bno055_vector_xyz_int16_t accel;
} bno055_calibration_offset_t;

typedef struct {
  uint16_t mag;
  uint16_t accel;
} bno055_calibration_radius_t;

typedef struct {
  bno055_calibration_offset_t offset;
  bno055_calibration_radius_t radius;
} bno055_calibration_data_t;

typedef struct {
  double w;
  double x;
  double y;
  double z;
} bno055_vector_t;

typedef struct {
  uint8_t x;
  uint8_t x_sign;
  uint8_t y;
  uint8_t y_sign;
  uint8_t z;
  uint8_t z_sign;
} bno055_axis_map_t;

typedef enum {
  BNO055_VECTOR_ACCELEROMETER = 0x08,  // Default: m/s²
  BNO055_VECTOR_MAGNETOMETER = 0x0E,   // Default: uT
  BNO055_VECTOR_GYROSCOPE = 0x14,      // Default: rad/s
  BNO055_VECTOR_EULER = 0x1A,          // Default: degrees
  BNO055_VECTOR_QUATERNION = 0x20,     // No units
  BNO055_VECTOR_LINEARACCEL = 0x28,    // Default: m/s²
  BNO055_VECTOR_GRAVITY = 0x2E         // Default: m/s²
} bno055_vector_type_t;

enum bno055_system_error_t {
  BNO055_SYSTEM_ERROR_NO_ERROR = 0x00,
  BNO055_SYSTEM_ERROR_PERIPHERAL_INITIALIZATION_ERROR = 0x01,
  BNO055_SYSTEM_ERROR_SYSTEM_INITIALIZATION_ERROR = 0x02,
  BNO055_SYSTEM_ERROR_SELF_TEST_FAILED = 0x03,
  BNO055_SYSTEM_ERROR_REG_MAP_VAL_OUT_OF_RANGE = 0x04,
  BNO055_SYSTEM_ERROR_REG_MAP_ADDR_OUT_OF_RANGE = 0x05,
  BNO055_SYSTEM_ERROR_REG_MAP_WRITE_ERROR = 0x06,
  BNO055_SYSTEM_ERROR_LOW_PWR_MODE_NOT_AVAILABLE_FOR_SELECTED_OPR_MODE = 0x07,
  BNO055_SYSTEM_ERROR_ACCEL_PWR_MODE_NOT_AVAILABLE = 0x08,
  BNO055_SYSTEM_ERROR_FUSION_ALGO_CONF_ERROR = 0x09,
  BNO055_SYSTEM_ERROR_SENSOR_CONF_ERROR = 0x0A
};

enum bno055_axis_map_representation_t {
  BNO055_AXIS_X = 0x00,
  BNO055_AXIS_Y = 0x01,
  BNO055_AXIS_Z = 0x02
};

enum bno055_axis_map_sign_t {
  BNO055_AXIS_SIGN_POSITIVE = 0x00,
  BNO055_AXIS_SIGN_NEGATIVE = 0x01
};

void bno055_writeData(uint8_t reg, uint8_t data);
void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len);
void bno055_delay(int time);

void bno055_reset();
bno055_opmode_t bno055_getOperationMode();
void bno055_setOperationMode(bno055_opmode_t mode);
void bno055_setOperationModeConfig();
void bno055_setOperationModeNDOF();
void bno055_enableExternalCrystal();
void bno055_disableExternalCrystal();
void bno055_setup();

int8_t bno055_getTemp();

uint8_t bno055_getBootloaderRevision();
uint8_t bno055_getSystemStatus();
uint8_t bno055_getSystemError();
int16_t bno055_getSWRevision();

bno055_self_test_result_t bno055_getSelfTestResult();
bno055_calibration_state_t bno055_getCalibrationState();
bno055_calibration_data_t bno055_getCalibrationData();
void bno055_setCalibrationData(bno055_calibration_data_t calData);
bno055_vector_t bno055_getVectorAccelerometer();
bno055_vector_t bno055_getVectorMagnetometer();
bno055_vector_t bno055_getVectorGyroscope();
bno055_vector_t bno055_getVectorEuler();
bno055_vector_t bno055_getVectorLinearAccel();
bno055_vector_t bno055_getVectorGravity();
bno055_vector_t bno055_getVectorQuaternion();
void bno055_setAxisMap(bno055_axis_map_t axis);

#ifdef __cplusplus
  }
#endif
#endif  // BNO055_H_
