#include "bno055.h"

#define I2C_HandleTypeDef int;
// I2C_HandleTypeDef *_bno055_i2c_port;

void bno055_delay(int time) {
}

void bno055_writeData(uint8_t reg, uint8_t data) {
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
}

int main() {
  bno055_setup();
}
