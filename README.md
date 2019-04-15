# BNO055 STM32 library

## WORK IN PROGRESS

There are no libraries to use BNO055 with STM32.
We created one to use with STM32 HAL I2C.

It does support FreeRTOS, see `bno055.h` for `#define FREERTOS_ENABLED`.

## Usage

Use CubeMX to init i2c in fast mode. Pass i2c handler to bno055_initI2C function and set work mode:

```c
bno055_initI2C(&hi2c1);
bno055_setOperationModeNDOF();
```

Then use bno055_getVectorEuler to receive data:

```c
bno055_vector_t v = bno055_getVectorEuler();
printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.x, v.y, v.z);
```
