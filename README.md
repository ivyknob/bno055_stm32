# BNO055 STM32 library [![Build Status](https://travis-ci.org/ivyknob/bno055_stm32.svg?branch=master)](https://travis-ci.org/ivyknob/bno055_stm32) [![Maintainability](https://api.codeclimate.com/v1/badges/121a150b18db278f559b/maintainability)](https://codeclimate.com/github/ivyknob/bno055_stm32/maintainability)


## WORK IN PROGRESS

There are no libraries to use BNO055 with STM32.
We created one to use with STM32 HAL I2C.

It does support FreeRTOS, see `bno055.h`. Uncomment `#define FREERTOS_ENABLED` to enable FreeRTOS.

## Usage

Use CubeMX to init i2c in fast mode.

Copy `bno055.c`, `bno055.h` and `bno055_stm32.h` to your project.
Include `bno055_stm32.h`.
Pass i2c handler to bno055_assignI2C function and set work mode:

```c
bno055_assignI2C(&hi2c1);
bno055_setup();
bno055_setOperationModeNDOF();
```

Then use bno055_getVectorEuler to receive data:

```c
bno055_vector_t v = bno055_getVectorEuler();
printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.x, v.y, v.z);
```

### Full example

```c
# main.c


```
