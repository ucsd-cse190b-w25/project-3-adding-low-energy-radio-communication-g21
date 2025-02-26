/*
 * lsm6dsl.c
 */
#include "lsm6dsl.h"
#include "i2c.h" 
#include <stm32l475xx.h>
#include <stdint.h>
#define LSM6DSL_ADDR 0x6A //SDO/SA0 pin is connected to ground, the LSb value is ‘0’ (address 1101010b) (0x6A or 0x6B)
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define INT1_CTRL 0x0D
#define STATUS_REG 0x1E
#define OUTX_L_XL 0x28 // start of accelerometer outpuot


void lsm6dsl_init() {
    uint8_t configure_accel_data[2] = {CTRL1_XL, 0x60};
    i2c_transaction(LSM6DSL_ADDR, 0, configure_accel_data, 2);

    uint8_t configure_gyro_data[2] = {CTRL2_G, 0x60};
    i2c_transaction(LSM6DSL_ADDR, 0, configure_gyro_data, 2);

    //enable data ready interrupt for accelerometer
    uint8_t int1_data[3] = {INT1_CTRL, 0x01, 0x02};
    i2c_transaction(LSM6DSL_ADDR, 0, int1_data, 3);
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t status;
    uint8_t status_reg_addr = STATUS_REG;

    i2c_transaction(LSM6DSL_ADDR, 0, &status_reg_addr, 1);
    i2c_transaction(LSM6DSL_ADDR, 1, &status, 1);
    while((status & 0x01) == 0);

    uint8_t xyz[6];
    uint8_t outx_low_addr = OUTX_L_XL;

    i2c_transaction(LSM6DSL_ADDR, 0, &outx_low_addr, 1);
    i2c_transaction(LSM6DSL_ADDR, 1, xyz, 6);

    *x = (int16_t) (xyz[1] << 8) | (xyz[0]);
    *y = (int16_t) (xyz[3] << 8) | (xyz[2]);
    *z = (int16_t) (xyz[5] << 8) | (xyz[4]);
}
