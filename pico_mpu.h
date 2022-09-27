/**
 *  Raspberry Pi Pico MPU API
 *
 *  Copyright (c) David C. Rankin, 2022
 *  License: GPLv2
 */

#ifndef PICO_MPU_H
#define PICO_MPU_H  1


/** i2c helper functions for read and write with default values */
int i2c_read (uint8_t addr, uint8_t *dst, size_t len);
int i2c_write (uint8_t addr, uint8_t *src, size_t len);
int i2c_read_reg (uint8_t addr, const uint8_t reg_addr, uint8_t *dst,
                  size_t len);
int i2c_write_reg (uint8_t addr, const uint8_t reg_addr, uint8_t *src,
                   size_t len);

/** mpu reset function */
int mpu_reset (void);

/** mpu initialization functions */
bool mpu_i2c_check (void);        /* check mpu exists on I2C bus */
bool mpu_init_default (void);     /* default FS_SEL - 250 deg/sec and 2 g. */
bool mpu_init (int gyro_fs_sel, int accel_fs_sel, uint8_t dlpfval);

/** mpu acceleration and rotational rate values */
void get_raw_acc_gyro (int16_t *acc, int16_t *gyro);
void get_acc_gyro (float *acc, float *gyro);
/* with temperature in deg C */
void get_raw_acc_gyro_tempc (int16_t *acc, int16_t *gyro, int16_t *tempc);
void get_acc_gyro_tempc (float *acc, float *gyro, float *tempc);


#ifdef MPU9250
/** magnetometer functions for MPU9250 */
bool ak8963_i2c_check (void);     /* check magnetometer exists on I2C bus */
bool ak8963_initialize (void);

void get_raw_orientation (int16_t *data);
void get_orientation (float *mag);
#endif

#endif
