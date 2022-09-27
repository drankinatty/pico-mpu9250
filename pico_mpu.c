/**
 *  Raspberry Pi Pico MPU API
 *
 *  Copyright (c) David C. Rankin, 2022
 *  License: GPLv2
 */

/* accomodates MPU6050 - MPU9250 */
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "pico_mpu.h"

/* direct acceleration and gyroscope axis bias
 * these are specific to my 2 MPU chips, to serve
 * as an example.
 */
#if MPUCHIP == 1
  static const float AXBIAS   = -0.02f;
  static const float AYBIAS   =  0.00f;
  static const float AZBIAS   =  0.01f;

  static const int8_t GXBIAS  =   0x20;
  static const int8_t GYBIAS  =   0x11;
  static const int8_t GZBIAS  =    -12;
#elif MPUCHIP == 2
  static const float AXBIAS   = -0.05f;
  static const float AYBIAS   = -0.03f;
  static const float AZBIAS   =  0.02f;

  static const int8_t GXBIAS  =   -102;
  static const int8_t GYBIAS  =    -16;
  static const int8_t GZBIAS  =     16;
#else
  static const float AXBIAS   =  0.00f;
  static const float AYBIAS   =  0.00f;
  static const float AZBIAS   =  0.00f;

  static const int8_t GXBIAS  =   0x00;
  static const int8_t GYBIAS  =   0x00;
  static const int8_t GZBIAS  =   0x00;
#endif

#ifdef DENOISE
  /* denoise limits stop bounce about zero on each axis */
  static const float DENOISE_LIM_ACCEL  =   0.1f;
  static const float DENOISE_LIM_GYRO   =   0.4f;
#else
  static const float DENOISE_LIM_ACCEL  =   0.0f;
  static const float DENOISE_LIM_GYRO   =   0.0f;
#endif

/* default i2c accress */
static const uint8_t MPU_DEFAULT_ADDRESS  =   0x68;

/* MPU Registers */
static const uint8_t MPU_SELF_TEST_X_GYRO       =   0x00;
static const uint8_t MPU_SELF_TEST_Y_GYRO       =   0x01;
static const uint8_t MPU_SELF_TEST_Z_GYRO       =   0x02;
static const uint8_t MPU_SELF_TEST_X_ACCEL      =   0x0D;
static const uint8_t MPU_SELF_TEST_Y_ACCEL      =   0x0E;
static const uint8_t MPU_SELF_TEST_Z_ACCEL      =   0x0F;

static const uint8_t MPU_XG_OFFSET_H            =   0x13;
static const uint8_t MPU_XG_OFFSET_L            =   0x14;
static const uint8_t MPU_YG_OFFSET_H            =   0x15;
static const uint8_t MPU_YG_OFFSET_L            =   0x16;
static const uint8_t MPU_ZG_OFFSET_H            =   0x17;
static const uint8_t MPU_ZG_OFFSET_L            =   0x18;

static const uint8_t MPU_SMPLRT_DIV             =   0x19;

static const uint8_t MPU_CONFIG                 =   0x1A;
static const uint8_t MPU_GYRO_CONFIG            =   0x1B;
static const uint8_t MPU_ACCEL_CONFIG           =   0x1C;
static const uint8_t MPU_ACCEL_CONFIG_2         =   0x1D;

static const uint8_t MPU_LP_ACCEL_ODR           =   0x1E;
static const uint8_t MPU_WOM_THR                =   0x1F;
static const uint8_t MPU_FIFO_EN                =   0x23;

static const uint8_t MPU_INT_PIN_CFG            =   0x37;
static const uint8_t MPU_INT_ENABLE             =   0x38;
static const uint8_t MPU_INT_STATUS             =   0x3A;

static const uint8_t MPU_ACCEL_XOUT_H           =   0x3B;
static const uint8_t MPU_ACCEL_XOUT_L           =   0x3C;
static const uint8_t MPU_ACCEL_YOUT_H           =   0x3D;
static const uint8_t MPU_ACCEL_YOUT_L           =   0x3E;
static const uint8_t MPU_ACCEL_ZOUT_H           =   0x3F;
static const uint8_t MPU_ACCEL_ZOUT_L           =   0x40;

static const uint8_t MPU_TEMP_OUT_H             =   0x41;
static const uint8_t MPU_TEMP_OUT_L             =   0x42;

static const uint8_t MPU_GYRO_XOUT_H            =   0x43;
static const uint8_t MPU_GYRO_XOUT_L            =   0x44;
static const uint8_t MPU_GYRO_YOUT_H            =   0x45;
static const uint8_t MPU_GYRO_YOUT_L            =   0x46;
static const uint8_t MPU_GYRO_ZOUT_H            =   0x47;
static const uint8_t MPU_GYRO_ZOUT_L            =   0x48;

static const uint8_t MPU_SIGNAL_PATH_RESET      =   0x68;
static const uint8_t GYRO_RST           =   0x04;
static const uint8_t ACCEL_RST          =   0x02;
static const uint8_t TEMP_RST           =   0x01;

static const uint8_t MPU_MOT_DETECT_CTRL        =   0x69;

static const uint8_t MPU_USER_CTRL              =   0x6A;
static const uint8_t FIFO_EN                    =   0x40;
static const uint8_t I2C_MST_EN                 =   0x20;
static const uint8_t I2C_IF_DIS                 =   0x10;
static const uint8_t FIFO_RST                   =   0x04;
static const uint8_t I2C_MST_RST                =   0x02;
static const uint8_t SIG_COND_RST               =   0x01;

static const uint8_t MPU_PWR_MGMT_1             =   0x6B;
static const uint8_t MPU_PWR_MGMT_2             =   0x6C;

static const uint8_t MPU_FIFO_COUNTH            =   0x72;
static const uint8_t MPU_FIFO_COUNTL            =   0x73;
static const uint8_t MPU_FIFO_R_W               =   0x74;

static const uint8_t MPU_WHO_AM_I               =   0x75;
static const uint8_t MPU_WHO_AM_I_DATA  =   0x71;

static const uint8_t MPU_XA_OFFSET_H            =   0x77;
static const uint8_t MPU_XA_OFFSET_L            =   0x78;
static const uint8_t MPU_YA_OFFSET_H            =   0x7A;
static const uint8_t MPU_YA_OFFSET_L            =   0x7B;
static const uint8_t MPU_ZA_OFFSET_H            =   0x7D;
static const uint8_t MPU_ZA_OFFSET_L            =   0x7E;

/* additional configuration values */
static const uint8_t MPU_CLOCK_PLL_XGYRO        =   0x01;

/* AK8963 Gyro Register Addresses */
static const uint8_t AK8963_DEFAULT_ADDRESS     =   0x0C;

static const uint8_t AK8963_WAI                 =   0x00;    /* read-only */
static const uint8_t AK8963_WAI_DATA    =   0x48;

static const uint8_t AK8963_INFO                =   0x01;
static const uint8_t AK8963_STI                 =   0x02;
static const uint8_t AK8963_HXL                 =   0x03;
static const uint8_t AK8963_HXH                 =   0x04;
static const uint8_t AK8963_HYL                 =   0x05;
static const uint8_t AK8963_HYH                 =   0x06;
static const uint8_t AK8963_HZL                 =   0x07;
static const uint8_t AK8963_HZH                 =   0x08;
static const uint8_t AK8963_ST2                 =   0x09;

static const uint8_t AK8963_CNTL1               =   0x0A;

static const uint8_t AK8963_CNTL1_PWR_DOWN      =   0x00;   /* power down mode */
static const uint8_t AK8963_CNTL1_SINGLE_MSRMT  =   0x01;   /* single measurement mode */
static const uint8_t AK8963_CNTL1_CONT_MSRMT1   =   0x02;   /* continuous measurement 1 mode */
static const uint8_t AK8963_CNTL1_CONT_MSRMT2   =   0x06;   /* continuous measurement 2 mode */
static const uint8_t AK8963_CNTL1_EXT_MSRMT     =   0x04;   /* external trigger measurement */
static const uint8_t AK8963_CNTL1_SELF_TEST     =   0x08;   /* self-test mode */
static const uint8_t AK8963_CNTL1_FUSE_ROM      =   0x0F;   /* Fuse ROM mode */
static const uint8_t AK8963_CNTL1_OUT_BITS      =   0x10;   /* output bit, 0 14-bit, 1 16-bit */

static const uint8_t AK8963_ASAX                =   0x10;   /* read-only */
static const uint8_t DRDY_LIMIT                 =   0x80;   /* data-read limit */

static const uint16_t MAG_SENSE                 =   4912;


/* default I2C device address for Inversense MPU and AK8963 magnetometer */
static const uint8_t mpu_addr = MPU_DEFAULT_ADDRESS;
static const uint8_t mag_addr = AK8963_DEFAULT_ADDRESS;

#define RWBUFSZ  16
uint8_t buffer[RWBUFSZ];    /* read-write buffer for general use */
uint8_t rw_buf[RWBUFSZ];    /* read-write buffer for i2c read/write use */
uint8_t dlpf;               /* provides extern reference to setting */
uint8_t trim[6] = {0};      /* axis trim values */

/**
 *  asax, asay, asaz are sensitivity adjustment data for each axis stored to
 *  fuse ROM on shipment. The adjusted value for each axis is given by:
 *
 *    Hadj = H * ( ((ASA - 128) * 0.5) / 128 + 1 )
 *
 *    where H is the asix measurement, ASA is the axis sensitivity adjustment
 *
 *  the values asax, asay, asaz hold ( ((ASA - 128) * 0.5) / 128 + 1 ) for
 *  each axis.
 */
float asax, asay, asaz;

/**
 *  gyro and accel sensitivity set during init from FS_SEL value provided, or
 *  set to default values of 250 deg/sec and 2 g if the default initialize
 *  function used, or in the event the user passes invalid FS_SEL values to
 *  the initialize function.
 */
float gyro_sens, accel_sens;

/**
 *  room temperature offset in degrees C.
 *  temperator reported is chip temperature, not room temperature, to get
 *  get close approximation, set room_temp_offset. (2.8 C * 333.87)
 */
float room_temp_offset = 2.5 * 333.87;

/** i2c helper functions for read and write with default values */

int i2c_read (uint8_t addr, uint8_t *dst, size_t len)
{
  /* i2c_default, addr, and nostop = false (master releases bus control) */
  return i2c_read_blocking (i2c_default, addr, dst, len, false);
}

int i2c_write (uint8_t addr,  uint8_t *src, size_t len)
{
  /* i2c_default, addr, and nostop = true (master retains bus control) */
  return i2c_write_blocking (i2c_default, addr, src, len, true);
}

/* to read after writing register address */
int i2c_read_reg (uint8_t addr, const uint8_t reg_addr, uint8_t *dst,
                  size_t len)
{
  /* no STOP bit should be required with write of addr before read */
  int rtn = i2c_write (addr, (uint8_t*)&reg_addr, 1);

  if (rtn == PICO_ERROR_GENERIC) {
    return rtn;
  }

  /* i2c_default, addr, and nostop = false (master releases bus control) */
  return i2c_read_blocking (i2c_default, addr, dst, len, false);
}

/* to write data to a specific register address before an independent read */
int i2c_write_reg (uint8_t addr, const uint8_t reg_addr, uint8_t *src,
                   size_t len)
{
  rw_buf[0] = reg_addr;

  for (size_t i = 0; i < len; i++) {
    rw_buf[i + 1] = src[i];
  }

  /* no STOP bit should be required with write of addr before read */
  return i2c_write_blocking (i2c_default, addr, rw_buf, len + 1, false);
}


/** reset accel, gyro and temperature sensor signal paths */
int mpu_reset (void)
{
  uint8_t data = 0x00;

  return i2c_write_reg (mpu_addr, MPU_SIGNAL_PATH_RESET, &data, 1);
}


/** add accelerometer bias */
static void add_accel_bias (float *acc)
{
  acc[0] += AXBIAS;
  acc[1] += AYBIAS;
  acc[2] += AZBIAS;
}


/** set gyro rate bias */
static void set_mpu_usr_offs (void)
{
#ifdef SETBIAS
  buffer[0] = (GXBIAS >> 8) & 0xff;
  buffer[1] =  GXBIAS & 0xff;
  i2c_write_reg (mpu_addr, MPU_XG_OFFSET_H, buffer, 2);

  buffer[0] = (GYBIAS >> 8) & 0xff;
  buffer[1] =  GYBIAS & 0xff;
  i2c_write_reg (mpu_addr, MPU_YG_OFFSET_H, buffer, 2);

  buffer[0] = (GZBIAS >> 8) & 0xff;
  buffer[1] =  GZBIAS & 0xff;
  i2c_write_reg (mpu_addr, MPU_ZG_OFFSET_H, buffer, 2);

#else
  buffer[0] = buffer[1] = 0;
  i2c_write_reg (mpu_addr, MPU_XG_OFFSET_H, buffer, 2);
  i2c_write_reg (mpu_addr, MPU_YG_OFFSET_H, buffer, 2);
  i2c_write_reg (mpu_addr, MPU_ZG_OFFSET_H, buffer, 2);

#endif

  i2c_read_reg (mpu_addr, MPU_XA_OFFSET_H, trim, 6);
}


/**
 *  Check whether MPU9250 is available on the I2C BUS
 */
bool mpu_i2c_check (void)
{
  uint8_t data;

  i2c_read_reg (mpu_addr, MPU_WHO_AM_I, &data, 1);

  if (data != MPU_WHO_AM_I_DATA) {
    return false;
  }

  return true;
}


/**
 *  Initialize mpu gyro with FS_SEL = 250 and accell FS_SEL = 2
 */
bool mpu_init_default (void)
{
  uint8_t data = 0x00;

  gyro_sens = 250.;
  accel_sens = 2.;

  /* confirm mpu found on I2C bus or return error */
  if (!mpu_i2c_check()) {
    return false;
  }

  /* reset sample rate divisor zero  */
  i2c_write_reg (mpu_addr, MPU_SMPLRT_DIV, &data, sizeof data);

  /* reset all sensors - write 0 to MPU_PWR_MGMT_2 */
  i2c_write_reg (mpu_addr, MPU_PWR_MGMT_2, &data, sizeof data);

  /* PWR1 set - auto select best available clock */
  data = MPU_CLOCK_PLL_XGYRO;          /* (0x01) */
  i2c_write_reg (mpu_addr, MPU_PWR_MGMT_1, &data, sizeof data);

  /* set DLPF to bandwidth 10Hz for gyro and temperature sensor */
  // data = 0x05;
  dlpf = 0x02;

  i2c_write_reg (mpu_addr, MPU_CONFIG, &dlpf, sizeof dlpf);

  /* set DLPF to bandwidth 10Hz for accel */
  i2c_write_reg (mpu_addr, MPU_ACCEL_CONFIG_2, &dlpf, sizeof dlpf);

  /* set gyro FS_SEL */
  data = 0x00;

  i2c_write_reg (mpu_addr, MPU_GYRO_CONFIG, &data, sizeof data);

  /* set accel FS_SEL */
  i2c_write_reg (mpu_addr, MPU_ACCEL_CONFIG, &data, sizeof data);

  /* set accel and gyro bias */
  set_mpu_usr_offs();

  return true;
}


/**
 *  Initialize mpu gyro and accel with values given by gyro_fs_sel and
 *  accel_fs_sel, if parameter values do not match valid FS-SEL values the
 *  corresponding gyro or accel will be initialized to the default FS_SEL
 *  values of 250 deg/sec and 2 g.
 */
bool mpu_init (int gyro_fs_sel, int accel_fs_sel, uint8_t dlpfval)
{
  uint8_t data = 0x00,
          fs_sel[] = { 0x0, 0x8, 0x10, 0x18 };
  int gyro_sens_sel[] = { 250, 500, 1000, 2000 },
      accel_sens_sel[] = { 2, 4, 8, 16 },
      i = 0;

  gyro_sens = 250.;       /* set default gyro and access sens values */
  accel_sens = 2.;

  /* confirm mpu found on I2C bus or return error */
  if (!mpu_i2c_check()) {
    return false;
  }

  /* reset sample rate divisor zero  */
  i2c_write_reg (mpu_addr, MPU_SMPLRT_DIV, &data, sizeof data);

  /* reset all sensors - write 0 to MPU_PWR_MGMT_2 */
  i2c_write_reg (mpu_addr, MPU_PWR_MGMT_2, &data, sizeof data);

  /* PWR1 set - auto select best available clock */
  data = MPU_CLOCK_PLL_XGYRO;          /* (0x01) */
  i2c_write_reg (mpu_addr, MPU_PWR_MGMT_1, &data, sizeof data);

  /* set DLPF to bandwidth 10Hz for gyro and temperature sensor */
  // data = 0x05;
  dlpf = dlpfval;
  data = dlpfval;
  i2c_write_reg (mpu_addr, MPU_CONFIG, &dlpf, sizeof dlpf);

  /* set DLPF to bandwidth 10Hz for accel */
  i2c_write_reg (mpu_addr, MPU_ACCEL_CONFIG_2, &dlpf, sizeof dlpf);

  /* set gyro FS_SEL and sensitivity */
  data = 0x00;
  for (i = 0; i < sizeof fs_sel; i++)       /* iterate over parameter values */
  {
    if (gyro_fs_sel == gyro_sens_sel[i])    /* if valid parameter */
    {
      gyro_sens = gyro_fs_sel;              /* set gyro sensitivity */
      data = fs_sel[i];                     /* set gyro FS_SEL */
      break;
    }
  }
  i2c_write_reg (mpu_addr, MPU_GYRO_CONFIG, &data, sizeof data);

  /* set accel FS_SEL and sensitivity */
  data = 0x00;
  for (i = 0; i < sizeof fs_sel; i++)       /* iterate over parameter values */
  {
    if (accel_fs_sel == accel_sens_sel[i])  /* if valid parameter */
    {
      accel_sens = accel_fs_sel;            /* set accel sensitivity */
      data = fs_sel[i];                     /* set accel FS_SEL */
      break;
    }
  }
  i2c_write_reg (mpu_addr, MPU_ACCEL_CONFIG, &data, sizeof data);

  /* set accel and gyro bias */
  set_mpu_usr_offs();

  return true;
}

/* apply limit around zero to reduce noise in values */
static void denoise_values (float limit, float *data)
{
  /* initialize x, y, z to positive values */
  float   x = data[0] < 0. ? -data[0] : data[0],
          y = data[1] < 0. ? -data[1] : data[1],
          z = data[2] < 0. ? -data[2] : data[2];

  if (limit < 0.) {           /* ensure limit is positive */
    limit = -limit;
  }

  if (x < limit) {            /* denoise x values */
    data[0] = 0.;
  }

  if (y < limit) {            /* denoise y values */
    data[1] = 0.;
  }

  if (z < limit) {            /* denoise z values */
    data[2] = 0.;
  }
}


/**
 *  Get raw acceleration values for linear and angular rate.
 */
void get_raw_acc_gyro (int16_t *acc, int16_t *gyro)
{
  i2c_read_reg (mpu_addr, MPU_ACCEL_XOUT_H, buffer, 14);

  acc[0] = (((uint16_t)buffer[0]) << 8) | buffer[1];
  acc[1] = (((uint16_t)buffer[2]) << 8) | buffer[3];
  acc[2] = (((uint16_t)buffer[4]) << 8) | buffer[5];

  gyro[0] = (((uint16_t)buffer[8]) << 8) | buffer[9];
  gyro[1] = (((uint16_t)buffer[10]) << 8) | buffer[11];
  gyro[2] = (((uint16_t)buffer[12]) << 8) | buffer[13];

  if (acc[0] > 32767) { acc[0] -= 65536; }
  if (acc[1] > 32767) { acc[1] -= 65536; }
  if (acc[2] > 32767) { acc[2] -= 65536; }

  if (gyro[0] > 32767) { gyro[0] -= 65536; }
  if (gyro[1] > 32767) { gyro[1] -= 65536; }
  if (gyro[2] > 32767) { gyro[2] -= 65536; }
}


/**
 *  Convert raw linear acceleration and angular rate values into g and deg/sec.
 */
void get_acc_gyro (float *acc, float *gyro)
{
  int16_t raw_acc[3] = {0}, raw_gyro[3] = {0};

  get_raw_acc_gyro (raw_acc, raw_gyro);

  acc[0] = (raw_acc[0] / 32768.0) * accel_sens;
  acc[1] = (raw_acc[1] / 32768.0) * accel_sens;
  acc[2] = (raw_acc[2] / 32768.0) * accel_sens;

  gyro[0] = (raw_gyro[0] / 32768.0) * gyro_sens;
  gyro[1] = (raw_gyro[1] / 32768.0) * gyro_sens;
  gyro[2] = (raw_gyro[2] / 32768.0) * gyro_sens;

#ifdef SETBIAS
  add_accel_bias (acc);
#endif

#ifdef DENOISE
  denoise_values (DENOISE_LIM_ACCEL, acc);
  denoise_values (DENOISE_LIM_GYRO, gyro);
#endif
}


/**
 *  Get raw acceleration values for linear and angular rate with temperature.
 */
void get_raw_acc_gyro_tempc (int16_t *acc, int16_t *gyro, int16_t *tempc)
{
  i2c_read_reg (mpu_addr, MPU_ACCEL_XOUT_H, buffer, 14);

  acc[0] = (((uint16_t)buffer[0]) << 8) | buffer[1];
  acc[1] = (((uint16_t)buffer[2]) << 8) | buffer[3];
  acc[2] = (((uint16_t)buffer[4]) << 8) | buffer[5];

  *tempc = (((uint16_t)buffer[6]) << 8) | buffer[7];

  gyro[0] = (((uint16_t)buffer[8]) << 8) | buffer[9];
  gyro[1] = (((uint16_t)buffer[10]) << 8) | buffer[11];
  gyro[2] = (((uint16_t)buffer[12]) << 8) | buffer[13];

  if (acc[0] > 32767) { acc[0] -= 65536; }
  if (acc[1] > 32767) { acc[1] -= 65536; }
  if (acc[2] > 32767) { acc[2] -= 65536; }

  if (*tempc > 32767) { *tempc -= 65536; }

  if (gyro[0] > 32767) { gyro[0] -= 65536; }
  if (gyro[1] > 32767) { gyro[1] -= 65536; }
  if (gyro[2] > 32767) { gyro[2] -= 65536; }
}


/**
 *  Convert raw linear acceleration, angular rate and temperature values into
 *  g, deg/sec and degrees C.
 */
void get_acc_gyro_tempc (float *acc, float *gyro, float *tempc)
{
  int16_t raw_acc[3] = {0}, raw_gyro[3] = {0}, raw_tempc = 0;

  get_raw_acc_gyro_tempc (raw_acc, raw_gyro, &raw_tempc);

  acc[0] = (raw_acc[0] / 32768.0) * accel_sens;
  acc[1] = (raw_acc[1] / 32768.0) * accel_sens;
  acc[2] = (raw_acc[2] / 32768.0) * accel_sens;

  *tempc = (raw_tempc - room_temp_offset) / 333.87 + 21.;

  gyro[0] = (raw_gyro[0] / 32768.0) * gyro_sens;
  gyro[1] = (raw_gyro[1] / 32768.0) * gyro_sens;
  gyro[2] = (raw_gyro[2] / 32768.0) * gyro_sens;

#ifdef SETBIAS
  add_accel_bias (acc);
#endif

#ifdef DENOISE
  denoise_values (DENOISE_LIM_ACCEL, acc);
  denoise_values (DENOISE_LIM_GYRO, gyro);
#endif
}


#ifdef MPU9250
/** Code specific to the AK8953 magnetometer on the MPU9250 */

/**
 *  Check whether AK8953 is available on the I2C BUS
 */
bool ak8963_i2c_check (void)
{
  uint8_t data;

  i2c_read_reg (mag_addr, AK8963_WAI, &data, 1);

  if (data != AK8963_WAI_DATA) {
    return false;
  }

  return true;
}


/**
 *  Initialize the AK8963 magnetometer CNTL1 by placing the magnetometer in
 *  FUSE mode to read the Axis Sensitivity Adjustment (ASA) values from
 *  FUSE_ROM, power-down the chip between mode changes and then place the
 *  magnetometer in 16-bit adc and continual measurement 2 for normal
 *  operations. Save the computed adjustments factorsin asax, asay, asaz.
 *  (Page 51 & 52 of Register_Map)
 */
bool ak8963_initialize (void)
{
  uint8_t cntl1_mode = AK8963_CNTL1_OUT_BITS | AK8963_CNTL1_CONT_MSRMT2,
          fuse_mode = AK8963_CNTL1_OUT_BITS | AK8963_CNTL1_FUSE_ROM,
          reset = 0x00;

  /* confirm magnetometer found on I2C bus or return error */
  if (!ak8963_i2c_check()) {
    return false;
  }

  /* set AK8963 in FUSE mode:
   *   write fuse_mode to AK8963_CNTL1
   *   read axis sensitivity adjustments into buffer from AK8963_ASAX
   */
  i2c_write_reg (mag_addr, AK8963_CNTL1, &fuse_mode, sizeof fuse_mode);
  sleep_ms (1);

  i2c_read_reg (mag_addr, AK8963_ASAX, buffer, 3);
  sleep_ms (1);

  /* power down ak8963 before mode change */
  i2c_write_reg (mag_addr, AK8963_CNTL1, &reset, sizeof reset);
  sleep_ms (10);

  /* set 16-bit output and continuous measurement mode 2 */
  i2c_write_reg (mag_addr, AK8963_CNTL1, &cntl1_mode, sizeof cntl1_mode);
  sleep_ms (1);

  /* save sensitivity adjustment factors from the per-axis FUSE_ROM ASA as:
   *
   *  asaX = (ASA - 128) * 0.5 / 128. + 1
   */
  asax = ((buffer[0] - 128) * 0.5) / 128. + 1;
  asay = ((buffer[1] - 128) * 0.5) / 128. + 1;
  asaz = ((buffer[2] - 128) * 0.5) / 128. + 1;

  return true;
}


/**
 *  Get raw magnetometer orientation values stored in two's-compliment
 *  little-endian format.
 */
void get_raw_orientation (int16_t *data)
{
  uint_fast8_t n = 0;

  do {  /* read AK8963_STI (status 1) until DRDY 1 (data ready) */
    i2c_read_reg (mag_addr, AK8963_STI, buffer, 1);
    n += 1;
  } while (!buffer[0] && n < DRDY_LIMIT);

  /* read magnetometer values and AK8963_ST2 that follows */
  i2c_read_reg (mag_addr, AK8963_HXL, buffer, 7);

  data[0] = buffer[0] | ((int16_t)buffer[1] << 8);
  data[1] = buffer[2] | ((int16_t)buffer[3] << 8);
  data[2] = buffer[4] | ((int16_t)buffer[5] << 8);
}


/**
 *  Convert raw magnetometer values into mx, my, mz in uT
 */
void get_orientation (float *mag)
{
  int16_t raw[3] = {0};
  float mag_sens = MAG_SENSE;         /* magnetometer sensitivity: 4800 uT */

  get_raw_orientation (raw);

  /* convert to uT */
  mag[0] = (raw[0] / 32767.0) * mag_sens * asax;
  mag[1] = (raw[1] / 32767.0) * mag_sens * asay;
  mag[2] = (raw[2] / 32767.0) * mag_sens * asaz;
}
#endif

