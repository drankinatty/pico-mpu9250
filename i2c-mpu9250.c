/**
 *  Raspberry Pi Pico MPU example, extends pico-examples/mpu6050_i2c to
 *  conditionally incorporate magnetometer on MPU9250 and computes
 *  calibrated values for all sensors providing acceleration in G,
 *  gyroscope rates in deg./sec. and magnetometer readings in uT.
 *
 *  Copyright (c) David C. Rankin, 2022
 *  License: GPLv2
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#ifdef PICOTOOL
#include "pico/binary_info.h"
#endif

#include "pico_mpu.h"   /* MPU library header */

/* Example code to talk to a MPU6050/9250 MEMS accelerometer, gyroscope and
   magnetometer (MPU9250 only)

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (Pico GPIO4 (pin 6)) -> SDA on MPU board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (Pico GPIO5 (pin 7)) -> SCL on MPU board
   3.3v (pin 36) -> VCC on MPU6050/9250 board
   GND (pin 38)  -> GND on MPU6050/9250 board

   Default I2C addresses:

      MPU6050/9250:            0x68
      AK8963 (magnetometer):   0x0C
*/

/** initialize I2C on default gpio 4, 5 (pins 6, 7) */
void i2c_init_default (uint baudrate)
{
  i2c_init (i2c_default, baudrate);

  gpio_set_function (PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function (PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

  gpio_pull_up (PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up (PICO_DEFAULT_I2C_SCL_PIN);
}

/** simple ANSI escapes to clear 5 lines for sensor display */
void ANSI_clear_lines (void)
{
    for (int i = 0; i < 5; i++) {
      puts ("\033[2K");
    }
    fputs ("\033[5A", stdout);
    fflush (stdout);
}


int main (void) {

  stdio_init_all();

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
  puts("Default I2C pins were not defined");
#else

#ifdef PICOTOOL
  /* Make the I2C pins available to picotool */
  bi_decl (bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
           PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
#endif

#ifdef MPU9250
  puts ("\033[?25lHello, MPU9250! Reading Sensor Data...\033[0K\n\033[2K");
#else
  puts ("\033[?25lHello, MPU6050! Reading Sensor Data...\033[0K\n\033[2K");
#endif

  ANSI_clear_lines();   /* clear output display lines (optional) */

  /* initialize default I2C config */
  i2c_init_default (400 * 1000);

  /* initialize the MPU verifying presence on I2C bus and initializing with
   * full-scale rates of 250 deg/sec and 2 g.
   */
  if (!mpu_init_default()) {
    puts ("error: MPU not found on I2C bus.");
    return 0;
  }
#ifdef MPU9250
  /* initialize magnetometer */
  if (!ak8963_initialize()) {
    puts ("error: ak8963 mag not found on I2C bus.");
    return 0;
  }
#endif

  /*  x, y, z arrays for accelerometer, gyroscope and magnetometer,
   *  and variables for temperature in Deg. C and F.
   */
  float acc[3], gyro[3], mag[3], temp, tempf;

  while (1) {
    /* read MPU sensor values and convert to deg./sec., g, and deg. C */
    get_acc_gyro_tempc (acc, gyro, &temp);
    tempf = temp * 9. / 5. + 32.;

    printf ("  Acc.    x: % 6.1f,   Y: % 6.1f,   z: % 6.1f\n"
            "  Gyro.   x: % 6.1f,   y: % 6.1f,   z: % 6.1f\n",
            acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
#ifdef MPU9250
    /* read magnetometer sensor values and convert to uT */
    get_orientation (mag);

    printf ("  Mag.    x: % 6.1f,   y: % 6.1f,   z: % 6.1f\n\n"
            "  Temp.   %.1f C   %.1f F\r\033[4A",
            mag[0], mag[1], mag[2], temp, tempf);
#else
    /* output temp only for MPU6050 */
    printf ("\n  Temp.   %.1f C   %.1f F\r\033[3A", temp, tempf);
#endif

    sleep_ms(150);
  }

#endif
}
