# pico-mpu9250
Raspberry Pi Pico API for MPU 6050/9250

This example and MPU library for the Raspberry Pi Pico expands on the pico-examples/i2c/mpu6050_i2c example to provide initialization for the MPU9250 magnetometer. The example will work with either the MPU6050 or MPU9250 depending on the preprocessor defines provided (explained below). Additionally, instead of simply dumping raw register values, this code draws from the datasheet and register-map documents to properly convert the raw register values to acceleration in G's, angular gyroscopic rates in Deg./Sec. and magnetometer values in uT.

Both the MPU MEMS sensors and the AK8963 magnetometer are initialized properly validating the chips are active on the I2C bus and properly identifying by reading the `WHO_AM_I`, `WAI` register data before initializing each and setting the `DLPF` bandwidth and gyro `FS_SEL` sensitivity to (default 250 deg./secx and 2 G.) and provides an initialization function that allows the user to set the values to the desired full-scale range and sensitivity.

The MPU code also allows for denoise limits to be applied to prevent value bounce around zero and independent bias to be applied to each axis to calibrate zero. There is a room temperature adjustment that can be set to convert the chip-temperature to room-temperature (default offset of 2.5 deg. C). There is also an example using pre-processor defines to set the per-axis biases on a per-MPU board basis. (each board and sensor will have slightly different output just due to differences during manufacture.

The preprocessor defines listed in the `CMakeLists.txt` file are:

*  `MPU9250` - define to activate magnetometer (not on MPU6050)
*  `DENOISE` - define to apply denoise limits to prevent bounce around zero
*  `SETBIAS` - apply chip-specific axiz bias to zero axis
*  `MPUCHIP` - (apply values specific to individual MPU board
*  `PICOTOOL` - make i2c pins available to picotool

You can select the sensor behavior you want by including or not including some, none or all of the defines in the `CMakeLists.txt` file in the `add_compile_definitions()` command.

The output values are updated in place in the terminal using ANSI escape sequences. If the terminal you use doesn't support ANSI escapes, you can simply remove them in the example source file.

The code builds as all other pico examples build. Create an out-of-source `build` directory, change to that directory and then issue the `cmake path-to-source-dir` command, followed by `make -j4` (or just `make`). You may need to adjust the `pico_enable_stdio_usb` and `pico_enable_stdio_uart` settings in the `CMakeLists.txt` file depending on whether you have a single pico and are flashing using the pico bootsel, displaying in a terminal over USB. If you are using the pico with a debug prope over the SWI serial wire connection (either a Pi or another pico) and outputting values over UART, no change is needed (current default).

A wiring diagram for connecting the pico and MPU is provided in the file `mpu6050_i2c_bb.png` (which is the same diagram provided in the pico-examples example)
