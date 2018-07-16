# `INA219C`

A C driver for INA219 IC.

## Choosing I2C implementation

Define one of variables below. The variable must be defined as build flag,
not in the code.

| Variable name | Platform | Framework | Driver |
|---------------|----------|-----------|--------|
| `INA219C_ESP_IDF.h` | `espressif32` | `esp-idf` | [Native I2C API](https://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/i2c.html) |
| `INA219C_Arduino_Wire.h` | `espressif32`, `espressif8266`, `atmelavr` | `arduino` | [`Wire`](https://www.arduino.cc/en/Reference/Wire) |
| `INA219C_I2C_BRZO` | `espressif8266` | `arduino` | [`Brzo I2C`](https://github.com/pasko-zh/brzo_i2c) |

## Usage

Configure `I2C` bus, such as `i2c_param_config()`, or `Wire.begin()`. The
library does not do that for you.

Create `struct` `ina219c_dev` by `ina219c_create()`. The `struct` has some
default values configured.

Set the configurations in the `struct` to Configuration Register by
`ina219c_configure()`.

Set calibration value to Calibration Register by `ina219c_set_calibration`. At
this point, the device is ready for measurement.

Read all the values from the device by `ina219c_get_sensor_values`. The values
are saved in the `struct`.

```c
/*
 * configure I2C here
 */

struct ina219c_dev dev;
dev = ina219c_create(0x40);
ina219c_configure(&dev);
ina219c_set_calibration(&dev);
ina219c_get_sensor_values(&dev);

/*
 * do something with dev.current, dev.power, etc ...
 */
```

For more functions, see [src/INA219C.h](src/INA219C.h).

Examples can be found under [examples](examples).

## Unit testing

### Hardware

* ESP32 development board
* INA219 breakout board
* A pair of pull-up registers for I2C bus
* A breadboard and some jumper wires

### Building the test application and running the tests

On BSD variants, replace `make` with `gmake(1)` in the following instructions.

Make sure necessary tool-chains have been installed. [Standard Setup of
Toolchain for Linux](http://esp-idf.readthedocs.io/en/latest/get-started/linux-setup.html)
lists the required packages.

Install `esp-idf`.

```
git clone https://github.com/espressif/esp-idf.git
```

Set `IDF_PATH` environment variable to the path to `esp-idf` sources.

```
export IDF_PATH=/usr/home/trombik/github/trombik/esp-idf
```

Configure the unit test application. Notably, path to serial port. Other than
that, it is assumed in this example that all default values are chosen.

```
make menuconfig
```

Build a circuit and connect it to the machine (TODO draw the schematic).

Build the unit test application.

```
make TEST_COMPONENTS="INA219C" flash
```

Establish USB serial connection.

```
make monitor
```

Or any other serial console command, such as `cu(1)`. Make sure:

* path to USB serial device file exists and is correct
  (`CONFIG_ESPTOOLPY_PORT` in `sdkconfig`)
* the USB serial device has read and write permissions for the current unix
  user
* Serial baud rate is correct (115200 baud by default, or
  `CONFIG_MONITOR_BAUD_115200B=y` in `sdkconfig`)

An example on BSDs:

```
cu -s 115200 -l /dev/cuaU0
```

After manually resetting the device, you will see something like the
following:

```
... boot log ...
I (240) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (246) heap_init: At 4008977C len 00016884 (90 KiB): IRAM
I (252) cpu_start: Pro cpu start user code
I (270) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.

Press ENTER to see the list of tests.
```

After hitting enter key, it will show the test menu.

```
Here's the test menu, pick your combo:
(1)	"first_test" [INA219C]
...
```

Entering the number starts the selected test. A special symbol `*` runs all
tests.
