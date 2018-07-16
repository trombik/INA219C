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
make TEST_COMPONENTS="TRB_INA219" flash
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
(1)	"first_test" [TRB_INA219]
...
```

Entering the number starts the selected test. A special symbol `*` runs all
tests.

