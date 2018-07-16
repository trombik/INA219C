# `TRB_INA219`

A C driver for INA219 IC.

## Choosing I2C implementation

Define one of variables below. The variable must be defined as build flag,
not in the code.

| Variable name | Platform | Framework | Driver |
|---------------|----------|-----------|--------|
| `TRB_INA219_I2C_ESP_IDF` | `espressif32` | `esp-idf` | [Native I2C API](https://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/i2c.html) |
| `TRB_INA219_I2C_WIRE` | `espressif32`, `espressif8266`, `atmelavr` | `arduino` | [`Wire`](https://www.arduino.cc/en/Reference/Wire) |
| `TRB_INA219_I2C_BRZO` | `espressif8266` | `arduino` | [`Brzo I2C`](https://github.com/pasko-zh/brzo_i2c) |

## Usage

Configure `I2C` bus, such as `i2c_param_config()`, or `Wire.begin()`. The
library does not do that for you.

Create `struct` `ina219_dev` by `ina219_create()`. The `struct` has some
default values configured.

Set the configurations in the `struct` to Configuration Register by
`ina219_configure()`.

Set calibration value to Calibration Register by `ina219_set_calibration`. At
this point, the device is ready for measurement.

Read all the values from the device by `ina219_get_sensor_values`. The values
are saved in the `struct`.

```c

#include <TRB_INA219.h>

/*
 * configure I2C here
 */

struct ina219_dev dev;
dev = ina219_create(0x40);
ina219_configure(&dev);
ina219_set_calibration(&dev);
ina219_get_sensor_values(&dev);

/*
 * do something with dev.current, dev.power, etc ...
 */
```

For more functions, see [TRB_INA219_common.h](TRB_INA219_common.h).

Examples can be found under [examples](examples).

## Unit testing

See [UNIT_TEST.md](UNIT_TEST.md).
