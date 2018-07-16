#include <driver/i2c.h>
#include <esp_err.h>

#include "unity.h"
#include "TRB_INA219.h"

#include <esp_log.h>

#define VALUE_IS_NEGATIVE 1

struct ina219_dev dev;
const uint8_t i2c_address = 0x40;
uint16_t reg_value;
ina219_mode_t mode;
ina219_range_t range;
ina219_pga_gain_t gain;
ina219_resolution_t res;
float voltage;
float milliv;

static char component[] = "[TRB_INA219]";

esp_err_t
i2c_init()
{
	i2c_config_t i2c_config;

	i2c_config.mode = I2C_MODE_MASTER;
	i2c_config.sda_io_num = GPIO_SDA;
	i2c_config.scl_io_num = GPIO_SCL;
	i2c_config.sda_pullup_en = GPIO_PULLUP_DISABLE;
	i2c_config.scl_pullup_en = GPIO_PULLUP_DISABLE;
	i2c_config.master.clk_speed = 400000L; // 400KHz

	TEST_ASSERT_EQUAL_INT8(0, i2c_param_config(I2C_NUM_0, &i2c_config));
	TEST_ASSERT_EQUAL_INT8(0, i2c_driver_install(I2C_NUM_0, i2c_config.mode, 0, 0, 0));
	return 0;
}

TEST_CASE("ina219_create", component)
{
	dev = ina219_create(i2c_address);
	TEST_ASSERT_EQUAL_UINT8(i2c_address, dev.address);
	TEST_ASSERT_EQUAL(3, dev.max_expected_i);
	TEST_ASSERT_EQUAL(1, dev.shunt_r * 10);
}

TEST_CASE("i2c_init", "[TRB_INA219]")
{
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, i2c_driver_delete(I2C_NUM_0));
}

TEST_CASE("ina219_reset", "[TRB_INA219]")
{
	dev = ina219_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));
	TEST_ASSERT_EQUAL_INT8(0, ina219_read16(&dev, TRB_INA219_REG_CONFIG, &reg_value));
	TEST_ASSERT_EQUAL_UINT16_MESSAGE(TRB_INA219_REG_CONFIG_DEFAULT, reg_value, "TRB_INA219_REG_CONFIG has TRB_INA219_REG_CONFIG_DEFAULT value");
	TEST_ASSERT_EQUAL_INT8(0, i2c_driver_delete(I2C_NUM_0));
}

TEST_CASE("ina219_decomplement", "[TRB_INA219]")
{
	TEST_ASSERT_EQUAL(-32000, ina219_decomplement(0b1000001100000000, VALUE_IS_NEGATIVE));
}

TEST_CASE("ina219_get_bits_from_mask", "[TRB_INA219]")
{
	for (uint8_t i = 0; i < 16; i++) {
		TEST_ASSERT_EQUAL_INT8(i, ina219_get_bits_from_mask(1 << i));
	}
}

TEST_CASE("ina219s_set_bits", "[TRB_INA219]")
{

	dev = ina219_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));

	TEST_ASSERT_EQUAL_UINT16_MESSAGE(TRB_INA219_REG_CONFIG_DEFAULT, reg_value, "TRB_INA219_REG_CONFIG has TRB_INA219_REG_CONFIG_DEFAULT value");
	TEST_ASSERT_EQUAL_INT8(0, ina219_set_bits(&dev, TRB_INA219_REG_CONFIG, (1 << 13), 0));
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_bits(&dev, TRB_INA219_REG_CONFIG, (1 << 13), &reg_value));
	TEST_ASSERT_EQUAL_UINT16(0, reg_value);
	TEST_ASSERT_EQUAL_INT8(0, ina219_set_bits(&dev, TRB_INA219_REG_CONFIG, (1 << 13), 1));
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_bits(&dev, TRB_INA219_REG_CONFIG, (1 << 13), &reg_value));
	TEST_ASSERT_EQUAL_UINT16(1, reg_value);

	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));
	TEST_ASSERT_EQUAL_INT8(0, i2c_driver_delete(I2C_NUM_0));
}

TEST_CASE("ina219_read16", "[TRB_INA219]")
{
	dev = ina219_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));

	TEST_ASSERT_EQUAL_INT8(0, ina219_read16(&dev, TRB_INA219_REG_CONFIG, &reg_value));
	TEST_ASSERT_EQUAL_INT16_MESSAGE(TRB_INA219_REG_CONFIG_DEFAULT, reg_value,
	    "TRB_INA219_REG_CONFIG has default value after reset");
	TEST_ASSERT_EQUAL(0, ina219_get_mode(&dev, &mode));
	TEST_ASSERT_EQUAL(TRB_INA219_MODE_SHUNT_BUS_CONTINUOUS, mode);
	TEST_ASSERT_EQUAL(0, ina219_get_bus_voltage_range(&dev, &range));
	TEST_ASSERT_EQUAL((ina219_range_t)TRB_INA219_BUS_VOLTAGE_RANGE_32V, range);
	TEST_ASSERT_EQUAL(0, ina219_get_pga_gain(&dev, &gain));
	TEST_ASSERT_EQUAL(TRB_INA219_PGA_GAIN_320MV, gain);
	TEST_ASSERT_EQUAL(0, ina219_get_sadc_value(&dev, &res));
	TEST_ASSERT_EQUAL(TRB_INA219_RESOLUTION_12BIT_1, res);
	TEST_ASSERT_EQUAL(0, ina219_get_badc_value(&dev, &res));
	TEST_ASSERT_EQUAL(TRB_INA219_RESOLUTION_12BIT_1, res);

	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));
	i2c_driver_delete(I2C_NUM_0);
}

TEST_CASE("set_pga_gain", "[TRB_INA219]")
{
	dev = ina219_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));

	ina219_pga_gain_t gains[] = {
		TRB_INA219_PGA_GAIN_40MV,
		TRB_INA219_PGA_GAIN_80MV,
		TRB_INA219_PGA_GAIN_160MV,
		TRB_INA219_PGA_GAIN_320MV
	};
	for (uint8_t i = 0; i <= sizeof(gains) / sizeof(gains[0]) - 1; i++) {
		dev.gain = gains[i];
		TEST_ASSERT_EQUAL_INT8(0, ina219_configure(&dev));
		TEST_ASSERT_EQUAL_INT8(0, ina219_get_pga_gain(&dev, &gain));
		TEST_ASSERT_EQUAL_INT8(gains[i], gain);
	}
	dev.gain = TRB_INA219_PGA_GAIN_80MV;
	TEST_ASSERT_EQUAL_INT8(0, ina219_configure(&dev));
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_pga_gain(&dev, &gain));
	TEST_ASSERT_EQUAL_INT8(TRB_INA219_PGA_GAIN_80MV, gain);

	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));
	i2c_driver_delete(I2C_NUM_0);
}

TEST_CASE("ina219_set", "[TRB_INA219]")
{
	dev = ina219_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));

	TEST_ASSERT_EQUAL(TRB_INA219_BUS_VOLTAGE_RANGE_16V, ina219_get_bus_voltage_range(&dev, &range));
	ina219_mode_t modes[] = {
		TRB_INA219_MODE_POWERDOWN,
		TRB_INA219_MODE_SHUNT_TRIGGERED,
		TRB_INA219_MODE_BUS_TRIGGERED,
		TRB_INA219_MODE_SHUNT_BUS_TRIGGERED,
		TRB_INA219_MODE_ADC_OFF,
		TRB_INA219_MODE_SHUNT_CONTINUOUS,
		TRB_INA219_MODE_BUS_CONTINUOUS,
		TRB_INA219_MODE_SHUNT_BUS_CONTINUOUS
	};
	for (uint8_t i = 0; i <= sizeof(modes) / sizeof(modes[0]) - 1; i++) {
		dev.mode = modes[i];
		TEST_ASSERT_EQUAL_INT8(0, ina219_configure(&dev));
		TEST_ASSERT_EQUAL_INT8(0, ina219_get_mode(&dev,&mode));
		TEST_ASSERT_EQUAL_INT8(modes[i], mode);
	}
	ina219_pga_gain_t gains[] = {
		TRB_INA219_PGA_GAIN_40MV,
		TRB_INA219_PGA_GAIN_80MV,
		TRB_INA219_PGA_GAIN_160MV,
		TRB_INA219_PGA_GAIN_320MV
	};
	for (uint8_t i = 0; i <= sizeof(gains) / sizeof(gains[0]) - 1; i++) {
		dev.gain = gains[i];
		TEST_ASSERT_EQUAL_INT8(0, ina219_configure(&dev));
		TEST_ASSERT_EQUAL_INT8(0, ina219_get_pga_gain(&dev, &gain));
		TEST_ASSERT_EQUAL_INT8(gains[i], gain);
	}
	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));
	i2c_driver_delete(I2C_NUM_0);
}

TEST_CASE("ina219_regular_use_case", "[TRB_INA219]")
{
	dev = ina219_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));

	TEST_ASSERT_EQUAL_INT8(0, ina219_read16(&dev, TRB_INA219_REG_CURRENT, &reg_value));
	TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, reg_value, "after reset TRB_INA219_REG_CURRENT is supposed to be zero");

	dev.gain = TRB_INA219_PGA_GAIN_40MV;
	TEST_ASSERT_EQUAL_INT8(0, ina219_configure(&dev));
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_pga_gain(&dev, &gain));
	TEST_ASSERT_EQUAL_INT8(TRB_INA219_PGA_GAIN_40MV, gain);
	TEST_ASSERT_EQUAL_INT8(0, ina219_set_calibration(&dev));

	for (uint8_t i = 0; i <= 100; i++) {
		if (i == 100)
			TEST_FAIL_MESSAGE("timeout while waiting for ina219_conversion_is_ready()");
		if (ina219_conversion_is_ready(&dev) == TRB_INA219_CONVERSION_IS_READY)
			break;
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_bus_voltage(&dev, &voltage));

	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));
	i2c_driver_delete(I2C_NUM_0);
}

TEST_CASE("ina219_calc_calibration", "[TRB_INA219]")
{
	dev = ina219_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, ina219_calc_calibration(&dev));
	TEST_ASSERT_EQUAL_HEX(0x1179, dev.cal);

	dev.max_expected_i = 1.0;
	TEST_ASSERT_EQUAL_INT8(0, ina219_calc_calibration(&dev));
	TEST_ASSERT_EQUAL_HEX(0x346c, dev.cal);
}

TEST_CASE("ina219_get_sensor_values", "[TRB_INA219]")
{
	dev = ina219_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	//TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));
	//TEST_ASSERT_EQUAL_INT8(0, ina219_read16(&dev, TRB_INA219_REG_CONFIG, &reg_value));

	/* set non-default values */
	dev.max_expected_i = 0.5; // Amps
	dev.range = TRB_INA219_BUS_VOLTAGE_RANGE_16V;
	dev.gain = TRB_INA219_PGA_GAIN_40MV;
	dev.bus_adc_resolution = TRB_INA219_RESOLUTION_11BIT_1;
	dev.shunt_adc_resolution = TRB_INA219_RESOLUTION_11BIT_1;
	dev.mode = TRB_INA219_MODE_SHUNT_BUS_TRIGGERED;

	/* write the configuration to TRB_INA219_REG_CONFIG */
	TEST_ASSERT_EQUAL_INT8(0, ina219_configure(&dev));

	/* make sure the changes has been applied */

	/* range */
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_bus_voltage_range(&dev, &range));
	TEST_ASSERT_EQUAL_UINT8(TRB_INA219_BUS_VOLTAGE_RANGE_16V, range);

	/* gain */
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_pga_gain(&dev, &gain));
	TEST_ASSERT_EQUAL_UINT8(TRB_INA219_PGA_GAIN_40MV, gain);

	/* mode */
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_mode(&dev, &mode));
	TEST_ASSERT_EQUAL_UINT8(TRB_INA219_MODE_SHUNT_BUS_TRIGGERED, mode);

	/* BADC */
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_badc_value(&dev, &res));
	TEST_ASSERT_EQUAL_UINT8(TRB_INA219_RESOLUTION_11BIT_1, res);

	/* SADC */
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_sadc_value(&dev, &res));
	TEST_ASSERT_EQUAL_UINT8(TRB_INA219_RESOLUTION_11BIT_1, res);

	/* do the calibration */
	TEST_ASSERT_EQUAL_INT8(0, ina219_set_calibration(&dev));

	/* should be ready soon after calibration */
	TEST_ASSERT_EQUAL_INT8(TRB_INA219_CONVERSION_IS_READY, ina219_conversion_is_ready(&dev));

	/* the circuit should not cause over-flowing */
	TEST_ASSERT_EQUAL_INT8(TRB_INA219_IS_NOT_OVERFLOWED, ina219_conversion_is_overflowed(&dev));

	/* read the all sensor values at once */
	TEST_ASSERT_EQUAL_INT8(0, ina219_get_sensor_values(&dev));

	/* the result should be something like:
	 *
	 * Bus Voltage:   3.30 V
	 * Shunt Voltage: 0.13 mV
	 * Power:         4.00 mW
	 * Current:       1.30 mA
	 */
	ESP_LOGI(__func__, "bus_voltage: %0.2fV", dev.bus_voltage);
	ESP_LOGI(__func__, "shunt_voltage: %0.2fmV", dev.shunt_voltage * 1000);
	ESP_LOGI(__func__, "power: %0.2fmW", dev.power * 1000);
	ESP_LOGI(__func__, "current: %0.2fmA", dev.current * 1000);

	TEST_ASSERT_INT8_WITHIN(3, 33, (int8_t)(dev.bus_voltage * 10));
	TEST_ASSERT_INT8_WITHIN(3, 13, (int8_t)(dev.shunt_voltage * 1000 * 100));
	TEST_ASSERT_INT8_WITHIN(1,  4, (int8_t)(dev.power * 1000));
	TEST_ASSERT_INT8_WITHIN(3, 13, (int8_t)(dev.current * 1000 * 10));

	TEST_ASSERT_EQUAL_INT8(0, ina219_reset(&dev));
	i2c_driver_delete(I2C_NUM_0);
}
