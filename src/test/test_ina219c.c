#include <driver/i2c.h>
#include <esp_err.h>

#include "unity.h"
#include "INA219C.h"

#include <esp_log.h>

#define VALUE_IS_NEGATIVE 1

struct ina219c_dev dev;
const uint8_t i2c_address = 0x40;
uint16_t reg_value;
ina219c_mode mode;
ina219c_range range;
ina219c_pga_gain_t gain;
ina219_resolution_t res;
float voltage;
float milliv;

static char component[] = "[INA219C]";

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

TEST_CASE("ina219c_create", component)
{
	dev = ina219c_create(i2c_address);
	TEST_ASSERT_EQUAL_UINT8(i2c_address, dev.address);
	TEST_ASSERT_EQUAL(3, dev.max_expected_i);
	TEST_ASSERT_EQUAL(1, dev.shunt_r * 10);
}

TEST_CASE("i2c_init", "[INA219C]")
{
	TEST_ASSERT(i2c_init() == 0);
	TEST_ASSERT_EQUAL_INT8(0, i2c_driver_delete(I2C_NUM_0));
}

TEST_CASE("ina219c_reset", "[INA219C]")
{
	dev = ina219c_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219c_reset(&dev));
	TEST_ASSERT_EQUAL_INT8(0, ina219c_read16(&dev, INA219C_REG_CONFIG, &reg_value));
	TEST_ASSERT_EQUAL_UINT16_MESSAGE(INA219C_REG_CONFIG_DEFAULT, reg_value, "INA219C_REG_CONFIG has INA219C_REG_CONFIG_DEFAULT value");
	TEST_ASSERT_EQUAL_INT8(0, i2c_driver_delete(I2C_NUM_0));
}

TEST_CASE("ina219c_decomplement", "[INA219C]")
{
	TEST_ASSERT_EQUAL(-32000, ina219c_decomplement(0b1000001100000000, VALUE_IS_NEGATIVE));
}

TEST_CASE("__ina219c_get_bits_from_mask", "[INA219C]")
{
	for (uint8_t i = 0; i < 16; i++) {
		TEST_ASSERT_EQUAL_INT8(i, __ina219c_get_bits_from_mask(1 << i));
	}
}

TEST_CASE("ina219s_set_bits", "[INA219C]")
{

	dev = ina219c_create(i2c_address);
	TEST_ASSERT(i2c_init() == 0);
	TEST_ASSERT(ina219c_reset(&dev) == 0);

	TEST_ASSERT_EQUAL_UINT16_MESSAGE(INA219C_REG_CONFIG_DEFAULT, reg_value, "INA219C_REG_CONFIG has INA219C_REG_CONFIG_DEFAULT value");
	TEST_ASSERT_EQUAL_INT8(0, ina219c_set_bits(&dev, INA219C_REG_CONFIG, (1 << 13), 0));
	TEST_ASSERT_EQUAL_INT8(0, ina219c_get_bits(&dev, INA219C_REG_CONFIG, (1 << 13), &reg_value));
	TEST_ASSERT_EQUAL_UINT16(0, reg_value);
	TEST_ASSERT_EQUAL_INT8(0, ina219c_set_bits(&dev, INA219C_REG_CONFIG, (1 << 13), 1));
	TEST_ASSERT_EQUAL_INT8(0, ina219c_get_bits(&dev, INA219C_REG_CONFIG, (1 << 13), &reg_value));
	TEST_ASSERT_EQUAL_UINT16(1, reg_value);

	TEST_ASSERT(ina219c_reset(&dev) == 0);
	TEST_ASSERT_EQUAL_INT8(0, i2c_driver_delete(I2C_NUM_0));
}

TEST_CASE("ina219c_read16", "[INA219C]")
{
	dev = ina219c_create(i2c_address);
	TEST_ASSERT(i2c_init() == 0);
	TEST_ASSERT(ina219c_reset(&dev) == 0);
	TEST_ASSERT(ina219c_read16(&dev, INA219C_REG_CONFIG, &reg_value) == 0);
	TEST_ASSERT_EQUAL_INT16_MESSAGE(INA219C_REG_CONFIG_DEFAULT, reg_value,
	    "INA219C_REG_CONFIG has default value after reset");
	TEST_ASSERT_EQUAL(0, ina219c_get_mode(&dev, &mode));
	TEST_ASSERT_EQUAL(INA219C_MODE_SHUNT_BUS_CONTINUOUS, mode);
	TEST_ASSERT_EQUAL(0, ina219c_get_bus_voltage_range(&dev, &range));
	TEST_ASSERT_EQUAL((ina219c_range)INA219C_BUS_VOLTAGE_RANGE_32V, range);
	TEST_ASSERT_EQUAL(0, ina219c_get_pga_gain(&dev, &gain));
	TEST_ASSERT_EQUAL(INA219C_PGA_GAIN_320MV, gain);
	TEST_ASSERT_EQUAL(0, ina219c_get_sadc_value(&dev, &res));
	TEST_ASSERT_EQUAL(INA219C_RESOLUTION_12BIT_1, res);
	TEST_ASSERT_EQUAL(0, ina219c_get_badc_value(&dev, &res));
	TEST_ASSERT_EQUAL(INA219C_RESOLUTION_12BIT_1, res);
	i2c_driver_delete(I2C_NUM_0);
}

TEST_CASE("set_pga_gain", "[INA219C]")
{
	dev = ina219c_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219c_reset(&dev));
	ina219c_pga_gain_t gains[] = {
		INA219C_PGA_GAIN_40MV,
		INA219C_PGA_GAIN_80MV,
		INA219C_PGA_GAIN_160MV,
		INA219C_PGA_GAIN_320MV
	};
	for (uint8_t i = 0; i <= sizeof(gains) / sizeof(gains[0]) - 1; i++) {
		TEST_ASSERT_EQUAL_INT8(0, ina219c_set_pga_gain(&dev, gains[i]));
		TEST_ASSERT_EQUAL_INT8(0, ina219c_get_pga_gain(&dev, &gain));
		TEST_ASSERT_EQUAL_INT8(gains[i], gain);
	}
	TEST_ASSERT_EQUAL_INT8(0, ina219c_set_pga_gain(&dev, INA219C_PGA_GAIN_80MV));
	TEST_ASSERT_EQUAL_INT8(0, ina219c_get_pga_gain(&dev, &gain));
	TEST_ASSERT_EQUAL_INT8(INA219C_PGA_GAIN_80MV, gain);

	TEST_ASSERT_EQUAL_INT8(0, ina219c_reset(&dev));
	i2c_driver_delete(I2C_NUM_0);
}

TEST_CASE("ina219c_set", "[INA219C]")
{
	dev = ina219c_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219c_reset(&dev));
	TEST_ASSERT_EQUAL_INT8(0, ina219c_set_bus_voltage_range(&dev, INA219C_BUS_VOLTAGE_RANGE_16V));
	TEST_ASSERT_EQUAL(INA219C_BUS_VOLTAGE_RANGE_16V, ina219c_get_bus_voltage_range(&dev, &range));
	ina219c_mode modes[] = {
		INA219C_MODE_POWERDOWN,
		INA219C_MODE_SHUNT_TRIGGERED,
		INA219C_MODE_BUS_TRIGGERED,
		INA219C_MODE_SHUNT_BUS_TRIGGERED,
		INA219C_MODE_ADC_OFF,
		INA219C_MODE_SHUNT_CONTINUOUS,
		INA219C_MODE_BUS_CONTINUOUS,
		INA219C_MODE_SHUNT_BUS_CONTINUOUS
	};
	for (uint8_t i = 0; i <= sizeof(modes) / sizeof(modes[0]) - 1; i++) {
		TEST_ASSERT_EQUAL_INT8(0, ina219c_set_mode(&dev, modes[i]));
		TEST_ASSERT_EQUAL_INT8(0, ina219c_get_mode(&dev,&mode));
		TEST_ASSERT_EQUAL_INT8(modes[i], mode);
	}
	ina219c_pga_gain_t gains[] = {
		INA219C_PGA_GAIN_40MV,
		INA219C_PGA_GAIN_80MV,
		INA219C_PGA_GAIN_160MV,
		INA219C_PGA_GAIN_320MV
	};
	for (uint8_t i = 0; i <= sizeof(gains) / sizeof(gains[0]) - 1; i++) {
		TEST_ASSERT_EQUAL_INT8(0, ina219c_set_pga_gain(&dev, gains[i]));
		TEST_ASSERT_EQUAL_INT8(0, ina219c_get_pga_gain(&dev, &gain));
		TEST_ASSERT_EQUAL_INT8(gains[i], gain);
	}
	TEST_ASSERT_EQUAL_INT8(0, ina219c_reset(&dev));
	i2c_driver_delete(I2C_NUM_0);
}

TEST_CASE("ina219c_regular_use_case", "[INA219C]")
{
	dev = ina219c_create(i2c_address);
	TEST_ASSERT_EQUAL_INT8(0, i2c_init());
	TEST_ASSERT_EQUAL_INT8(0, ina219c_reset(&dev));

	TEST_ASSERT_EQUAL_INT8(0, ina219c_get_current_register(&dev, &reg_value));
	TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, reg_value, "after reset INA219C_REG_CURRENT is supposed to be zero");
	TEST_ASSERT_EQUAL_INT8(0, ina219c_get_shunt_voltage(&dev, &milliv));
	TEST_ASSERT_INT16_WITHIN_MESSAGE(32000, 0, (int16_t)(milliv * 100), "voltage reading should be +/- 320.00");

	TEST_ASSERT_EQUAL_INT8(0, ina219c_set_pga_gain(&dev, INA219C_PGA_GAIN_40MV));
	TEST_ASSERT_EQUAL_INT8(0, ina219c_set_calibration(&dev));

	for (uint8_t i = 0; i <= 100; i++) {
		if (i == 100)
			TEST_FAIL_MESSAGE("timeout while waiting for ina219c_conversion_is_ready()");
		if (ina219c_conversion_is_ready(&dev) == INA219C_CONVERSION_IS_READY)
			break;
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	TEST_ASSERT_EQUAL_INT8(0, ina219c_get_bus_voltage(&dev, &voltage));

	TEST_ASSERT_EQUAL_INT8(0, ina219c_reset(&dev));
	i2c_driver_delete(I2C_NUM_0);
}
