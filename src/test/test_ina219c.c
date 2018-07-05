#define UNITY_INCLUDE_FLOAT

#include <driver/i2c.h>
#include <esp_err.h>

#include "unity.h"
#include "INA219C.h"

esp_err_t
i2c_init(const gpio_num_t sda, const gpio_num_t scl)
{
	i2c_config_t i2c_config;
	esp_err_t r;

	i2c_config.mode = I2C_MODE_MASTER;
	i2c_config.sda_io_num = sda;
	i2c_config.scl_io_num = scl;
	i2c_config.sda_pullup_en = GPIO_PULLUP_DISABLE;
	i2c_config.scl_pullup_en = GPIO_PULLUP_DISABLE;
	i2c_config.master.clk_speed = 400000L; // 400KHz

	r = i2c_param_config(I2C_NUM_0, &i2c_config);
	ESP_ERROR_CHECK(r);
	i2c_driver_install(I2C_NUM_0, i2c_config.mode, 0, 0, 0);
	ESP_ERROR_CHECK(r);
	return r;
}

TEST_CASE("first_test", "[INA219C]")
{
	struct ina219c_dev dev;
	const uint8_t i2c_address = 0x40;
	dev = ina219c_create(i2c_address);
	TEST_ASSERT_EQUAL_UINT8(i2c_address, dev.address);
}

TEST_CASE("i2c_init", "[INA219C]")
{
	TEST_ASSERT(i2c_init(GPIO_SDA, GPIO_SCL) == 0);
}
