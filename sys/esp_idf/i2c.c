#if defined(_TRB_INA219_ESP_IDF_h)
#include <stdint.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include "../../TRB_INA219.h"

/* I2C implementations for esp-idf */

/* I2C master will check ack from slave */
#define ACK_CHECK_ENABLE	0x01

static i2c_port_t i2c_port = I2C_NUM_0;

i2c_port_t
ina219_set_i2c_port(i2c_port_t port)
{
	i2c_port = port;
	return i2c_port;
}

i2c_port_t
ina219_get_i2c_port()
{
	return i2c_port;
}


void
ina219_delay_ms(const uint32_t period)
{
vTaskDelay(period / portTICK_PERIOD_MS);
}

int8_t
ina219_read(const uint8_t addr, const uint8_t reg, uint8_t *data, const uint8_t len)
{
	int8_t r = 0;
	i2c_cmd_handle_t command;

	command = i2c_cmd_link_create();
	/* I2C start */
	ESP_ERROR_CHECK(i2c_master_start(command));
	/* write I2C address with WRITE */
	ESP_ERROR_CHECK(i2c_master_write_byte(command, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE));
	/* write register address */
	ESP_ERROR_CHECK(i2c_master_write_byte(command, reg, ACK_CHECK_ENABLE));
	/* repeated start */
	ESP_ERROR_CHECK(i2c_master_start(command));
	/* write I2C address with READ */
	ESP_ERROR_CHECK(i2c_master_write_byte(command, (addr << 1) | I2C_MASTER_READ, ACK_CHECK_ENABLE));
	/* read the values */
	if (len > 1)
		ESP_ERROR_CHECK(i2c_master_read(command, data, len - 1, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_read_byte(command, data + len - 1, I2C_MASTER_NACK));
	/* I2C stop */
	ESP_ERROR_CHECK(i2c_master_stop(command));
	/* start the transaction */
	/* TODO support I2C_NUM_1 */
	r = i2c_master_cmd_begin(i2c_port, command, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(command);
	if (r != ESP_OK)
		ESP_LOGE("ina219_read", "i2c_master_cmd_begin() failed: %d", r);
	return r;
}

int8_t
ina219_write(const uint8_t addr, const uint8_t reg, uint8_t *data, const uint8_t len)
{
	i2c_cmd_handle_t command;
	esp_err_t r;

	/* read-only registers */
	assert(
	    reg != INA219_REG_SHUNT && reg != INA219_REG_BUS &&
	    reg != INA219_REG_POWER && reg != INA219_REG_CURRENT);
	command = i2c_cmd_link_create();

	/* I2C start */
	ESP_ERROR_CHECK(i2c_master_start(command));
	/* write I2C address with WRITE bit */
	ESP_ERROR_CHECK(i2c_master_write_byte(command, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE));
	/* write register address */
	ESP_ERROR_CHECK(i2c_master_write_byte(command, reg, ACK_CHECK_ENABLE));
	/* write data */
	ESP_ERROR_CHECK(i2c_master_write(command, data, len, ACK_CHECK_ENABLE));
	/* I2C stop */
	ESP_ERROR_CHECK(i2c_master_stop(command));
	/* start transation */
	r = i2c_master_cmd_begin(i2c_port, command, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(command);

	if (r != ESP_OK)
		ESP_LOGE("ina219_write", "i2c_master_cmd_begin() failed: %d", r);
	/* Register contents are updated 4 micro seconds after completion of the write
	 * command */
	ina219_delay_ms(1);
	return r;
}
#endif
