#define WITH_REPEATED_START true
#define WITHOUT_REPEATED_START false

#include <Arduino.h>
#include <brzo_i2c.h>

#if defined(__cplusplus)
extern "C" {
#endif

static uint16_t scl_freq = 100; // in KHz

uint16_t
ina219_brzo_get_scl_freq()
{
	return scl_freq;
}

uint16_t
ina219_brzo_set_scl_freq(const uint16_t freq)
{
	scl_freq = freq;
	return scl_freq;
}

void
ina219_delay_ms(const uint32_t period)
{
	uint32_t now = millis();
	/* do NOOP instead of delay(), which does not block */
	while (millis() < now + period) {
#if defined(ESP8266) || defined(ESP32)
		/* avoid watchdog timeout */
		yield();
#endif
	}
}

int32_t
ina219_read(const uint8_t dev_id, const uint8_t reg_addr, uint8_t *reg_data, const uint8_t len)
{
	int32_t result = 0;
	uint8_t reg_addr_copy;

	reg_addr_copy = reg_addr;

	/* start transaction */
	brzo_i2c_start_transaction(dev_id, ina219_brzo_get_scl_freq());
	/* write register address with repeated start */
	brzo_i2c_write(&reg_addr_copy, 1, WITH_REPEATED_START);
	/* read the values */
	brzo_i2c_read(reg_data, len, WITHOUT_REPEATED_START);
	/* end the transaction */
	result = brzo_i2c_end_transaction();
	return result;
}

int32_t
ina219_write(const uint8_t dev_id, const uint8_t reg_addr, uint8_t *reg_data, const uint8_t len)
{
	int32_t result = 0;
	uint8_t *buffer;

	/* the buffer needs an extra byte in the first element */
	buffer = (uint8_t *)malloc(sizeof(uint8_t) * (len + 1));
	if (buffer == NULL)
		return -1;

	buffer[0] = reg_addr;
	for (uint8_t i = 1; i <= len; i++)
		buffer[i] = reg_data[i - 1];

	/* start transaction */
	brzo_i2c_start_transaction(dev_id, ina219_brzo_get_scl_freq());
	/* write register address + the value */
	brzo_i2c_write(buffer, len + 1, WITHOUT_REPEATED_START);
	/* end end transaction */
	result = brzo_i2c_end_transaction();
	free(buffer);
	ina219_delay_ms(10);
	return result;
}

#if defined(__cplusplus)
}
#endif
