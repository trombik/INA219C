#include <Arduino.h>
#include <I2C.h>

#ifdef __cplusplus
extern "C" {
#endif

void
ina219_delay_ms(const uint32_t period)
{
	uint32_t now = millis();
	/* do NOOP instead of delay(), which does not block */
	while (millis() < now + period) {}
}

int8_t
ina219_read(const uint8_t dev_id, const uint8_t reg_addr, uint8_t *reg_data, const uint8_t len)
{
	/* Read register */
	return I2c.read(dev_id, reg_addr, len, reg_data);
}

int8_t
ina219_write(const uint8_t dev_id, const uint8_t reg_addr, uint8_t *reg_data, const uint8_t len)
{
    return I2c.write(dev_id, reg_addr, reg_data, len);
}

#ifdef __cplusplus
}
#endif
