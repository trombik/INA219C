#if defined(INA219_I2C_ARDUINO_WIRE)

#if defined(ARDUINO_ARCH_AVR)
#define WITH_STOP 1
#define WITHOUT_STOP 0
#else
#define WITH_STOP true
#define WITHOUT_STOP false
#endif

#define WITH_REPEATED_START false
#define WITHOUT_REPEATED_START true

#include <Arduino.h>
#include <Wire.h>
#include "../../INA219C.h"

void
ina219c_delay_ms(const uint32_t period)
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

int8_t
ina219c_read(const uint8_t dev_id, const uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
	int8_t result = 0;

	/* start transaction */
	Wire.beginTransmission(dev_id);
	/* write register address */
	Wire.write(reg_addr);
	/* read the values with repeated start, and end the transaction with stop
	* bit */
	result = Wire.endTransmission(WITH_REPEATED_START);
	if (result != 0)
		return result;
#if defined(ARDUINO_ARCH_AVR)
	if (Wire.requestFrom(dev_id, (uint8_t)len, (uint8_t)WITH_STOP) != len)
#else
	if (Wire.requestFrom(dev_id, (uint8_t)len, WITH_STOP) != len)
#endif
		return -1;
	for (uint16_t i = 0; i <= len - 1; i++) {
		reg_data[i] = Wire.read();
	}
	return result;
}

int8_t
ina219c_write(const uint8_t dev_id, const uint8_t reg_addr, uint8_t *reg_data, const uint8_t len)
{
	int8_t result = 0;

	/* start transaction */
	Wire.beginTransmission(dev_id);
	/* write register address */
	Wire.write(reg_addr);
	/* write values */
	Wire.write(reg_data, len);
	/* end end transaction */
	result = Wire.endTransmission();
	return result;
}

#endif
