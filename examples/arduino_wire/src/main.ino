#include <Arduino.h>

#if defined(TRB_INA219_I2C_WIRE)
#include <Wire.h>
#elif defined(TRB_INA219_I2C_LIB_I2C)
#include <I2C.h>
#endif

#include <TRB_INA219.h>

struct ina219_dev dev;

char *
float2string(float f)
{
	static char buffer[10];
	if (f >= 100000 || f <= -100000) {
		strlcpy(buffer, "-XXXXX.XX", sizeof(buffer));
	} else {
		dtostrf(f, 9, 2, buffer);
	}
	return buffer;
}

void
halt()
{
	while(1) {
#if defined(ESP32) || defined(ESP8266)
		yield();
#endif
	}
}
void
setup()
{
	Serial.begin(115200);
#if defined(TRB_INA219_I2C_WIRE)
#if defined(ESP8266) || defined(ESP32)
	Serial.println(F("initializing I2C (Wire for ESPs)"));
	Wire.begin(GPIO_SDA, GPIO_SCL);
#else
	Serial.println(F("initializing I2C (Wire)"));
	Wire.begin();
#endif
#elif defined(TRB_INA219_I2C_LIB_I2C)
	Serial.println(F("initializing I2C (I2c)"));
	I2c.begin();
#endif // defined(TRB_INA219_I2C_WIRE)
	dev = ina219_create(0x40);
	if (ina219_reset(&dev) != 0) {
		Serial.println(F("falied to ina219_reset()"));
		halt();
	}
	dev.gain = INA219_PGA_GAIN_40MV;
	if (ina219_configure(&dev) != 0) {
		Serial.println(F("failed to ina219_configure()"));
		halt();
	}
	if (ina219_set_calibration(&dev) != 0) {
		Serial.println(F("failed to ina219_set_calibration()"));
		halt();
	}
	for (uint8_t i = 0; i <= 100; i++) {
		if (i == 100) {
			Serial.println(F("Timeout while ina219_conversion_is_ready()"));
			halt();
		}
		if (ina219_conversion_is_ready(&dev) == INA219_CONVERSION_IS_READY) {
			break;
		}
	}
}

void
loop()
{
	if (ina219_get_sensor_values(&dev) != 0) {
		Serial.println(F("failed to ina219_get_sensor_values()"));
		goto fail;
	}

	Serial.print(F("bus_voltage:   "));
	Serial.print(float2string(dev.bus_voltage));
	Serial.println(F("V"));
	Serial.print(F("shunt_voltage: "));
	Serial.print(float2string(dev.shunt_voltage * 1000));
	Serial.println(F("mV"));
	Serial.print(F("power:         "));
	Serial.print(float2string(dev.power * 1000));
	Serial.println(F("mW"));
	Serial.print(F("current:       "));
	Serial.print(float2string(dev.current * 1000));
	Serial.println(F("mA"));
fail:
	ina219_delay_ms(1000);
}
