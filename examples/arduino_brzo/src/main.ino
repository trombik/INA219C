#define STRETCHING_TIMEOUT_MILLI_SEC 1000
#define I2C_ADDRESS 0x40

#include <INA219C.h>
#include <Arduino.h>
#include <brzo_i2c.h>

struct ina219c_dev dev;

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
setup()
{
	Serial.begin(115200);
	brzo_i2c_setup(GPIO_SDA, GPIO_SCL, STRETCHING_TIMEOUT_MILLI_SEC);
	dev = ina219c_create(I2C_ADDRESS);
	ina219c_configure(&dev);
	ina219c_set_calibration(&dev);
}

void
loop()
{
	ina219c_get_sensor_values(&dev);
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
	ina219c_delay_ms(1000);
}
