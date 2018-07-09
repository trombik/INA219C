#include "INA219C.h"

#include <math.h>
#include <assert.h>

#if defined(INA219_I2C_ESP_IDF)
#include "sys/esp_idf/i2c.c"
#endif

int8_t
ina219c_read16(const struct ina219c_dev *ina219_dev, const uint8_t reg, uint16_t *data)
{
	int8_t r = 0;
	uint8_t buffer[2];
	ina219_dev->read(ina219_dev->address, reg, buffer, 2);
	*data = (buffer[0] << 8) | buffer[1];
	return r;
}

int8_t
ina219c_write16(const struct ina219c_dev *dev, const uint8_t reg, const uint16_t *data)
{
	int8_t r = 0;
	uint8_t buffer[2];
	buffer[0] = (uint8_t)(*data >> 8);
	buffer[1] = (uint8_t)(*data & 0xff);
	r = dev->write(dev->address, reg, buffer, 2);
	return r;
}

int8_t
ina219c_reset(struct ina219c_dev *dev)
{
	int8_t r = 0;
	r = ina219c_set_bits(dev, INA219C_REG_CONFIG, INA219C_REG_CONFIG_MASK_RESET, 1);
	return r;
}

int8_t
ina219c_get_mode(const struct ina219c_dev *dev, ina219c_mode *mode)
{
	int8_t r;
	uint16_t value;
	r = ina219c_get_bits(dev, INA219C_REG_CONFIG, INA219C_REG_CONFIG_MASK_MODE, &value);
	if (r != 0)
		return r;
	*mode = (ina219c_mode)value;
	return 0;
}


int8_t
ina219c_get_bits(const struct ina219c_dev *dev, const uint8_t reg, const uint16_t mask, uint16_t *value)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_read16(dev, reg, &reg_value);
	if (r != 0)
		return r;
	*value = (reg_value & mask) >> ina219c_get_bits_from_mask(mask);
	return r;
}

uint8_t
ina219c_get_bits_from_mask(const uint16_t mask)
{
	uint16_t bits;

	bits = 0;
	while (((mask >> bits) & 0x01) == 0x00) {
		bits++;
		assert(bits < 16);
	}
	return bits;
}

int8_t
ina219c_set_bits(const struct ina219c_dev *dev, const uint8_t reg, const uint16_t mask, const uint16_t value)
{
	int8_t r;
	uint16_t reg_value;

	r = ina219c_read16(dev, reg, &reg_value);
	if (r != 0)
		return r;
	reg_value = (reg_value & ~mask) | (value << ina219c_get_bits_from_mask(mask) );
	r = ina219c_write16(dev, reg, &reg_value);
	return r;
}

int8_t
ina219c_get_bus_voltage_range(const struct ina219c_dev *dev, ina219c_range *range)
{
	uint8_t r;
	uint16_t reg_value;
	r = ina219c_get_bits(dev, INA219C_REG_CONFIG, INA219C_REG_CONFIG_MASK_BRNG, &reg_value);
	if (r != 0)
		return r;
	*range = reg_value;
	return r;
}

float
__ina219c_get_shunt_max_v(struct ina219c_dev *dev)
{
	switch (dev->gain) {
	case INA219C_PGA_GAIN_40MV:
		return 0.04;
	case INA219C_PGA_GAIN_80MV:
		return 0.08;
	case INA219C_PGA_GAIN_160MV:
		return 0.16;
	case INA219C_PGA_GAIN_320MV:
		return 0.32;
	default:
	/* should not be reached */
		assert(1 == 0);
	}
	return 0;
}

int8_t
ina219c_calc_calibration(struct ina219c_dev *dev)
{
	float minimumLSB;
	float currentlsb;

	assert(dev->max_expected_i > 0);
	assert(dev->shunt_r > 0);

	minimumLSB = dev->max_expected_i / 32767;
	currentlsb = (uint16_t)(minimumLSB * 100000000) + 1;
	currentlsb /= 100000000;

	assert(currentlsb != 0);

	dev->current_lsb = currentlsb;
	dev->cal = (uint16_t)((0.04096) / (currentlsb * dev->shunt_r));
	dev->power_lsb = currentlsb * 20;
	dev->max_possible_i = __ina219c_get_shunt_max_v(dev) / dev->shunt_r;
	return 0;
}

int8_t
ina219c_set_calibration(struct ina219c_dev *dev)
{
	int8_t r;
	r = ina219c_calc_calibration(dev);
	if (r != 0)
		return r;
	r = ina219c_write16(dev, INA219C_REG_CALIBRATION, &(dev->cal));
	return r;
}

int8_t
ina219c_get_calibration(struct ina219c_dev *dev, uint16_t *value)
{
	int8_t r;
	r = ina219c_read16(dev, INA219C_REG_CALIBRATION, &(dev->cal));
	return r;
}

int8_t
ina219c_get_pga_gain(const struct ina219c_dev *dev, ina219c_pga_gain_t *gain)
{
	int8_t r;
	uint16_t value;
	r = ina219c_get_bits(dev, INA219C_REG_CONFIG, INA219C_REG_CONFIG_MASK_PG, &value);
	if (r != 0)
		return r;
	*gain = value;
	return r;
}

int8_t
ina219c_get_sadc_value(const struct ina219c_dev *dev, ina219_resolution_t *res)
{
	int8_t r;
	uint16_t value;
	r = ina219c_get_bits(dev, INA219C_REG_CONFIG, INA219C_REG_CONFIG_MASK_SADC, &value);
	if (r != 0)
		return r;
	*res = (ina219_resolution_t)value;
	return r;
}

int8_t
ina219c_get_badc_value(const struct ina219c_dev *dev, ina219_resolution_t *res)
{
	int8_t r;
	uint16_t value;
	r = ina219c_get_bits(dev, INA219C_REG_CONFIG, INA219C_REG_CONFIG_MASK_BADC, &value);
	if (r != 0)
		return r;
	*res = (ina219_resolution_t)value;
	return r;
}

int16_t
ina219c_decomplement(uint16_t v, uint8_t sign_bits)
{
	v = (v << sign_bits) >> sign_bits;
	v -= 1;
	v = ~v;
	v *= -1;
	return v;
}

int8_t
ina219c_conversion_is_ready(const struct ina219c_dev *dev)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_get_bits(dev, INA219C_REG_BUS, INA219C_REG_BUS_MASK_CNVR, &reg_value);
	if (r != 0)
		return -1;
	return reg_value == 1 ? INA219C_CONVERSION_IS_READY : INA219C_CONVERSION_IS_NOT_READY;
}

int8_t
ina219c_conversion_is_overflowed(const struct ina219c_dev *dev)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_read16(dev, INA219C_REG_BUS, &reg_value);
	if (r != 0)
		return -1;
	return (reg_value & 0x01) ? INA219C_IS_OVERFLOWED : INA219C_IS_NOT_OVERFLOWED;
}

int8_t
ina219c_get_bus_voltage(const struct ina219c_dev *dev, float *voltage)
{
	int8_t r;
	uint16_t bus_voltage_raw;
	r = ina219c_read16(dev, INA219C_REG_BUS, &bus_voltage_raw);
	if (r != 0)
		return r;
	*voltage = (float)(bus_voltage_raw >> 3) * 0.004 ; // in V (LSB 4mV)
	return r;
}

int8_t
ina219c_get_power(const struct ina219c_dev *dev, float *power)
{
	int8_t r;
	uint16_t power_raw;
	r = ina219c_read16(dev, INA219C_REG_POWER, &power_raw);
	if (r != 0)
		return r;
	*power = (float)(dev->power_lsb * power_raw);
	return r;
}

int8_t
ina219c_get_current_register(const struct ina219c_dev *dev, uint16_t *reg_value)
{
	return ina219c_read16(dev, INA219C_REG_CURRENT, reg_value);
}

int8_t
ina219c_get_current(const struct ina219c_dev *dev, float *current)
{
	int8_t r;
	uint16_t current_raw;
	r = ina219c_get_current_register(dev, &current_raw);
	if (r != 0)
		return r;
	if ((current_raw >> 15 ) == 1) {
		*current = ina219c_decomplement(current_raw, 1) * dev->current_lsb;
	} else {
		*current = current_raw * dev->current_lsb; // in A
	}
	return r;
}

int8_t
ina219c_get_shunt_voltage(struct ina219c_dev *dev, float *shunt_voltage)
{
	int8_t r;
	uint8_t sign_bits = 0;
	uint16_t shunt_voltage_raw;

	r = ina219c_read16(dev, INA219C_REG_SHUNT, &shunt_voltage_raw);
	if (r != 0)
		return r;

	switch (dev->gain) {
	case INA219C_PGA_GAIN_320MV:
		sign_bits = 1;
		break;
	case INA219C_PGA_GAIN_160MV:
		sign_bits = 2;
		break;
	case INA219C_PGA_GAIN_80MV:
		sign_bits = 3;
		break;
	case INA219C_PGA_GAIN_40MV:
		sign_bits = 4;
		break;
	}
	assert(sign_bits != 0);
	*shunt_voltage = ((shunt_voltage_raw >> 15) == 1)
	    ? (float)ina219c_decomplement(shunt_voltage_raw, sign_bits) / 100000.0
	    : (float)shunt_voltage_raw / 100000.0; // in V (LSB 10uV)
	return r;
}

int8_t
ina219c_get_sensor_values(struct ina219c_dev *dev)
{
	int8_t r;

	r = ina219c_get_shunt_voltage(dev, &(dev->shunt_voltage));
	if (r != 0)
		return r;
	r = ina219c_get_bus_voltage(dev, &(dev->bus_voltage));
	if (r != 0)
		return r;
	r = ina219c_get_power(dev, &(dev->power)); // in W
	if (r != 0)
		return r;
	r = ina219c_get_current(dev, &(dev->current));
	if (r != 0)
		return r;
	return 0;
}

struct ina219c_dev
ina219c_create(const uint8_t addr)
{
	struct ina219c_dev dev;
	dev.address = addr;
	dev.read = ina219c_read;
	dev.write = ina219c_write;
	dev.max_expected_i = 3.0;
	dev.shunt_r = 0.1;

	dev.range = INA219C_BUS_VOLTAGE_RANGE_32V;
	dev.gain = INA219C_PGA_GAIN_320MV;
	dev.bus_adc_resolution = INA219C_RESOLUTION_12BIT_1;
	dev.shunt_adc_resolution = INA219C_RESOLUTION_12BIT_1;
	dev.mode = INA219C_MODE_SHUNT_BUS_CONTINUOUS;

	ina219c_calc_calibration(&dev);
	return dev;
}

int8_t
ina219c_configure(struct ina219c_dev *dev)
{
	int8_t r;
	uint16_t reg_value;

	reg_value =
	    (dev->range << ina219c_get_bits_from_mask(INA219C_REG_CONFIG_MASK_BRNG)) |
	    (dev->gain << ina219c_get_bits_from_mask(INA219C_REG_CONFIG_MASK_PG)) |
	    (dev->bus_adc_resolution << ina219c_get_bits_from_mask(INA219C_REG_CONFIG_MASK_BADC)) |
	    (dev->shunt_adc_resolution << ina219c_get_bits_from_mask(INA219C_REG_CONFIG_MASK_SADC)) |
	    (dev->mode << ina219c_get_bits_from_mask(INA219C_REG_CONFIG_MASK_MODE));
	r = ina219c_write16(dev, INA219C_REG_CONFIG, &reg_value);
	ina219c_read16(dev, INA219C_REG_CONFIG, &reg_value);
        return r;
}
