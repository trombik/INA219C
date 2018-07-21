#include <stdint.h>
#include <math.h>
#include <assert.h>

#include "TRB_INA219.h"

#if defined(__cplusplus)
extern "C" {
#endif

int32_t
ina219_read16(const struct ina219_dev *ina219_dev, const uint8_t reg, uint16_t *data)
{
	int32_t r;
	uint8_t buffer[2];
	r = ina219_dev->read(ina219_dev->address, reg, buffer, 2);
	*data = (buffer[0] << 8) | buffer[1];
	return r;
}

int32_t
ina219_write16(const struct ina219_dev *dev, const uint8_t reg, const uint16_t *data)
{
	int32_t r;
	uint8_t buffer[2];
	buffer[0] = (uint8_t)(*data >> 8);
	buffer[1] = (uint8_t)(*data & 0xff);
	r = dev->write(dev->address, reg, buffer, 2);
	return r;
}

int32_t
ina219_reset(struct ina219_dev *dev)
{
	int32_t r;
	r = ina219_set_bits(dev, INA219_REG_CONFIG, INA219_REG_CONFIG_MASK_RESET, 1);
	return r;
}

int32_t
ina219_get_mode(const struct ina219_dev *dev, ina219_mode_t *mode)
{
	int32_t r;
	uint16_t value;
	r = ina219_get_bits(dev, INA219_REG_CONFIG, INA219_REG_CONFIG_MASK_MODE, &value);
	if (r != 0)
		return r;
	*mode = (ina219_mode_t)value;
	return 0;
}


int32_t
ina219_get_bits(const struct ina219_dev *dev, const uint8_t reg, const uint16_t mask, uint16_t *value)
{
	int32_t r;
	uint16_t reg_value;
	r = ina219_read16(dev, reg, &reg_value);
	if (r != 0)
		return r;
	*value = (reg_value & mask) >> ina219_get_bits_from_mask(mask);
	return r;
}

uint8_t
ina219_get_bits_from_mask(const uint16_t mask)
{
	uint16_t bits;

	bits = 0;
	while (((mask >> bits) & 0x01) == 0x00) {
		bits++;
		assert(bits < 16);
	}
	return bits;
}

int32_t
ina219_set_bits(const struct ina219_dev *dev, const uint8_t reg, const uint16_t mask, const uint16_t value)
{
	int32_t r;
	uint16_t reg_value;

	r = ina219_read16(dev, reg, &reg_value);
	if (r != 0)
		return r;
	reg_value = (reg_value & ~mask) | (value << ina219_get_bits_from_mask(mask) );
	r = ina219_write16(dev, reg, &reg_value);
	return r;
}

int32_t
ina219_get_v_bus_range(const struct ina219_dev *dev, ina219_range_t *range)
{
	int32_t r;
	uint16_t reg_value;
	r = ina219_get_bits(dev, INA219_REG_CONFIG, INA219_REG_CONFIG_MASK_BRNG, &reg_value);
	if (r != 0)
		return r;
	*range = (ina219_range_t)reg_value;
	return r;
}

float
__ina219_get_shunt_max_v(struct ina219_dev *dev)
{
	switch (dev->gain) {
	case INA219_PGA_GAIN_40MV:
		return 0.04;
	case INA219_PGA_GAIN_80MV:
		return 0.08;
	case INA219_PGA_GAIN_160MV:
		return 0.16;
	case INA219_PGA_GAIN_320MV:
		return 0.32;
	default:
	/* should not be reached */
		assert(1 == 0);
	}
	return 0;
}

int32_t
ina219_calc_calibration(struct ina219_dev *dev)
{
	float minimumLSB;
	float current_lsb;

	assert(dev->i_max_expected > 0);
	assert(dev->r_shunt > 0);

	minimumLSB = dev->i_max_expected / 32767;
	current_lsb = (uint16_t)(minimumLSB * 100000000) + 1;
	current_lsb /= 100000000;

	assert(current_lsb != 0);

	dev->current_lsb = current_lsb;
	dev->cal = (uint16_t)((0.04096) / (current_lsb * dev->r_shunt));
	dev->power_lsb = current_lsb * 20;
	dev->i_max_possible = __ina219_get_shunt_max_v(dev) / dev->r_shunt;
	return 0;
}

int32_t
ina219_set_calibration(struct ina219_dev *dev)
{
	int32_t r;
	r = ina219_calc_calibration(dev);
	if (r != 0)
		return r;
	r = ina219_write16(dev, INA219_REG_CALIBRATION, &(dev->cal));
	return r;
}

int32_t
ina219_get_calibration(struct ina219_dev *dev, uint16_t *value)
{
	int32_t r;
	r = ina219_read16(dev, INA219_REG_CALIBRATION, &(dev->cal));
	return r;
}

int32_t
ina219_get_pga_gain(const struct ina219_dev *dev, ina219_pga_gain_t *gain)
{
	int32_t r;
	uint16_t value;
	r = ina219_get_bits(dev, INA219_REG_CONFIG, INA219_REG_CONFIG_MASK_PG, &value);
	if (r != 0)
		return r;
	*gain = (ina219_pga_gain_t)value;
	return r;
}

int32_t
ina219_get_sadc_value(const struct ina219_dev *dev, ina219_resolution_t *res)
{
	int32_t r;
	uint16_t value;
	r = ina219_get_bits(dev, INA219_REG_CONFIG, INA219_REG_CONFIG_MASK_SADC, &value);
	if (r != 0)
		return r;
	*res = (ina219_resolution_t)value;
	return r;
}

int32_t
ina219_get_badc_value(const struct ina219_dev *dev, ina219_resolution_t *res)
{
	int32_t r;
	uint16_t value;
	r = ina219_get_bits(dev, INA219_REG_CONFIG, INA219_REG_CONFIG_MASK_BADC, &value);
	if (r != 0)
		return r;
	*res = (ina219_resolution_t)value;
	return r;
}

int16_t
ina219_decomplement(uint16_t v, uint8_t sign_bits)
{
	v = (v << sign_bits) >> sign_bits;
	v -= 1;
	v = ~v;
	v *= -1;
	return v;
}

int32_t
ina219_conversion_is_ready(const struct ina219_dev *dev)
{
	int32_t r;
	uint16_t reg_value;
	r = ina219_get_bits(dev, INA219_REG_BUS, INA219_REG_BUS_MASK_CNVR, &reg_value);
	if (r != 0)
		return -1;
	return reg_value == 1 ? INA219_CONVERSION_IS_READY : INA219_CONVERSION_IS_NOT_READY;
}

int32_t
ina219_conversion_is_overflowed(const struct ina219_dev *dev)
{
	int32_t r;
	uint16_t reg_value;
	r = ina219_read16(dev, INA219_REG_BUS, &reg_value);
	if (r != 0)
		return -1;
	return (reg_value & 0x01) ? INA219_IS_OVERFLOWED : INA219_IS_NOT_OVERFLOWED;
}

int32_t
ina219_get_v_bus(const struct ina219_dev *dev, float *voltage)
{
	int32_t r;
	uint16_t v_bus_raw;
	r = ina219_read16(dev, INA219_REG_BUS, &v_bus_raw);
	if (r != 0)
		return r;
	*voltage = (float)(v_bus_raw >> 3) * 0.004 ; // in V (LSB 4mV)
	return r;
}

int32_t
ina219_get_p_bus(const struct ina219_dev *dev, float *p_bus)
{
	int32_t r;
	uint16_t p_bus_raw;
	r = ina219_read16(dev, INA219_REG_POWER, &p_bus_raw);
	if (r != 0)
		return r;
	*p_bus = (float)(dev->power_lsb * p_bus_raw);
	return r;
}

int32_t
ina219_get_i_bus(const struct ina219_dev *dev, float *i_bus)
{
	int32_t r;
	uint16_t i_bus_raw;
	r = ina219_read16(dev, INA219_REG_CURRENT, &i_bus_raw);
	if (r != 0)
		return r;
	if ((i_bus_raw >> 15 ) == 1) {
		*i_bus = ina219_decomplement(i_bus_raw, 1) * dev->current_lsb;
	} else {
		*i_bus = i_bus_raw * dev->current_lsb; // in A
	}
	return r;
}

int32_t
ina219_get_v_shunt(struct ina219_dev *dev, float *v_shunt)
{
	int32_t r;
	uint8_t sign_bits = 0;
	uint16_t v_r_shuntaw;

	r = ina219_read16(dev, INA219_REG_SHUNT, &v_r_shuntaw);
	if (r != 0)
		return r;

	switch (dev->gain) {
	case INA219_PGA_GAIN_320MV:
		sign_bits = 1;
		break;
	case INA219_PGA_GAIN_160MV:
		sign_bits = 2;
		break;
	case INA219_PGA_GAIN_80MV:
		sign_bits = 3;
		break;
	case INA219_PGA_GAIN_40MV:
		sign_bits = 4;
		break;
	}
	assert(sign_bits != 0);
	*v_shunt = ((v_r_shuntaw >> 15) == 1)
	    ? (float)ina219_decomplement(v_r_shuntaw, sign_bits) / 100000.0
	    : (float)v_r_shuntaw / 100000.0; // in V (LSB 10uV)
	return r;
}

int32_t
ina219_get_sensor_values(struct ina219_dev *dev)
{
	int32_t r;

	r = ina219_get_v_shunt(dev, &(dev->v_shunt));
	if (r != 0)
		return r;
	r = ina219_get_v_bus(dev, &(dev->v_bus));
	if (r != 0)
		return r;
	r = ina219_get_p_bus(dev, &(dev->p_bus)); // in W
	if (r != 0)
		return r;
	r = ina219_get_i_bus(dev, &(dev->i_bus));
	if (r != 0)
		return r;
	return 0;
}

struct ina219_dev
ina219_create(const uint8_t addr)
{
	struct ina219_dev dev;
	dev.address = addr;
	dev.read = ina219_read;
	dev.write = ina219_write;
	dev.i_max_expected = 3.0;
	dev.r_shunt = 0.1;

	dev.range = INA219_BUS_VOLTAGE_RANGE_32V;
	dev.gain = INA219_PGA_GAIN_320MV;
	dev.bus_adc_resolution = INA219_RESOLUTION_12BIT_1;
	dev.shunt_adc_resolution = INA219_RESOLUTION_12BIT_1;
	dev.mode = INA219_MODE_SHUNT_BUS_CONTINUOUS;

	ina219_calc_calibration(&dev);
	return dev;
}

int32_t
ina219_configure(struct ina219_dev *dev)
{
	int32_t r;
	uint16_t reg_value;

	reg_value =
	    (dev->range << ina219_get_bits_from_mask(INA219_REG_CONFIG_MASK_BRNG)) |
	    (dev->gain << ina219_get_bits_from_mask(INA219_REG_CONFIG_MASK_PG)) |
	    (dev->bus_adc_resolution << ina219_get_bits_from_mask(INA219_REG_CONFIG_MASK_BADC)) |
	    (dev->shunt_adc_resolution << ina219_get_bits_from_mask(INA219_REG_CONFIG_MASK_SADC)) |
	    (dev->mode << ina219_get_bits_from_mask(INA219_REG_CONFIG_MASK_MODE));
	r = ina219_write16(dev, INA219_REG_CONFIG, &reg_value);
	ina219_read16(dev, INA219_REG_CONFIG, &reg_value);
        return r;
}

#if defined(__cplusplus)
}
#endif
