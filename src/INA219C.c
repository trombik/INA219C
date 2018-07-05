#include "INA219C.h"

#include <math.h>
#include <assert.h>

#if defined(INA219_I2C_ESP_IDF)
#include "sys/esp_idf/i2c.c"
#endif

struct ina219c_dev
ina219c_create(const uint8_t addr)
{
	struct ina219c_dev dev;
	dev.address = addr;
	dev.read = ina219c_read;
	dev.write = ina219c_write;
	dev.max_expected_i = 3.0;
	dev.shunt_r = 0.1;
	return dev;
}

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
ina219c_write16(const struct ina219c_dev *dev, const uint8_t reg, uint16_t *data)
{
	int8_t r = 0;
	uint8_t buffer[2];
	buffer[0] = *data >> 8;
	buffer[1] = *data & 0x0f;
	r = dev->write(dev->address, reg, buffer, 2);
	*data = (buffer[0] << 8) | buffer[1];
	return r;
}

int8_t
ina219c_reset(struct ina219c_dev *dev)
{
	int8_t r = 0;
	uint16_t value = 1 << INA219C_REG_CONfIG_BIT_RESET;
	r = ina219c_write16(dev, INA219C_REG_CONFIG, &value);
	return r;
}

int8_t
ina219c_set_mode(const struct ina219c_dev *dev, const ina219c_mode mode)
{
	int8_t r;
	r = ina219c_set_bits(dev, INA219C_REG_CONFIG, INA219C_REG_CONfIG_MASK_MODE, mode);
	return r;
}

int8_t
ina219c_get_mode(const struct ina219c_dev *dev, ina219c_mode *mode)
{
	int8_t r;
	uint16_t value;
	r = ina219c_read16(dev, INA219C_REG_CONFIG, &value);
	if (r != 0)
		return r;
	*mode = (value & INA219C_REG_CONfIG_MASK_MODE);
	return 0;
}


int8_t
ina219c_set_bits(const struct ina219c_dev *dev, const uint8_t reg, const uint16_t mask, const uint16_t value)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_read16(dev, reg, &reg_value);
	if (r != 0)
		return r;
	reg_value = (reg_value & ~mask) | value;
	r = ina219c_write16(dev, reg, &reg_value);
	return r;
}

int8_t
ina219c_get_bus_voltage_range(const struct ina219c_dev *dev, ina219c_range *range)
{
	uint8_t r;
	uint16_t value;
	r = ina219c_read16(dev, INA219C_REG_CONFIG, &value);
	if (r != 0)
		return r;
	value &= INA219C_REG_CONfIG_MASK_BRNG >> INA219C_REG_CONfIG_BIT_BRNG;
	*range = (ina219c_range)value;
	return r;
}

int8_t
ina219c_set_bus_voltage_range(const struct ina219c_dev *dev, const ina219c_range range)
{
	int8_t r;
	uint16_t value;
	r = ina219c_set_bits(dev, INA219C_REG_CONFIG, INA219C_REG_CONfIG_BIT_BRNG, range);
	if (r != 0)
		return r;
	r = ina219c_read16(dev, INA219C_REG_CONFIG, &value);
	if (r != 0)
		return r;
	value &= INA219C_REG_CONfIG_MASK_BRNG >> INA219C_REG_CONfIG_BIT_BRNG;
	if ((ina219c_range)value != range)
		return -1;
	return 0;
}

float
ina219c_get_currrent_lsb(const struct ina219c_dev *dev)
{

	float minimum_lsb, current_lsb;
	minimum_lsb = dev->max_expected_i / 32768;
	current_lsb = (uint16_t)(minimum_lsb * 100000000);
	current_lsb /= 100000000;
	current_lsb /= 0.0001;
	current_lsb = ceil(current_lsb);
	current_lsb *= 0.0001;
	return current_lsb;
}

int8_t
ina219c_set_calibration(const struct ina219c_dev *dev)
{
	int8_t r;
	uint16_t calibration_value;
	float current_lsb;
	assert(dev->max_expected_i && dev->max_expected_i > 0);
	assert(dev->shunt_r && dev->shunt_r > 0);

	current_lsb = ina219c_get_currrent_lsb(dev);
	calibration_value = 0.04096 / (current_lsb * dev->shunt_r);
	r = ina219c_write16(dev, INA219C_REG_CALIBRATION, &calibration_value);
	return r;
}

int8_t
ina219c_get_calibration(const struct ina219c_dev *dev, uint16_t *value)
{
	int8_t r;
	uint16_t calibration_value;
	r = ina219c_read16(dev, INA219C_REG_CALIBRATION, &calibration_value);
	return r;
}

int8_t
ina219c_get_pga_gain(const struct ina219c_dev *dev, ina219c_pga_gain_t *gain)
{
	int8_t r;
	uint16_t value;
	r = ina219c_read16(dev, INA219C_REG_CONFIG, &value);
	if (r != 0)
		return r;
	*gain = (value & INA219C_REG_CONfIG_MASK_PG) >> INA219C_REG_CONfIG_BIT_PG;
	return r;
}

int8_t
ina219c_set_pga_gain(const struct ina219c_dev *dev, const ina219c_pga_gain_t gain)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_set_bits(dev, INA219C_REG_CONFIG, INA219C_REG_CONfIG_MASK_PG, gain);
	if (r != 0)
		return r;
	r = ina219c_read16(dev, INA219C_REG_CONFIG, &reg_value);
	if (r != 0)
		return r;
	reg_value &= INA219C_REG_CONfIG_MASK_PG >> INA219C_REG_CONfIG_BIT_PG;
	if (reg_value != gain)
		return -1;
	return 0;
}

int8_t
ina219c_get_sadc_value(const struct ina219c_dev *dev, ina219_resolution_t *res)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_read16(dev, INA219C_REG_CONFIG, &reg_value);
	if (r != 0)
		return r;
	*res = (reg_value & INA219C_REG_CONfIG_MASK_SDAC) >> INA219C_REG_CONfIG_BIT_SADC;
	return r;
}

int8_t
ina219c_get_badc_value(const struct ina219c_dev *dev, ina219_resolution_t *res)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_read16(dev, INA219C_REG_CONFIG, &reg_value);
	if (r != 0)
		return r;
	*res = (reg_value & INA219C_REG_CONfIG_MASK_BADC) >> INA219C_REG_CONfIG_BIT_BADC;
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
ina219c_get_shunt_voltage(const struct ina219c_dev *dev, float *milliv)
{
	int8_t r;
	uint8_t sign_bits = 0;
	uint16_t reg_value;
	ina219c_pga_gain_t gain;

	r = ina219c_get_pga_gain(dev, &gain);
	if (r != 0)
		return r;
	r = ina219c_read16(dev, INA219C_REG_SHUNT, &reg_value);
	if (r != 0)
		return r;
	switch (gain) {
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
	*milliv = ((reg_value >> 15) == 1)
	    ?  ina219c_decomplement(reg_value, sign_bits) / 100
	    : reg_value / 100;
	return r;
}

int8_t
ina219c_conversion_is_ready(const struct ina219c_dev *dev)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_read16(dev, INA219C_REG_BUS, &reg_value);
	if (r != 0)
		return -1;
	return (reg_value >> 1) & 0x01;
}

int8_t
ina219c_conversion_is_overflowed(const struct ina219c_dev *dev)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_read16(dev, INA219C_REG_BUS, &reg_value);
	if (r != 0)
		return -1;
	return reg_value & 0x01;
}

int8_t
ina219c_get_bus_voltage(const struct ina219c_dev *dev, float *voltage)
{
	int8_t r;
	uint16_t reg_value;
	uint16_t max = 0;
	ina219c_range range;
	r = ina219c_read16(dev, INA219C_REG_BUS, &reg_value);
	if (r != 0)
		return r;
	r = ina219c_get_bus_voltage_range(dev, &range);
	if (r != 0)
		return r;
	switch (range) {
	case INA219C_BUS_VOLTAGE_RANGE_16V:
		max = 0x0FA0;
		break;
	case INA219C_BUS_VOLTAGE_RANGE_32V:
		max = 0x1F40;
		break;
	}
	assert(max != 0);
	*voltage = (reg_value >> 3) / max;
	return r;
}

int8_t
ina219c_get_power(const struct ina219c_dev *dev, float *power)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_read16(dev, INA219C_REG_POWER, &reg_value);
	if (r != 0)
		return r;
	*power = (float)(ina219c_get_currrent_lsb(dev) * 20 * reg_value);
	return r;
}

int8_t
ina219c_get_current(const struct ina219c_dev *dev, float *current)
{
	int8_t r;
	uint16_t reg_value;
	r = ina219c_read16(dev, INA219C_REG_CURRENT, &reg_value);
	if (r != 0)
		return r;
	*current = (float)(ina219c_get_currrent_lsb(dev) * reg_value);
	return r;
}
