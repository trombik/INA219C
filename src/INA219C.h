#if !defined(_INA219C_h)
#define _INA219C_h

#include <stdint.h>

/* read and write functions that must be implemented.
 * addr I2C address
 * reg_addr register address to read or write
 * data array of data to write or read
 * len the length of data
 */
typedef int8_t (*ina219c_fptr_t) (const uint8_t addr, const uint8_t reg_addr, uint8_t *data, uint8_t len);

struct ina219c_dev {
	/*! I2C address */
	uint8_t address;
	/* framework-dependant function pointers */
	ina219c_fptr_t read;
	ina219c_fptr_t write;
	/* maximum expected current in Amps */
	float max_expected_i;
	/* Shunt register value in Ohms */
	float shunt_r;
};

void
ina219c_delay_ms(const uint32_t period);

/* default I2C address */
#define INA219C_ADDRESS		(0x40)

/* register addresses */
#define INA219C_REG_CONFIG	(0x00)
#define INA219C_REG_SHUNT	(0x01)
#define INA219C_REG_BUS		(0x02)
#define INA219C_REG_POWER	(0x03)
#define INA219C_REG_CURRENT	(0x04)
#define INA219C_REG_CALIBRATION	(0x05)

/* INA219C_REG_CONFIG */

/* bit positions in INA219C_REG_CONFIG */
#define INA219C_REG_CONfIG_BIT_MODE	(0)
#define INA219C_REG_CONfIG_BIT_SADC	(3)
#define INA219C_REG_CONfIG_BIT_BADC	(7)
#define INA219C_REG_CONfIG_BIT_PG	(11)
#define INA219C_REG_CONfIG_BIT_BRNG	(13)
#define INA219C_REG_CONfIG_BIT_RESET	(15)

/* masks for configuration bits */
#define INA219C_REG_CONfIG_MASK_MODE	(1 << 0 | 1 << 1 | 1 << 2)
#define INA219C_REG_CONfIG_MASK_SDAC	(1 << 3 | 1 << 4 | 1 << 5 | 1 << 6)
#define INA219C_REG_CONfIG_MASK_BADC	(1 << 7 | 1 << 8 | 1 << 9 | 1 << 10)
#define INA219C_REG_CONfIG_MASK_PG	(1 << 11 | 1 << 12)
#define INA219C_REG_CONfIG_MASK_BRNG	(1 << 13)
#define INA219C_REG_CONfIG_MASK_RESET	(1 << 15)

/* default value of INA219C_REG_CONFIG */
#define INA219C_REG_CONFIG_DEFAULT	(0x399F)

typedef enum
{
	INA219C_BUS_VOLTAGE_RANGE_16V	= 0b00,
	INA219C_BUS_VOLTAGE_RANGE_32V	= 0b01 /* default */
} ina219c_range;

typedef enum
{
	INA219C_PGA_GAIN_40MV	= 0b00,
	INA219C_PGA_GAIN_80MV	= 0b01,
	INA219C_PGA_GAIN_160MV	= 0b10,
	INA219C_PGA_GAIN_320MV	= 0b11 /* default */
} ina219c_pga_gain_t;

typedef enum
{
	INA219C_RESOLUTION_9BIT_1	= 0b0000,
	INA219C_RESOLUTION_10BIT_1	= 0b0001,
	INA219C_RESOLUTION_11BIT_1	= 0b0010,
	INA219C_RESOLUTION_12BIT_1	= 0b0011,
	INA219C_RESOLUTION_12BIT_2	= 0b1001,
	INA219C_RESOLUTION_12BIT_4	= 0b1010,
	INA219C_RESOLUTION_12BIT_8	= 0b1011,
	INA219C_RESOLUTION_12BIT_16	= 0b1100,
	INA219C_RESOLUTION_12BIT_32	= 0b1101,
	INA219C_RESOLUTION_12BIT_64	= 0b1110,
	INA219C_RESOLUTION_12BIT_128	= 0b1111
} ina219_resolution_t;

typedef enum
{
	INA219C_MODE_POWERDOWN			= 0b000,
	INA219C_MODE_SHUNT_TRIGGERED		= 0b001,
	INA219C_MODE_BUS_TRIGGERED		= 0b010,
	INA219C_MODE_SHUNT_BUS_TRIGGERED	= 0b011,
	INA219C_MODE_ADC_OFF			= 0b100,
	INA219C_MODE_SHUNT_CONTINUOUS		= 0b101,
	INA219C_MODE_BUS_CONTINUOUS		= 0b110,
	INA219C_MODE_SHUNT_BUS_CONTINUOUS	= 0b111,
} ina219c_mode;

/*!
 * @brief Create struct ina219c_dev.
 *
 * @param[in] addr : I2C address of INA219
 * @retval srtuct *ina219c_dev
 */
struct ina219c_dev
ina219c_create(const uint8_t addr);

/*!
 * @brief A thin wrapper of a function to read variable length of bytes from a
 * register.
 *
 * @param[in] addr : I2C address
 * @param[in] reg : Register address
 * @param[in] *data : Pointer to the variable to store the values
 * @param[in] len: Length of bytes to read
 */
int8_t
ina219c_read(const uint8_t addr, const uint8_t reg, uint8_t *data, uint8_t len);

/*!
 * @brief A thin wrapper of a function to writes variable length of bytes to a
 * register.
 *
 * @param[in] addr : I2C address
 * @param[in] reg : Register address
 * @param[in] *data : Pointer to the variable that stores the values to write
 * @param[in] len : Length of bytes to write
 */
int8_t
ina219c_write(const uint8_t addr, const uint8_t reg, uint8_t *data, uint8_t len);

/*!
 * @brief Writes two bytes to a register
 */
int8_t
ina219c_write16(const struct ina219c_dev *ina219_dev, const uint8_t reg, uint16_t *data);

/*!
 * @brief Read 2bytes from a regsiter
 *
 * @param[in] *ina219c_dev : A pointer to struct ina219_dev
 * @param[in] reg : Register address
 * @param[in] *data : A pointer to store the value
 */
int8_t
ina219c_read16(const struct ina219c_dev *ina219_dev, const uint8_t reg, uint16_t *data);

/*!
 * @brief Write a byte to a regsiter
 *
 * @param[in] *ina219c_dev : A pointer to struct ina219_dev
 * @param[in] reg : Register address
 * @param[in] *data : A pointer to a variable to store the value
 */
int8_t
ina219c_read8(struct ina219c_dev *ina219_dev, const uint8_t reg, uint8_t *data);

/*!
 * @brief Write one or more bits to the register, verify the result.
 *
 * @param[in] *ina219_dev : A pointer to struct ina219_dev
 * @param[in] reg : Register address
 * @param[in] mask : Shift `value` nth_bit when setting the value
 * @param[in] value : Value to set
 */
int8_t
ina219c_set_bits(const struct ina219c_dev *dev, const uint8_t reg, const uint16_t mask, const uint16_t value);

/*!
 * @brief Reset the device
 */
int8_t
ina219c_reset(struct ina219c_dev *dev);

/*!
 * @brief Set operating mode
 *
 * @param[in] *ina219c_dev : A pointer to struct ina219_dev
 * @param[in] mode: one of ina219c_mode
 */
int8_t
ina219c_set_mode(const struct ina219c_dev *dev, const ina219c_mode mode);

/*!
 * @brief Get operating mode
 * @param[in] *ina219c_dev : A pointer to struct ina219_dev
 * @param[out] *mode : A pointer to a variable to store one of ina219c_mode
 */
int8_t
ina219c_get_mode(const struct ina219c_dev *dev, ina219c_mode *mode);

/*!
 * @brief Get Bus Voltage Range
 * @param[in] *ina219c_dev : A pointer to struct ina219_dev
 * @param[out] range : one of ina219c_range
 */
int8_t
ina219c_get_bus_voltage_range(const struct ina219c_dev *dev, ina219c_range *range);

/*!
 * @brief Set Bus Voltage Range to range
 * @param[in] *ina219c_dev : A pointer to struct ina219_dev
 * @param[in] range : one of ina219c_range
 */
int8_t
ina219c_set_bus_voltage_range(const struct ina219c_dev *dev, const ina219c_range range);

/*!
 * @brief Caribrate the sensor.
 * @param[in] *ina219c_dev : Pointer to struct ina219c_dev
 */
int8_t
ina219c_set_calibration(const struct ina219c_dev *dev);

/*!
 * @brief Get INA219C_REG_CALIBRATION value
 * @param[in] *ina219c_dev : Pointer to struct ina219c_dev
 */
int8_t
ina219c_get_calibration(const struct ina219c_dev *dev, uint16_t *value);

/*!
 * @brief Get PGA gain value
 * @param[in] *ina219c_dev : Pointer to struct ina219c_dev
 * @param[in] *gain : Pointer to ina219c_pga_gain_t variable
 */
int8_t
ina219c_get_pga_gain(const struct ina219c_dev *dev, ina219c_pga_gain_t *gain);

int8_t
ina219c_set_pga_gain(const struct ina219c_dev *dev, const ina219c_pga_gain_t gain);

int8_t
ina219c_get_sadc_value(const struct ina219c_dev *dev, ina219_resolution_t *res);

int8_t
ina219c_get_badc_value(const struct ina219c_dev *dev, ina219_resolution_t *res);

int16_t
ina219c_decomplement(uint16_t v, uint8_t sign_bits);

int8_t
ina219c_conversion_is_ready(const struct ina219c_dev *dev);

int8_t
ina219c_conversion_is_overflowed(const struct ina219c_dev *dev);

int8_t
ina219c_get_bus_voltage(const struct ina219c_dev *dev, float *voltage);

float
ina219c_get_currrent_lsb(const struct ina219c_dev *dev);
#endif