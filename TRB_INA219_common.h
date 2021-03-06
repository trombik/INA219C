#if !defined(_TRB_INA219_common_h)
#define _TRB_INA219_common_h

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

/* read and write functions that must be implemented.
 * addr I2C address
 * reg_addr register address to read or write
 * data array of data to write or read
 * len the length of data
 */
typedef int32_t (*ina219_fptr_t) (const uint8_t addr, const uint8_t reg_addr, uint8_t *data, const uint8_t len);

void
ina219_delay_ms(const uint32_t period);

/* default I2C address */
#define INA219_ADDRESS		(0x40)

/* register addresses */
#define INA219_REG_CONFIG	(0x00)
#define INA219_REG_SHUNT	(0x01)
#define INA219_REG_BUS		(0x02)
#define INA219_REG_POWER	(0x03)
#define INA219_REG_CURRENT	(0x04)
#define INA219_REG_CALIBRATION	(0x05)

/* masks for configuration bits */
#define INA219_REG_CONFIG_MASK_MODE	(1 << 0 | 1 << 1 | 1 << 2)
#define INA219_REG_CONFIG_MASK_SADC	(1 << 3 | 1 << 4 | 1 << 5 | 1 << 6)
#define INA219_REG_CONFIG_MASK_BADC	(1 << 7 | 1 << 8 | 1 << 9 | 1 << 10)
#define INA219_REG_CONFIG_MASK_PG	(1 << 11 | 1 << 12)
#define INA219_REG_CONFIG_MASK_BRNG	(1 << 13)
#define INA219_REG_CONFIG_MASK_RESET	(1 << 15)

/* masks for bus voltage register */
#define INA219_REG_BUS_MASK_CNVR	(1 << 1)
#define INA219_REG_BUS_MASK_OVF	(1 << 0)
#define INA219_REG_BUS_MASK_BD		(0b1111111111111000)

/* return values of ina219_conversion_is_ready() */
#define INA219_CONVERSION_IS_READY 1
#define INA219_CONVERSION_IS_NOT_READY 0

/* return values of ina219_conversion_is_overflowed() */
#define INA219_IS_OVERFLOWED	1
#define INA219_IS_NOT_OVERFLOWED	2

/* default value of INA219_REG_CONFIG */
#define INA219_REG_CONFIG_DEFAULT	(0x399F)

/*!
 * @brief Bus Voltage range
 */
typedef enum
{
	INA219_BUS_VOLTAGE_RANGE_16V	= 0b00, /**< 16V */
	INA219_BUS_VOLTAGE_RANGE_32V	= 0b01  /**< 21V, default */
} ina219_range_t;

/*!
 * @brief Gain of PGA Function
 */
typedef enum
{
	INA219_PGA_GAIN_40MV	= 0b00, /**< 40mV */
	INA219_PGA_GAIN_80MV	= 0b01, /**< 80mV */
	INA219_PGA_GAIN_160MV	= 0b10, /**< 160mV */
	INA219_PGA_GAIN_320MV	= 0b11  /**< 320mV, default */
} ina219_pga_gain_t;

/*!
 * @brief ADC resolution/averaging and conversion time settings
 *
 * See also 8.6.2.1 Configuration Register
 */
typedef enum
{
	INA219_RESOLUTION_9BIT_1	= 0b0000, /**< 9 bit / 84 us */
	INA219_RESOLUTION_10BIT_1	= 0b0001, /**< 10 bit / 148 us */
	INA219_RESOLUTION_11BIT_1	= 0b0010, /**< 11 bit / 276 us */
	INA219_RESOLUTION_12BIT_1	= 0b0011, /**< 12 bit / 532 us, default */
	INA219_RESOLUTION_12BIT_2	= 0b1001, /**< 12 bit, 2 samples / 1.06 ms */
	INA219_RESOLUTION_12BIT_4	= 0b1010, /**< 12 bit, 4 samples / 2.13 ms */
	INA219_RESOLUTION_12BIT_8	= 0b1011, /**< 12 bit, 8 samples / 4.26 ms */
	INA219_RESOLUTION_12BIT_16	= 0b1100, /**< 12 bit, 16 samples / 8.51 ms */
	INA219_RESOLUTION_12BIT_32	= 0b1101, /**< 12 bit, 32 samples / 17.02 ms */
	INA219_RESOLUTION_12BIT_64	= 0b1110, /**< 12 bit, 64 samples / 34.05 ms */
	INA219_RESOLUTION_12BIT_128	= 0b1111  /**< 12 bit, 128 samples / 68.10 ms */
} ina219_resolution_t;

/*!
 * @brief ADC conversion time in us
 */
typedef enum
{
	INA219_RESOLUTION_9BIT_1_US	=    84,	/**< 9 bit / 84 us */
	INA219_RESOLUTION_10BIT_1_US	=   148,	/**< 10 bit / 148 us */
	INA219_RESOLUTION_11BIT_1_US	=   276,	/**< 11 bit / 276 us */
	INA219_RESOLUTION_12BIT_1_US	=   532,	/**< 12 bit / 532 us, default */
	INA219_RESOLUTION_12BIT_2_US	=  1060,	/**< 12 bit, 2 samples / 1.06 ms */
	INA219_RESOLUTION_12BIT_4_US	=  2130,	/**< 12 bit, 4 samples / 2.13 ms */
	INA219_RESOLUTION_12BIT_8_US	=  4260,	/**< 12 bit, 8 samples / 4.26 ms */
	INA219_RESOLUTION_12BIT_16_US	=  8510,	/**< 12 bit, 16 samples / 8.51 ms */
	INA219_RESOLUTION_12BIT_32_US	= 17020,	/**< 12 bit, 32 samples / 17.02 ms */
	INA219_RESOLUTION_12BIT_64_US	= 34050,	/**< 12 bit, 64 samples / 34.05 ms */
	INA219_RESOLUTION_12BIT_128_US	= 68100 	/**< 12 bit, 128 samples / 68.10 ms */
} ina219_conversion_time_t;

/*
 * @brief Operation mode
 *
 * See also 8.6.2.1 Configuration Register
 */
typedef enum
{
	INA219_MODE_POWERDOWN			= 0b000,
	INA219_MODE_SHUNT_TRIGGERED		= 0b001,
	INA219_MODE_BUS_TRIGGERED		= 0b010,
	INA219_MODE_SHUNT_BUS_TRIGGERED	= 0b011,
	INA219_MODE_ADC_OFF			= 0b100,
	INA219_MODE_SHUNT_CONTINUOUS		= 0b101,
	INA219_MODE_BUS_CONTINUOUS		= 0b110,
	INA219_MODE_SHUNT_BUS_CONTINUOUS	= 0b111, /* default */
} ina219_mode_t;

/*!
 * @brief A struct representing the device
 */
struct ina219_dev {
	uint8_t address; /**< I2C address (RW) */
	float i_max_expected; /**< Maximum expected i_bus in Amps (RW) */
	float r_shunt; /**< Shunt register value in Ohms (RW) */
	ina219_range_t range; /**< Bus Voltage range (RW) */
	ina219_pga_gain_t gain; /**< PGA gain (RW) */
	ina219_resolution_t bus_adc_resolution; /**< Bus ADC resolution (RW) */
	ina219_resolution_t shunt_adc_resolution; /** Shunt ADC resolution (RW) */
	ina219_mode_t mode; /**< Operaion mode (RW) */

	float v_shunt; /**< Shunt volatage in V (RO) */
	float v_bus; /**< Bus voltage in V (RO) */
	float p_bus; /**< Power in Watt (RO) */
	float i_bus; /**< Current in Amps (RO) */
	float current_lsb; /**< Current_LSB (RO) */
	float power_lsb; /**< Power_LSB (RO) */
	float i_max_possible; /**< Maximum measurable i_bus in Amps (RO) */
	uint16_t cal; /**< A Calibration value for Calibration Register (RO) */

	ina219_fptr_t read; /**< framework-dependant function pointer to I2C read (RO) */
	ina219_fptr_t write; /**< framework-dependant function pointer to I2C write (RO) */
};

/*!
 * @brief Create struct ina219_dev.
 *
 * @param[in] addr : I2C address of INA219
 * @retval srtuct *ina219_dev
 */
struct ina219_dev
ina219_create(const uint8_t addr);

/*!
 * @brief A thin wrapper of a function to read variable length of bytes from
 * slaves.
 *
 * This function is supposed to be implemented in code specific to
 * architectures, I2C libraries and/or frameworks, located under `sys`
 * directory.
 *
 * The implementation must implement the interface defined here, and provide
 * I2C read function.
 *
 * @param[in] addr : I2C address
 * @param[in] reg : Register address
 * @param[out] *data : Pointer to the variable to store the values
 * @param[in] len: Length of bytes to read
 */
int32_t
ina219_read(const uint8_t addr, const uint8_t reg, uint8_t *data, const uint8_t len);

/*!
 * @brief A thin wrapper of a function to writes variable length of bytes to
 * slaves.
 *
 * The implementation must implement the interface defined here, and provide
 * I2C read function.
 *
 * See also ina219_create().
 *
 * @param[in] addr : I2C address
 * @param[in] reg : Register address
 * @param[in] data : The variable that stores the values to write
 * @param[in] len : Length of bytes to write
 */
int32_t
ina219_write(const uint8_t addr, const uint8_t reg, uint8_t *data, const uint8_t len);

/*!
 * @brief Writes two bytes to a register.
 *
 * This function simply calls ina219_write() with len = 2.
 *
 * @param[in] *ina219_dev : A struct of ina219_dev
 * @reg[in] reg : Register address
 * @data[in] data : Two bytes of data to write
 */
int32_t
ina219_write16(const struct ina219_dev *ina219_dev, const uint8_t reg, const uint16_t *data);

/*!
 * @brief Read 2bytes from a regsiter
 *
 * This function simply calls ina219_read() with len = 2.
 *
 * @param[in] *ina219_dev : A pointer to struct ina219_dev
 * @param[in] reg : Register address
 * @param[in] *data : A pointer to store the value
 */
int32_t
ina219_read16(const struct ina219_dev *ina219_dev, const uint8_t reg, uint16_t *data);

/*!
 * @brief Write one or more bits to a register at a specific bit.
 *
 * @param[in] *ina219_dev : A struct ina219_dev
 * @param[in] reg : Register address
 * @param[in] mask : Shift bit when setting the value. To write 0b11 at 8th
 * bit (8th and 9th bit will be 1), mask should be 0b00000011 00000000
 * @param[in] value : Value to set
 */
int32_t
ina219_set_bits(const struct ina219_dev *dev, const uint8_t reg, const uint16_t mask, const uint16_t value);

/*!
 * @brief Reset the device.
 *
 * Reset the device. Will set all the register values to defaults.
 */
int32_t
ina219_reset(struct ina219_dev *dev);

/*!
 * @brief Get operating mode
 * @param[in] *ina219_dev : A pointer to struct ina219_dev
 * @param[out] *mode : A pointer to a variable to store one of ina219_mode_t
 */
int32_t
ina219_get_mode(const struct ina219_dev *dev, ina219_mode_t *mode);

/*!
 * @brief Get Bus Voltage Range
 * @param[in] *ina219_dev : A pointer to struct ina219_dev
 * @param[out] range : one of ina219_range_t
 */
int32_t
ina219_get_v_bus_range(const struct ina219_dev *dev, ina219_range_t *range);

/*!
 * @brief Caribrate the sensor.
 *
 * The function calculates value for the calibration register from defined
 * settings in ina219_dev. The value will be written to the register.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 */
int32_t
ina219_set_calibration(struct ina219_dev *dev);

/*!
 * @brief Get INA219_REG_CALIBRATION value from calibration register
 *
 * Read the calibration register and save the value to a variable.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 * @param[out] *value : variable to store the value
 */
int32_t
ina219_get_calibration(struct ina219_dev *dev, uint16_t *value);

/*!
 * @brief Get PGA gain value from configuration register.
 *
 * Read the PGA gain from the configuration register, save the value to a
 * variable.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 * @param[out] *gain : Pointer to ina219_pga_gain_t variable
 */
int32_t
ina219_get_pga_gain(const struct ina219_dev *dev, ina219_pga_gain_t *gain);

/*!
 * @brief Get Bus ADC Resolution from confgiuration register, save the value
 * to a variable.
 *
 * @param[in] *ina219_dev : A struct ina219_dev
 * @param[out] res : Variable to store ina219_resolution_t value
 */
int32_t
ina219_get_sadc_value(const struct ina219_dev *dev, ina219_resolution_t *res);

/*
 * @brief Get Shunt ADC Resolution from confgiuration register, save the value
 * to a variable.
 *
 * @param[in] *ina219_dev : A struct ina219_dev
 * @param[out] res : Variable to store ina219_resolution_t value
 */
int32_t
ina219_get_badc_value(const struct ina219_dev *dev, ina219_resolution_t *res);

/*!
 * @brief Convert complemented value to regular value
 *
 * Return human-readable value of 2's complement of a negative number.
 *
 * See also 8.6.3.1 Shunt Voltage Register.
 *
 * @param[in] v : the complemented value
 * @param[in] sign_bits : number of SIGN bits in the complemented value
 * @retval A regular number after calculation
 */
int16_t
ina219_decomplement(uint16_t v, uint8_t sign_bits);

/*!
 * @brief Return whether conversion is ready or not.
 *
 * Read CNVR, Conversion Ready bit, in Bus Voltage Register, returns the
 * result.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 * @retval INA219_CONVERSION_IS_READY or INA219_CONVERSION_IS_NOT_READY
 */
int32_t
ina219_conversion_is_ready(const struct ina219_dev *dev);

/*!
 * @brief Return whether p_bus or i_bus calculation are out of range
 *
 * Return OVF, or Math Overflow Flag, in Bus Voltage Register, return
 * the result.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 */
int32_t
ina219_conversion_is_overflowed(const struct ina219_dev *dev);

/*!
 * @brief Get bus voltage reading
 *
 * Read Bus Voltage Register, return the voltage after conversion.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 * @param[out] voltage : Bus voltage in V
 */
int32_t
ina219_get_v_bus(const struct ina219_dev *dev, float *voltage);

/*!
 * @brief Read one or more bits from a register
 *
 * Read a register, extract spicific bits using a given mask.
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 *
 * @param[in] reg : Register address
 * @param[in] mask : Mask of bits to read
 * @param[out] *value : Variable to save the result
 */
int32_t
ina219_get_bits(const struct ina219_dev *dev, const uint8_t reg, const uint16_t mask, uint16_t *value);

/*!
 * @brief Helper function to calculate bits to shift from bit mask
 *
 * Calclate how many bits to shift from a given mask
 *
 * @param[in] mask : A bit mask
 * @retval number of times to shift
 */
uint8_t
ina219_get_bits_from_mask(uint16_t mask);

/*!
 * @brief Get p_bus in W from Power Register
 *
 * Read Power Register and return the p_bus after calculation.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 * @p_bus[out] *p_bus : Variable to save the value
 */
int32_t
ina219_get_p_bus(const struct ina219_dev *dev, float *p_bus);

/*!
 * @brief Get i_bus in Amps from Current Register
 *
 * Read Current Register and return the i_bus after calculation.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 * @p_bus[out] *i_bus : Variable to save the value
 */
int32_t
ina219_get_i_bus(const struct ina219_dev *dev, float *i_bus);

int32_t
ina219_calc_calibration(struct ina219_dev *dev);

/*!
 * @brief Get i_bus, p_bus, shunt register voltage, Vbus voltage values from
 * multiple registers
 *
 * Call ina219_get_v_shunt(), ina219_get_v_bus(),
 * ina219_get_p_bus(), and ina219_get_i_bus(), then save the values in
 * struct ina219_dev.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 */
int32_t
ina219_get_sensor_values(struct ina219_dev *dev);

/*!
 * @brief Set configuration values to Configuration Register
 *
 * Collect all the configuration values in struct ina219_dev, writes them to
 * Configuration Register.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 */
int32_t
ina219_configure(struct ina219_dev * dev);

/*!
 * @brief Get shunt register voltage from Shunt Voltage Register
 *
 * Read Shunt Voltage Register, and save the voltage in Volts to a variable.
 *
 * @param[in] *ina219_dev : Pointer to struct ina219_dev
 * @param[out] v_shunt : Variable to save the value
 */
int32_t
ina219_get_v_shunt(struct ina219_dev *dev, float *v_shunt);

#if defined(__cplusplus)
}
#endif

#endif // !defined(_TRB_INA219_common_h)
