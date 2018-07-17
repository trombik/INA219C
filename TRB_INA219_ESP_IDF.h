#if !defined(_TRB_INA219_ESP_IDF_h)
#define _TRB_INA219_ESP_IDF_h

#include "TRB_INA219_common.h"
#include <driver/i2c.h>

/*!
 * @brief Set I2C port to use
 *
 * @param[in] port : i2c_port_t (default is I2C_NUM_0)
 * @retval i2c_port_t
 */
i2c_port_t
ina219_set_i2c_port(i2c_port_t port);

/*!
 * @brief Get I2C port in use
 *
 * @retval i2c_port_t
 */
i2c_port_t
ina219_get_i2c_port();

#endif // !defined(_TRB_INA219_ESP_IDF_h)
