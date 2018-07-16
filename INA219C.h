#if !defined(_INA219C_h)
#define _INA219C_h

#if defined(INA219C_I2C_ESP_IDF)
#include "INA219C_ESP_IDF.h"

#elif defined(INA219C_I2C_WIRE)
#include "INA219C_Arduino_Wire.h"

#elif defined(INA219C_I2C_BRZO)
#include "INA219C_Arduino_brzo.h"
#else
#error Please define one of INA219C_I2C_ESP_IDF, INA219C_I2C_WIRE, and INA219C_I2C_BRZO
#endif // defined(INA219C_I2C_ESP_IDF)

#endif // !defined(_INA219C_h)
