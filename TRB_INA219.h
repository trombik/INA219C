#if !defined(_TRB_INA219_h)
#define _TRB_INA219_h

#if defined(TRB_INA219_I2C_ESP_IDF)
#include "TRB_INA219_ESP_IDF.h"

#elif defined(TRB_INA219_I2C_WIRE)
#include "TRB_INA219_Arduino_Wire.h"

#elif defined(TRB_INA219_I2C_BRZO)
#include "TRB_INA219_Arduino_brzo.h"

#elif defined(TRB_INA219_I2C_LIB_I2C)
#include "TRB_INA219_Arduino_LIB_I2C.h"

#else
#error Please define one of TRB_INA219_I2C_ESP_IDF, TRB_INA219_I2C_WIRE, TRB_INA219_I2C_BRZO, and TRB_INA219_I2C_LIB_I2C
#endif // defined(TRB_INA219_I2C_ESP_IDF)

#endif // !defined(_TRB_INA219_h)
