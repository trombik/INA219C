#if defined(TRB_INA219_I2C_WIRE)
#include "TRB_INA219_Arduino_Wire.cpp"

#elif defined(TRB_INA219_I2C_BRZO)
#include "TRB_INA219_Arduino_brzo.cpp"

#elif defined(TRB_INA219_I2C_LIB_I2C)
#include "TRB_INA219_Arduino_LIB_I2C.cpp"

#else
#error Please define TRB_INA219_I2C_WIRE, TRB_INA219_I2C_LIB_I2C or TRB_INA219_I2C_BRZO
#endif
