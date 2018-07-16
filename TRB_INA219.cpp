#if defined(TRB_INA219_I2C_WIRE)
#include "TRB_INA219_Arduino_Wire.cpp"
#elif defined(TRB_INA219_I2C_BRZO)
#include "TRB_INA219_Arduino_brzo.cpp"
#else
#error Please define TRB_INA219_I2C_WIRE or TRB_INA219_I2C_BRZO
#endif
