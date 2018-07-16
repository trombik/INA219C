#if defined(INA219C_I2C_WIRE)
#include "INA219C_Arduino_Wire.cpp"
#elif defined(INA219C_I2C_BRZO)
#include "INA219C_Arduino_brzo.cpp"
#else
#error Please define INA219C_I2C_WIRE or INA219C_I2C_BRZO
#endif
