#if !defined(_INA219C_c)
#define _INA219C_c

#if defined(INA219C_I2C_ESP_IDF)
#include "INA219C_ESP_IDF.c"
#else
#error no I2C was chosen
#endif

#endif // defined(INA219C_I2C_ESP_IDF)
