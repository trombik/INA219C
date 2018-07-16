#if !defined(_TRB_INA219_c)
#define _TRB_INA219_c

#if defined(TRB_INA219_I2C_ESP_IDF)
#include "TRB_INA219_ESP_IDF.c"
#else
#error no I2C was chosen
#endif

#endif // defined(TRB_INA219_I2C_ESP_IDF)
