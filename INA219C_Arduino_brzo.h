#if !defined(_INA219C_Arduino_brzo_h)
#define _INA219C_Arduino_brzo_h

#include "INA219C_common.h"
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

uint16_t
ina219c_brzo_get_scl_freq();

uint16_t
ina219c_brzo_set_scl_freq(const uint16_t freq);

#if defined(__cplusplus)
}
#endif

#endif // !defined(_INA219C_Arduino_Wire_h)
