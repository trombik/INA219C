#if !defined(_TRB_INA219_Arduino_brzo_h)
#define _TRB_INA219_Arduino_brzo_h

#include "TRB_INA219_common.h"
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

uint16_t
ina219_brzo_get_scl_freq();

uint16_t
ina219_brzo_set_scl_freq(const uint16_t freq);

#if defined(__cplusplus)
}
#endif

#endif // !defined(_TRB_INA219_Arduino_Wire_h)
