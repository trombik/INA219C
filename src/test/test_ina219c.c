#include "unity.h"
#include "INA219C.h"

TEST_CASE("first_test", "[INA219C]")
{
	struct ina219c_dev dev;
	const uint8_t i2c_address = 0x40;
	dev = ina219c_create(i2c_address);
	TEST_ASSERT_EQUAL_UINT8(i2c_address, dev.address);
}
