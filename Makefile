#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := INA219C
UNIT_TEST_APP :=	$(IDF_PATH)/tools/unit-test-app
# include unity component for unit testing
EXTRA_COMPONENT_DIRS=	$(UNIT_TEST_APP)/components
SDKCONFIG_DEFAULTS=	 $(UNIT_TEST_APP)/sdkconfig.defaults

CFLAGS := -DINA219_I2C_ESP_IDF -DGPIO_SDA=21 -DGPIO_SCL=22

include $(UNIT_TEST_APP)/Makefile
