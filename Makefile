#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := INA219C
# include unity component for unit testing
EXTRA_COMPONENT_DIRS=	$(IDF_PATH)/tools/unit-test-app/components

include $(IDF_PATH)/make/project.mk
