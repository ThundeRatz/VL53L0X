THIS_PATH := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

C_INCLUDES +=                                                          \
	-I$(THIS_PATH)/core/inc                                            \
	-I$(THIS_PATH)/platform/inc                                        \

LIB_SOURCES += $(shell find $(THIS_PATH)/core/src $(THIS_PATH)/platform/src -name "*.c")


undefine THIS_PATH
