THIS_PATH := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

C_INCLUDES +=                                                          \
	-I$(THIS_PATH)/core/inc                                            \
	-I$(THIS_PATH)/platform/inc                                        \
    -I$(THIS_PATH)/inc                                                 \

LIB_SOURCES += $(shell find $(THIS_PATH)/core/src $(THIS_PATH)/platform/src $(THIS_PATH)/src -name "*.c")


undefine THIS_PATH
