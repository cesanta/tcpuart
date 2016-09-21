APP = tcpuart
ifneq ($(wildcard ../../mongoose-iot),)
MIOT_PATH = ../../mongoose-iot
else
MIOT_PATH = ../../..
endif

PLATFORM ?= esp8266

include $(MIOT_PATH)/fw/platforms/$(PLATFORM)/Makefile
