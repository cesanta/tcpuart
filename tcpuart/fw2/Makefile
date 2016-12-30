APP = tcpuart
ifneq ($(wildcard ../../mongoose-iot),)
MGOS_PATH = ../../mongoose-iot
else
MGOS_PATH = ../../..
endif
MAKEFILE_BUILD = Makefile.build

PLATFORM ?= esp8266

include $(MGOS_PATH)/fw/platforms/$(PLATFORM)/Makefile
