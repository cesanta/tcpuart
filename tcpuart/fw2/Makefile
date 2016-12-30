APP = tcpuart
ifneq ($(wildcard ../../mongoose-os),)
MGOS_PATH = ../../mongoose-os
else
MGOS_PATH = ../../..
endif
MAKEFILE_BUILD = Makefile.build

PLATFORM ?= esp8266

include $(MGOS_PATH)/fw/platforms/$(PLATFORM)/Makefile
