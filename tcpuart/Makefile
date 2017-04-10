.PHONY: all format

all:

UPLOAD_PATH=/web/mgos/httpdocs/downloads/tcpuart/
upload:
	for P in cc3200 esp8266; do \
		$(MAKE) -C fw2 clean all PLATFORM=$$P ; \
		gcloud compute copy-files \
			fw2/firmware/tcpuart-$$P-last.zip mgos-com:$(UPLOAD_PATH) ; \
	done

REPO_ROOT = ../..
include $(REPO_ROOT)/common.mk