CLANG_FORMAT:=clang-format
# installable with: `brew install llvm36 --with-clang`
ifneq ("$(wildcard /usr/local/bin/clang-format-3.6)","")
	CLANG_FORMAT:=/usr/local/bin/clang-format-3.6
endif

.PHONY: all format

all:

format:
ifneq ($(OS),Windows_NT)
#  In Windows, clang-format "formats" symlinks
	@(for i in fw2 test; do \
		/usr/bin/find $$i -type f -name "*.[ch]" | grep -v /\.build.*/; \
		done) | xargs $(CLANG_FORMAT) -i
else
	@/usr/bin/true
endif

UPLOAD_PATH=/web/mgos/httpdocs/downloads/tcpuart/
upload:
	for P in cc3200 esp8266; do \
		$(MAKE) -C fw2 clean all PLATFORM=$$P ; \
		gcloud compute copy-files \
			fw2/firmware/tcpuart-$$P-last.zip mgos-com:$(UPLOAD_PATH) ; \
	done
