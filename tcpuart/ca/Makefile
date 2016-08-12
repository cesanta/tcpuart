all: certs

certs: server.key server.pem client.key client.pem

# ca.key is not checked in on purpose. For tests it's perfectly fine to
# re-generate everything from scratch.
ca.pem: Makefile
	openssl req \
		-out ca.pem \
		-keyout ca.key \
		-newkey rsa:1024 \
		-nodes \
		-subj '/CN=scandoc_test_ca/O=Quantex/OU=testing' \
		-days 3650 \
		-x509 \
		-text

ca.key: ca.pem
	@true

ca:
	mkdir $@

ca/index.txt:
	touch $@

ca/ca.srl:
	echo 00 > $@

ca.cnf: ca.pem ca.key ca ca/index.txt ca/ca.srl

%.csr: %.cnf
	openssl req \
		-out $@ \
		-keyout $(@:.csr=.key) \
		-newkey rsa:1024 \
		-nodes \
		-text \
		-days 3650 \
		-config $(@:.csr=.cnf)

%.key: %.csr
	@true

%.pem.tmp: %.csr %.cnf ca.cnf
	openssl ca \
		-in $< \
		-out $@ \
		-config ca.cnf \
		-batch

%.pem: %.pem.tmp
	openssl x509 \
		-in $< \
		-text > $@
	cat $(patsubst %.pem,%.key,$@) >> $@
	rm -f $(patsubst %.pem,%.key,$@)

clean:
	rm -f *.srl *.key *.pem index.txt* *.old
	rm -rf ca
