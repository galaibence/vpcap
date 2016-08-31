all:
	cd src && $(MAKE)
	cp ./lib/* ~/Libs/libvpcap/lib/
	cp ./inc/* ~/Libs/libvpcap/inc/