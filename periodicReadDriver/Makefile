obj-m += tempSensKM.o

KDIR = /lib/modules/$(shell uname -r)/build


all:
	# modules is a keyword defined by the linux kernel makefile as far as I could find
	# it looks for the files defined with obj-m in the directory, whicn in this case
	# was expanded to the current working dir with M and fetches them as compile targets
	make -C $(KDIR) M=$(shell pwd) modules

clean:
	# clean is also a keyword defined by the linux kernel, it seems to just clean all
	# generated filex *.c, *.mod.c, *.o, *.mod.o and such
	make -C $(KDIR) M=$(shell pwd) clean
	
