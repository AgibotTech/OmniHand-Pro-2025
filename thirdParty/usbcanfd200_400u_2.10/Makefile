KDIR=/lib/modules/`uname -r`/build
obj-m := usbcanfd.o
all: clean module deploy
module:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean
deploy:
	echo done
