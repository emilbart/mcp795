ARCH := arm
CROSS_COMPILE := /home/emba/rasberry/pi2/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
KDIR := /home/emba/rasberry/pi2/linux-rpi/
PWD := $(shell pwd)

MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

obj-m := rtc-mcp795-driver.o

all:
	$(MAKEARCH) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKEARCH) -C $(KDIR) M=$(PWD) clean
