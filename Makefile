obj-m += lprf_mod_1.o
#obj-m += at86rf230.o

#scull-objs := lprf.o

KERNELDIR ?= /home/pi/linux

all:
	make -C $(KERNELDIR) M=$(PWD) LDDINC=$(PWD)/../include modules

clean:
	make -C $(KERNELDIR) M=$(PWD) clean


