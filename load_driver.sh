#!/bin/bash

if lsmod | grep "at86rf230" &> /dev/null ; then
	echo "at86rf230 is loaded and will be unloaded now"
	rmmod at86rf230
else
	echo "at86rf230 is not loaded"
fi

dmesg -c

if lsmod | grep "lprf_mod_1" &> /dev/null ; then
	echo "lprf module is currently loaded:"
	echo "remove lprf module..."
	rmmod lprf_mod_1
fi

echo "compile module..."
make 

echo "load lprf module..."
insmod lprf_mod_1.ko

cat /proc/modules

echo "create device node..."
mknod /dev/lprf c 243 0

echo "setup wpan device..."
/home/pi/ieee802154/setup_wpan.sh

echo "start userspace client..."
/home/pi/ieee802154/client_udp_802154
