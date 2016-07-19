#!/bin/bash

if ifconfig | grep "wpan0" ; then
	echo "wpan0 is up and will be set down..."
	ip link set wpan0 down
else
	echo "wpan0 is down"
fi

# check for loaded regmap module
if lsmod | grep "regmap_spi" &> /dev/null ; then
	echo "regmap_spi is loaded"
else
	echo "regmap_spi will be loaded..."
	insmod /home/pi/kernel/linux/drivers/base/regmap/regmap-spi.ko
fi

# at86rf230 can't be loaded if we want to use the lprf driver
if lsmod | grep "at86rf230" &> /dev/null ; then
	echo "at86rf230 is loaded and will be unloaded now..."
	rmmod at86rf230
else
	echo "at86rf230 is not loaded"
fi

# clean up the dmesg
echo "clean up the dmesg..."
dmesg -c &> /dev/null

if lsmod | grep "lprf_tx" &> /dev/null ; then
	echo "lprf module is currently loaded:"
	echo "remove lprf module..."
	rmmod lprf_tx
fi

echo "compile module..."
make 

echo "load lprf module..."
insmod lprf_tx.ko

cat /proc/modules

# create the device node for the char driver
echo "create device node..."
major=$(awk "\$2==\"lprf\" {print \$1}" /proc/devices)
rm -f /dev/lprf
mknod /dev/lprf c $major 0

echo "setup wpan device..."
ieee802154/setup_wpan.sh

echo "start userspace client..."
ieee802154/client_udp_802154
