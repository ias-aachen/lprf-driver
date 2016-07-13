#!/bin/bash

sudo iwpan dev wpan0 set pan_id 0xdead
sudo iwpan dev wpan0 set short_addr 0xbeef
sudo ip link set wpan0 up

ifconfig 
