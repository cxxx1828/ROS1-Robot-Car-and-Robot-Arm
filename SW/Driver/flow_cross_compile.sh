#!/bin/bash

exit 0

cd motor_ctrl/

RASBIAN=$PWD/../../../../RPi_Robo_Framework_Work/Rasbian/

pushd $RASBIAN
mkdir boot rootfs

IMG=Raspbian_Buster_with_ROS_for_RPi2-v2023.1.img
fdisk -l $IMG
#TODO end
#sudo mount -o loop,offset=$((512*8192)) $IMG boot
sudo mount -o loop,offset=$((512*532480)) $IMG rootfs

popd


export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
make KDIR=$RASBIAN/rootfs/usr/src/linux-headers-5.10.103-v7+/
#TODO KBUILD_SCRIPTROOT=/lib/modules/$(CURRENT)/build V=1
