#!/bin/bash
################################################################################
# Copyright (c) 2020 ModalAI, Inc. All rights reserved.
#
# Installs the ipk package on target.
# Requires the ipk to be built and an adb connection.
################################################################################
set -e

PACKAGE=$(cat ipk/control/control | grep "Package" | cut -d' ' -f 2)

# count ipk files in current directory
NUM_FILES=$(ls -1q $PACKAGE*.ipk | wc -l)

if [ $NUM_FILES -eq "0" ]; then
	echo "ERROR: no ipk file found"
	echo "run build.sh and make_ipk.sh first"
	exit 1
elif [ $NUM_FILES -gt "1" ]; then
	echo "ERROR: more than 1 ipk file found"
	echo "make sure there is only one ipk file in the current directory"
	exit 1
fi

# now we know only one ipk file exists
FILE=$(ls -1q $PACKAGE*.ipk)

if [ "$1" == "ssh" ]; then
	if [ -f /usr/bin/sshpass ];then
		if [ -z ${VOXL_IP+x} ]; then
			echo "Did not find a VOXL_IP env variable,"
			echo ""
			echo "If you would like to push over ssh automatically,"
			echo "please export VOXL_IP in your bashrc"
			echo ""
			read -p "Please enter an IP to push to:" SEND_IP

		else
			SEND_IP="${VOXL_IP}"
		fi

		echo "Pushing File to $SEND_IP"
		sshpass -p "oelinux123" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null ./$FILE root@$SEND_IP:/home/root/ipk/$FILE 2>/dev/null > /dev/null \
		&& echo "File pushed, Installing" \
		&& sshpass -p "oelinux123" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null root@$SEND_IP "opkg install --force-reinstall --force-downgrade --force-depends /home/root/ipk/$FILE" 2>/dev/null
	else
		echo ""
		echo "You do not have sshpass installed"
		echo "Please install sshpass to use the install via ssh feature"
		echo ""
	fi

else
	if [ -f /usr/bin/adb ];then
		echo "searching for ADB device"
		adb wait-for-device
		echo "adb device found"

		echo "pushing $FILE to target"
		adb push $FILE /home/root/ipk/$FILE
		adb shell "opkg install --force-reinstall --force-downgrade --force-depends /home/root/ipk/$FILE"
	else
		echo ""
		echo "You do not have adb installed"
		echo "Please install adb to use the install via adb feature"
		echo ""
	fi
fi
