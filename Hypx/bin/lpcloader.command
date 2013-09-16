#!/bin/bash

if [ -z "$1" ]; then
	filepath=./firmware.bin
else
	filepath=$1
fi

if [ ! -f "$filepath" ]; then
	echo "Firmware file $filepath not found"
else
	echo "...Detecting LPC device"
	
	lpcpath=`df -P | grep "CRP DISABLD" | tr -s ' ' | cut -d ' ' -f 1`
	
	if [ "${#lpcpath}" == 0 ]; then
		echo "LPC device not detected"
	else
		echo "Device found at $lpcpath"
		echo "...Unmounting $lpcpath"
		
		if [ `uname` == 'Darwin' ]; then
			diskutil umount $lpcpath
		else
			umount ${lpcpath}
		fi

		echo "...Wiritng $filepath"
		dd if=$filepath of=$lpcpath seek=4

		echo "...Remounting $lpcpath"
		if [ `uname` == "Darwin" ]; then
			diskutil mount ${lpcpath}
		else
			mount ${lpcpath}
		fi
		
		echo "...Finished"
	fi
fi

exit

