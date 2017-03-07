#!/bin/bash
# should execute by "sh /usr/sbin/access_usb_storage.sh"
# cd ramfs or tmpfs
echo "access usb storage"
i=0
total=0
echo "src="$PWD
dst="/mnt/usb"
while [ true ]
do
	RANDOM=`date '+%s'`
	# generate filename
	filename="aaa${RANDOM}.dat"
	size=$(( $RANDOM % 2000 ))
	i=$(( $i + 1 ))
	echo "size=${size}"
	# generate random data in file
	dd if=/dev/urandom of=$filename bs=1 count=$size 2> /dev/null
	# copy file to $dst
	cp $filename $dst
	# request to flush file system buffers
	sync
	# flush cache
	echo 3 > /proc/sys/vm/drop_caches
	# compare file
	if cmp $filename $dst/$filename ; then
		echo "OK count=${i}"
	else
		break
	fi
	total=$(( $total + $size ))
	# check size
	if [ $(( $i % 254 )) -eq 0 ] ; then
		echo "remove all aaa*.dat files in directory $PWD and $dst"
		rm -rf aaa*.dat
		rm -rf $dst/aaa*.dat
		total=0
	fi
done
