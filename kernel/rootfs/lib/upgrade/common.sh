#!/bin/sh

. /etc/cdb.sh

RAM_ROOT=/tmp/root

ldd() { LD_TRACE_LOADED_OBJECTS=1 $*; }
libs() { ldd $* | awk '{print $3}'; }

install_file() { # <file> [ <file> ... ]
	for file in "$@"; do
		dest="$RAM_ROOT/$file"
		[ -f $file -a ! -f $dest ] && {
			dir="$(dirname $dest)"
			mkdir -p "$dir"
			cp $file $dest
		}
	done
}

install_bin() { # <file> [ <symlink> ... ]
	src=$1
	files=$1
	[ -x "$src" ] && files="$src $(libs $src)"
	install_file $files
	[ -e /lib/ld-linux.so.3 ] && {
		install_file /lib/ld-linux.so.3
	}
	shift
	for link in "$@"; do {
		dest="$RAM_ROOT/$link"
		dir="$(dirname $dest)"
		mkdir -p "$dir"
		[ -f "$dest" ] || ln -s $src $dest
	}; done
}

pivot() { # <new_root> <old_root>
	mount | grep "on $1 type" 2>&- 1>&- || mount -o bind $1 $1
	mkdir -p $1$2 $1/proc $1/dev $1/tmp $1/overlay && \
	mount -o move /proc $1/proc && \
	pivot_root $1 $1$2 || {
		v "`date`: pivot_root is failed"
        umount $1 $1
		return 1
	}
	mount -o move $2/dev /dev
	mount -o move $2/tmp /tmp
	mount -o move $2/overlay /overlay 2>&-
	ln -s /tmp /var
	return 0
}

run_ramfs() { # <command> [...]
	v "`date`: do run_ramfs"
	install_bin /bin/busybox /bin/ash /bin/sh /bin/sed /bin/mount /bin/umount /bin/sync /bin/dd /bin/grep /bin/cp /bin/mv /bin/tar /bin/vi /bin/ls /bin/cat /bin/sleep /bin/zcat /bin/date /bin/ps /bin/ln 
	install_bin /bin/busybox /sbin/start-stop-daemon /sbin/pivot_root /sbin/watchdog /sbin/reboot
	install_bin /bin/busybox /usr/bin/wget /usr/bin/md5sum "/usr/bin/[" /usr/bin/awk /usr/bin/hexdump /usr/bin/bzcat /usr/bin/cmp /usr/bin/head /usr/bin/tail /usr/bin/tr /usr/bin/wc

	install_bin /sbin/cdb
	install_bin /sbin/mtd
	install_bin /usr/sbin/uhttpd
	install_bin /usr/sbin/hostapd
	# /www/cli.cgi must be copied
	# need directory: /www and file: cli.cgi for uhttpd
	install_bin /www/cli.cgi
	for file in $RAMFS_COPY_BIN; do
		install_bin $file
	done
	install_file /etc/resolv.conf /etc/functions.sh /etc/rc.common /etc/cdb.sh /etc/init.d/uhttpd /lib/upgrade/*.sh /lib/wdk/upload /lib/wdk/reboot $RAMFS_COPY_DATA
	install_file /tmp/upload_state
	v "`date`: install bin/file done"

	pivot $RAM_ROOT /mnt || {
		v "`date`: Failed to switch over to ramfs. Please reboot."
		exit 1
	}
	v "`date`: pivot done"

	mount -o remount,ro /mnt
	umount -l /mnt

	grep /overlay /proc/mounts > /dev/null && {
		mount -o remount,ro /overlay
		umount -l /overlay
	}

	v "`date`: new shell from ramdisk"
	# spawn a new shell from ramdisk to reduce the probability of cache issues
	exec /bin/busybox ash -c "$*"
}

run_hooks() {
	local arg="$1"; shift
	for func in "$@"; do
		eval "$func $arg"
	done
}

ask_bool() {
	local default="$1"; shift;
	local answer="$default"

	[ "$INTERACTIVE" -eq 1 ] && {
		case "$default" in
			0) echo -n "$* (y/N): ";;
			*) echo -n "$* (Y/n): ";;
		esac
		read answer
		case "$answer" in
			y*) answer=1;;
			n*) answer=0;;
			*) answer="$default";;
		esac
	}
	[ "$answer" -gt 0 ]
}

v() {
	[ "$VERBOSE" -ge 1 ] && echo "$@" || echo "$@" >> /tmp/upgrade.log
	[ -c /dev/console ] && echo "$@" > /dev/console
}

rootfs_type() {
	mount | awk '($3 ~ /^\/$/) && ($5 !~ /rootfs/) { print $5 }'
}

get_image() { # <source> [ <command> ]
	local from="$1"
	local conc="$2"
	local cmd

	case "$from" in
		http://*|ftp://*) cmd="wget -O- -q";;
		*) cmd="cat";;
	esac
	if [ -z "$conc" ]; then
		local magic="$(eval $cmd $from | dd bs=2 count=1 2>/dev/null | hexdump -n 2 -e '1/1 "%02x"')"
		case "$magic" in
			1f8b) conc="zcat";;
			425a) conc="bzcat";;
		esac
	fi

	eval "$cmd $from ${conc:+| $conc}"
}

get_magic_word() {
	get_image "$@" | dd bs=2 count=1 2>/dev/null | hexdump -v -n 2 -e '1/1 "%02x"'
}

refresh_mtd_partitions() {
	mtd refresh rootfs
}

jffs2_copy_config() {
	if grep rootfs_data /proc/mtd >/dev/null; then
		# squashfs+jffs2
		mtd -e rootfs_data jffs2write "$CONF_TAR" rootfs_data
	else
		# jffs2
		mtd jffs2write "$CONF_TAR" rootfs
	fi
}

default_do_upgrade() {
	sync
	if [ "$SAVE_CONFIG" -eq 1 -a -z "$USE_REFRESH" ]; then
		get_image "$1" | mtd -j "$CONF_TAR" write - "${PART_NAME:-image}"
	else
		get_image "$1" | mtd write - "${PART_NAME:-image}"
	fi
}

do_upgrade() {
	local sysupgrade_result=1

	v "Performing system upgrade..."
	if type 'platform_do_upgrade' >/dev/null 2>/dev/null; then
		platform_do_upgrade "$ARGV"
	else
		default_do_upgrade "$ARGV"
	fi

	if type 'platform_check_flash' >/dev/null 2>/dev/null; then
		platform_check_flash "$ARGV"
		if [ "$?" = "0" ]; then
			v "*** do_upgrade is PASS!! ***"
			sysupgrade_result=0
		fi
	fi

	[ "$SAVE_CONFIG" -eq 1 -a -n "$USE_REFRESH" ] && {
		v "Refreshing partitions"
		if type 'platform_refresh_partitions' >/dev/null 2>/dev/null; then
			platform_refresh_partitions
		else
			refresh_mtd_partitions
		fi
		if type 'platform_copy_config' >/dev/null 2>/dev/null; then
			platform_copy_config
		else
			jffs2_copy_config
		fi
	}
	v "Upgrade completed"
	[ -n "$DELAY" ] && sleep "$DELAY"

#	/etc/init.d/uhttpd restart
#	v "uhttpd restart..."
#	ask_bool 1 "Reboot" && {
#		v "Rebooting system..."
#		reboot
#		sleep 5
#		echo b 2>/dev/null >/proc/sysrq-trigger
#	}

	exit ${sysupgrade_result}
}
