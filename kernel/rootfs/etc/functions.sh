#!/bin/sh
# Copyright (C) 2006 OpenWrt.org
# Copyright (C) 2006 Fokus Fraunhofer <carsten.tittel@fokus.fraunhofer.de>

debug () {
	${DEBUG:-:} "$@"
}
mount() {
	busybox mount "$@"
}

# newline
N="
"

_C=0
NO_EXPORT=1
LOAD_STATE=1
LIST_SEP=" "

hotplug_dev() {
	env -i ACTION=$1 INTERFACE=$2 /sbin/hotplug-call net
}

append() {
	local var="$1"
	local value="$2"
	local sep="${3:- }"

	eval "export ${NO_EXPORT:+-n} -- \"$var=\${$var:+\${$var}\${value:+\$sep}}\$value\""
}

list_contains() {
	local var="$1"
	local str="$2"
	local val

	eval "val=\" \${$var} \""
	[ "${val%% $str *}" != "$val" ]
}

list_remove() {
	local var="$1"
	local remove="$2"
	local val

	eval "val=\" \${$var} \""
	val1="${val%% $remove *}"
	[ "$val1" = "$val" ] && return
	val2="${val##* $remove }"
	[ "$val2" = "$val" ] && return
	val="${val1## } ${val2%% }"
	val="${val%% }"
	eval "export ${NO_EXPORT:+-n} -- \"$var=\$val\""
}

load_modules() {
	[ -d /etc/modules.d ] && {
		cd /etc/modules.d
		sed 's/^[^#]/insmod &/' $* | ash 2>&- || :
	}
}

include() {
	local file

	for file in $(ls $1/*.sh 2>/dev/null); do
		. $file
	done
}

find_mtd_part() {
	local PART="$(grep "\"$1\"" /proc/mtd | awk -F: '{print $1}')"
	local PREFIX=/dev/mtdblock

	PART="${PART##mtd}"
	[ -d /dev/mtdblock ] && PREFIX=/dev/mtdblock/
	echo "${PART:+$PREFIX$PART}"
}

jffs2_mark_erase() {
	local part="$(find_mtd_part "$1")"
	[ -z "$part" ] && {
		echo Partition not found.
		return 1
	}
	echo -e "\xde\xad\xc0\xde" | mtd -qq write - "$1"
}

