# This is a FreeBSD kernel driver that implements NT synchronization
# primitives. It is designed to support high-performance synchronization
# for compatibility layers like Wine and Proton.
#
# Build with WOW64 / 32-bit Wine support (COMPAT_FREEBSD32):
#
#   make COMPAT32=yes
#
# Without the flag the driver still works for 64-bit Wine; the
# ntsync_proc_is_32bit() helper simply returns false and the compiler
# optimises away the compat32 guard blocks entirely.
#
# The kernel itself must be built with options COMPAT_FREEBSD32 (it is
# in the stock GENERIC kernel on amd64) for SV_ILP32 to be defined.
# The module does not require a COMPAT_FREEBSD32 kernel to load; the
# #ifdef guards make it safe either way.

KMOD    = ntsync
SRCS    = ntsync.c

SRCS   += device_if.h bus_if.h vnode_if.h

SRCS   += opt_global.h

NTS_HEADER= ntsync.h
HEADER_DIR= /usr/local/include/linux

CFLAGS += -I${SYSDIR}/compat/linuxkpi/common/include
CFLAGS += -I${SYSDIR}/compat/linuxkpi/dummy/include

CFLAGS += -fstack-protector-strong
CFLAGS += -fno-common
CFLAGS += -Wall

.if defined(COMPAT32) && ${COMPAT32} == "yes"
CFLAGS += -DCOMPAT_FREEBSD32
# Pull in the FreeBSD compat32 headers from the kernel source tree.
# Adjust SYSDIR if your kernel sources live elsewhere.
SYSDIR ?= /usr/src/sys
CFLAGS += -I${SYSDIR}/compat/freebsd32
.endif

# Debug logging
# CFLAGS += -DNTSYNC_DEBUG

.include <bsd.kmod.mk>

afterinstall:
	@echo "Installing header: ${NTS_HEADER} to ${HEADER_DIR}"
	install -d ${DESTDIR}${HEADER_DIR}
	${INSTALL} -m 444 ${.CURDIR}/${NTS_HEADER} ${DESTDIR}${HEADER_DIR}
