/*
 * decode_ioctl
 * Copyright 2014, François Revol <revol@free.fr>.
 * Distributed under the terms of the MIT License.
 *
 * 64bit build:
 * make decode_ioctl
 *
 * 32bit build:
 * make decode_ioctl CFLAGS=-m32
 */

#include <stdlib.h>
#include <stdio.h>

#include <asm/ioctl.h>

#include "lsadrv/lsadrv-ioctl.h"

// packed versions of some structs to see the difference...

struct lsadrv_bulk_transfer_control_packed
{
	unsigned int ep;
	unsigned int len;
	unsigned int timeout; /* in milliseconds */
	void *data;
} __attribute__ ((packed));

struct lsadrv_iso_read_control_packed
{
	unsigned int PacketSize;
	unsigned int PacketCount;
	/* Timeout for reading ISO buffer (msec) */
	unsigned int Timeout;
	unsigned char *buffer;
	unsigned int  bufferSize;	/* IN: buffer sizer */
		/* buffer size = (PacketSize + sizeof(struct lsadrv_iso_packet_desc)) * PacketCount */
} __attribute__ ((packed));

void decode_ioctl(unsigned long code)
{
	printf("code: %ld 0x%08lx\n", code, code);
	printf("DIR:  %ld 0x%08lx\n", _IOC_DIR(code), _IOC_DIR(code));
	printf("TYPE: %ld 0x%08lx '%c'\n", _IOC_TYPE(code), _IOC_TYPE(code),
		(char)_IOC_TYPE(code));
	printf("NR:   %ld 0x%08lx\n", _IOC_NR(code), _IOC_NR(code));
	printf("SIZE: %ld 0x%08lx\n", _IOC_SIZE(code), _IOC_SIZE(code));
}

int main(int argc, char **argv)
{
	unsigned long code;

	/*
	if (argc < 2) {
		fprintf(stderr, "Usage: %s <code>\n", argv[0]);
		return 1;
	}
	code = strtoul(argv[1], NULL, 0);
	decode_ioctl(code);
	*/
	decode_ioctl(0x401078c8);

	printf("sizes for %d bits:\n", (sizeof(long) == 4 ? 32 : 64));

#define print_struct_sz(st) \
	printf("sizeof(struct %s) = %ld\n", \
		#st, \
		sizeof(struct st));

	print_struct_sz(lsadrv_mouse_input);
	print_struct_sz(lsadrv_bulk_transfer_control);
	print_struct_sz(lsadrv_bulk_transfer_control_packed);
	print_struct_sz(lsadrv_control_transfer_control);
	print_struct_sz(lsadrv_iso_read_control);
	print_struct_sz(lsadrv_iso_read_control_packed);
	/*
	printf("sizeof(int) = %d\n", sizeof(int));
	printf("sizeof(long) = %d\n", sizeof(long));
	printf("sizeof(void *) = %d\n", sizeof(void *));
	*/
}
