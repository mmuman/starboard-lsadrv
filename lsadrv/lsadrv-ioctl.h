/*==========================================================================
 * Linux kernel driver for eIT-Xiroku optical touch sensor
 *
 * File: lsadrv-ioctl.h
 *
 * Purpose:
 *    definitions of ioctls for the lsadrv USB kernel driver
 *
 * Copyright (c) 2004-2005  eIT Co. Ltd. & Xiroku Inc.
 *
============================================================================*/
#ifndef _LSADRV_IOCTL_H_
#define _LSADRV_IOCTL_H_

//#include <linux/usb.h>	// for usb_device_descriptor

#ifndef __KERNEL__
#include <sys/types.h>		/* for u_intXX_t etc. */
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define LSADRV_IOC_MAGIC	'x'
#define LSADRV_IOCTL_BASE	192

/*--------------------------------------------------------------------------
 * mouse input event generation 
 *--------------------------------------------------------------------------*/
struct lsadrv_mouse_input {
    int dx;
    int dy;
    int flags;
};
/* flags */
#define MOUSEEVENTF_MOVE        0x0001 /* mouse move */
#define MOUSEEVENTF_LEFTDOWN    0x0002 /* left button down */
#define MOUSEEVENTF_LEFTUP      0x0004 /* left button up */
#define MOUSEEVENTF_RIGHTDOWN   0x0008 /* right button down */
#define MOUSEEVENTF_RIGHTUP     0x0010 /* right button up */
#define MOUSEEVENTF_MIDDLEDOWN  0x0020 /* middle button down */
#define MOUSEEVENTF_MIDDLEUP    0x0040 /* middle button up */
#define MOUSEEVENTF_XDOWN       0x0080 /* x button down */
#define MOUSEEVENTF_XUP         0x0100 /* x button down */
#define MOUSEEVENTF_WHEEL       0x0800 /* wheel button rolled */
#define MOUSEEVENTF_VIRTUALDESK 0x4000 /* map to entire virtual desktop */
#define MOUSEEVENTF_ABSOLUTE    0x8000 /* absolute move */

/*--------------------------------------------------------------------------
 * keyboard input event generation 
 *--------------------------------------------------------------------------*/
struct lsadrv_keybd_input {
    int vkey;	/* virtual key code */
    int flags;
};
/* flags */
#define KEYEVENTF_EXTENDEDKEY   0x0001 /* extended key */
#define KEYEVENTF_KEYUP         0x0002 /* key up */
#define KEYEVENTF_UNICODE       0x0004
#define KEYEVENTF_SCANCODE      0x0008
#define KEYEVENTF_INPUT_KEY     0x8000 /* key code defined in input.h */

#ifndef __KERNEL__
#ifndef __LINUX_USB_CH9_H
/*--------------------------------------------------------------------------
 * Device descriptor
 *--------------------------------------------------------------------------*/
struct usb_device_descriptor {
	u_int8_t  bLength;
	u_int8_t  bDescriptorType;
	u_int16_t bcdUSB;
	u_int8_t  bDeviceClass;
	u_int8_t  bDeviceSubClass;
	u_int8_t  bDeviceProtocol;
	u_int8_t  bMaxPacketSize0;
	u_int16_t idVendor;
	u_int16_t idProduct;
	u_int16_t bcdDevice;
	u_int8_t  iManufacturer;
	u_int8_t  iProduct;
	u_int8_t  iSerialNumber;
	u_int8_t  bNumConfigurations;
} __attribute__ ((packed));
#endif //__LINUX_USB_CH9_H
#endif //__KERNEL__

/*--------------------------------------------------------------------------
 * control structure for getting pipe info
 *--------------------------------------------------------------------------*/
struct lsadrv_pipe_info {
	u_int8_t 	bEndpointAddress;
	u_int8_t 	bmAttributes;
	u_int16_t	wMaxPacketSize;
	u_int8_t 	bInterval;
	u_int8_t 	bRefresh;
	u_int8_t 	bSynchAddress;
	u_int8_t 	bReserved;
} __attribute__ ((packed));

struct lsadrv_interface_info {
	u_int16_t	wLength;	/* OUT: length of valid data */
	u_int8_t 	bInterfaceNumber;
	u_int8_t 	bAlternateSetting;
	u_int8_t 	bInterfaceClass;
	u_int8_t 	bInterfaceSubClass;
	u_int8_t 	bInterfaceProtocol;
	u_int8_t 	bNumEndpoints;
	struct lsadrv_pipe_info Pipes[30];
} __attribute__ ((packed));

/*--------------------------------------------------------------------------
 * control structure for bulk and interrupt data transfers
 *--------------------------------------------------------------------------*/
struct lsadrv_bulk_transfer_control
{
	unsigned int ep;
	unsigned int len;
	unsigned int timeout; /* in milliseconds */
	void *data;
};

/*--------------------------------------------------------------------------
 * control structure for sending requests to the control endpoint.
 *--------------------------------------------------------------------------*/
struct lsadrv_control_transfer_control
{
	u_int8_t  requesttype;
	u_int8_t  request;
	u_int16_t value;
	u_int16_t index;
	u_int16_t length;
	u_int32_t timeout;  /* in milliseconds */
	void *data;
} __attribute__ ((packed));

/*--------------------------------------------------------------------------
 * control structure for isochronous data transfers
 *--------------------------------------------------------------------------*/
struct lsadrv_iso_transfer_control
{
	unsigned int Pipe;	/* endpoint address(1-15) + direction(0x80 for IN) */
	/*
	 * ISO packet size.  Determines how much data is transferred each
	 * frame.  Should be less than or equal to the maxpacketsize for
	 * the endpoint.
	 */
	unsigned int PacketSize;
	/* Total number of ISO packets to transfer. */
	unsigned int PacketCount;
	/*
	 * The following two parameters detmine how buffers are managed for
	 * an ISO transfer.  In order to maintain an ISO stream, the driver
	 * must create at least 2 transfer buffers and ping pong between them.
	 * BufferCount determines how many buffers the driver creates to ping
	 * pong between.  FramesPerBuffer specifies how many USB frames of data
	 * are transferred by each buffer.
	 */
	unsigned int FramesPerBuffer;     // 10 is a good value
	unsigned int BufferCount;         // 2 is a good value
};

/*--------------------------------------------------------------------------
 * control structure for reading isochronous stream data
 *--------------------------------------------------------------------------*/
/* isochronous transfer packet descriptor */
struct lsadrv_iso_packet_desc {
	unsigned int Length;	/* actual length of data received */
	unsigned int Status;
};

struct lsadrv_iso_read_control
{
	unsigned int PacketSize;
	unsigned int PacketCount;
	/* Timeout for reading ISO buffer (msec) */
	unsigned int Timeout;
	unsigned char *buffer;
	unsigned int  bufferSize;	/* IN: buffer sizer */
		/* buffer size = (PacketSize + sizeof(struct lsadrv_iso_packet_desc)) * PacketCount */
};

// device file in /proc file system
#define LSADRV_PROC_DIR_PATH	"/proc/lsadrv"


struct lsadrv_setinterface {
	unsigned int interface;
	unsigned int altsetting;
};

struct lsadrv_driver_version
{
	u_int16_t	MajorVersion;
	u_int16_t	MinorVersion;
	u_int16_t	BuildVersion;
};


///////////////////////////////////////////////////////
//
//              IOCTL Definitions
//
///////////////////////////////////////////////////////

/* get driver version */
#define LSADRV_IOC_GET_DRIVER_VERSION		_IOR(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 0, \
							struct lsadrv_driver_version)
/* send mouse input event */
#define LSADRV_IOC_MOUSEEVENT   		_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 1, \
							struct lsadrv_mouse_input)
/* send keyboard input event */
#define LSADRV_IOC_KEYBDEVENT  			_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 2, \
							struct lsadrv_keybd_input)
/* get device descriptor */
#define LSADRV_IOC_GET_DEVICE_DESCRIPTOR 	_IOR(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 3, \
							struct usb_device_descriptor)
/* get configuration descriptor */
#define LSADRV_IOC_GET_CONFIGURATION_DESCRIPTOR(size)	\
						_IOC(_IOC_READ, LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 4, \
							(size))	/* buffer size (must be < 16384)*/
/* get pipe information in current setting */
#define LSADRV_IOC_GET_PIPE_INFO		_IOR(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 5, \
							struct lsadrv_interface_info)
/* abort an endpoint */
#define LSADRV_IOC_ABORTPIPE  			_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 6, \
							unsigned int)	/* endpoint address + direction */

/*** following 6 ioctls have alternatives in devio, though... ***/
/* vendor or class request */
#define LSADRV_IOC_CONTROL			_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 8, \
							struct lsadrv_control_transfer_control)	/* ->USBDEVFS_CONTROL */
/* perform an IN/OUT transfer over the specified bulk or interrupt pipe */
#define LSADRV_IOC_BULK				_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 9, \
							struct lsadrv_bulk_transfer_control)	/* ->USBDEVFS_BULK */
/* reset an endpoint */
#define LSADRV_IOC_RESETPIPE  			_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 10, \
							unsigned int)	/* ->USBDEVFS_RESETEP */
/* cancel halt status of an endpoint */
#define LSADRV_IOC_CLEAR_HALT  			_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 11, \
							unsigned int)	/* ->USBDEVFS_CLEAR_HALT */
/* set configuration and alternative interface */
#define LSADRV_IOC_SETINTERFACE  		_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 12, \
							struct lsadrv_setinterface)	/* ->USBDEVFS_SETINTERFACE */
/* bus reset */
#define LSADRV_IOC_RESET 			_IO(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 13)	/* ->USBDEVFS_RESET */

/* start isochronous stream */
#define LSADRV_IOC_START_ISO_STREAM		_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 16, \
							struct lsadrv_iso_transfer_control)
/* stop isochronous stream */
#define LSADRV_IOC_STOP_ISO_STREAM		_IO(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 17)
/* read data from isochronous stream data buffer */
/* 	return value: >=0: length of data transfered; <0:error */
#define LSADRV_IOC_READ_ISO_BUFFER		_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 18, \
							struct lsadrv_iso_read_control)
/* claim/unclaim device for use of isochronous stream channel */
#define LSADRV_IOC_CLAIM_STREAM			_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 19, \
							int)
/* check if the device is ready */
#define LSADRV_IOC_CHECK			_IOR(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 20, \
							int)
/* get usb error information */
#define LSADRV_IOC_GET_LAST_ERROR		_IOR(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 21, \
							int[2])
/* retrieve the current USB frame number from the host controller */
#define LSADRV_IOC_GET_CURRENT_FRAME_NUMBER	_IOR(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 22, \
							int)

#ifdef __cplusplus
}
#endif

#endif /* _LSADRV_IOCTL_H_ */
