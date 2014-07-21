/*==========================================================================
 * lsadrv.h : Linux kernel driver for eIT-Xiroku optical touch sensor header file
 *
 * Copyright (c) 2004-2009  eIT Co. Ltd. & Xiroku Inc.
 *
============================================================================*/

#ifndef LSADRV_H
#define LSADRV_H

#include <linux/spinlock.h>
#include <linux/wait.h>

#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif

#include <linux/input.h>
#include <linux/usb.h>
#include "lsadrv-ioctl.h"

/* Turn some debugging options on/off */
#define LSADRV_DEBUG 0
//#define CHECK_INPUT_DEVICE_NAME 1

/* define this to prevent being connected to mousedev driver (experiment) */
//#define PREVENT_MOUSE_DRIVER_MATCH

/* Driver version */
#define LSADRV_KDRIVER_MAJOR	1
#define LSADRV_KDRIVER_MINOR	2
#define LSADRV_KDRIVER_BUILD	3
#define LSADRV_KDRIVER_VERSION 	"1.2.3"
#define LSADRV_NAME 	"lsadrv"

/* Defines and structures for the eIT-Xiroku light sensor */

/* Trace certain actions in the driver */
#define LSADRV_TRACE_MODULE	0x0001
#define LSADRV_TRACE_PROBE	0x0002
#define LSADRV_TRACE_OPEN	0x0004
#define LSADRV_TRACE_STREAM	0x0008
#define LSADRV_TRACE_IOCTL	0x0010
#define LSADRV_TRACE_MOUSE	0x0020
#define LSADRV_TRACE_MEMORY	0x0040
#define LSADRV_TRACE_READ	0x0080
#define LSADRV_TRACE_FLOW	0x0200

#define Trace(R, A...) if (lsadrv_trace & R) lsadrv_printk(KERN_DEBUG LSADRV_NAME " " A)
#define Debug(A...) lsadrv_printk(KERN_DEBUG LSADRV_NAME " " A)
#define Info(A...)  lsadrv_printk(KERN_INFO  LSADRV_NAME " " A)
#define Warning(A...)  lsadrv_printk(KERN_WARNING  LSADRV_NAME " " A)
#define Err(A...)   lsadrv_printk(KERN_ERR   LSADRV_NAME " " A)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36)
#define init_MUTEX(sem) sema_init(sem, 1)
#endif

/* 
 * bit operation macros (defined in bitops.h in kernel 2.26.24 or later)
 */
#ifndef BIT_MASK
/* BIT_MASK() - former BIT() */
#define BIT_MASK(nr)		(1UL << ((nr) % BITS_PER_LONG))
#endif 
#ifndef BIT_WORD
/* BIT_WORD() - former LONG() */
#define BIT_WORD(nr)		((nr) / BITS_PER_LONG)
#endif


#ifdef __cplusplus
extern "C" {
#endif

/* main lsadrv device data */
struct lsadrv_device
{
	/* Pointer to our usb_device */
	struct usb_device *udev;

	/* link to device list */
	struct list_head device_list;
   
	int unplugged;		/* set when the plug is pulled */
	int usage;		/* number of i/o in progress */

	/* isochronous stream stuff */
	int iso_claim;
	int iso_init;
	struct lsadrv_iso_stream_object *stream;
	spinlock_t	*streamLock;
	int StopIsoStream;
	int CancelIsoStream;
	int statusStreamStopReason;
	int LastFailedUrbStatus;
	int LastFailedStreamUrbStatus;
   
	struct semaphore modlock;
	/*** Misc. data ***/
	wait_queue_head_t remove_ok;		/* When we got hot unplugged, we have to avoid a few race conditions */
};

/* lsadrv input device data */
struct lsadrv_input_dev
{
	struct input_dev*	idev;
	char			phys_path[64];
	int			open;	/* open count */
	int			mouse_data[4];
};

/* Global variables */
extern int lsadrv_trace;

/* functions defined in lsadrv-ioctl.c */
int lsadrv_usb_ioctl(struct lsadrv_device *xdev, unsigned int cmd, void *arg);

/* functions defined in lsadrv-isoc.c */
int lsadrv_start_iso_stream(
	struct lsadrv_device *xdev,
	unsigned int ep,
	unsigned int PacketSize,
	unsigned int PacketCount,
	unsigned int FramesPerBuffer,
	unsigned int BufferCount);
int lsadrv_stop_iso_stream(struct lsadrv_device *xdev);
int lsadrv_read_iso_buffer(
	struct lsadrv_device *xdev,
	unsigned int   PacketCount,
	unsigned int   PacketSize,
	unsigned char* dataBuffer,
	unsigned int*  pBytesRead,
	signed long    timeout);		/* jiffies */
void lsadrv_isoc_handler(void *context, int status);

/* functions defined in lsadrv-vkey.c */
int lsadrv_get_key_list(const int **list);

/* functions defined in lsadrv-sub.c */
void lsadrv_printk(const char *fmt, ...)
	__attribute__ ((format (printf, 1, 2)));
void lsadrv_free(const void *);
void *lsadrv_malloc(size_t);
void lsadrv_set_current_state(int state);
void lsadrv_schedule(void);
signed long lsadrv_schedule_timeout(signed long timeout);
signed long lsadrv_msec_to_jiffies(__u32 msec);
void lsadrv_init_waitqueue_head(wait_queue_head_t **q);
void lsadrv_free_waitqueue_head(wait_queue_head_t *q);
void lsadrv_init_waitqueue_entry(void *buf, int size);
void lsadrv_add_wait_queue(wait_queue_head_t *q, wait_queue_t *wait);
void lsadrv_remove_wait_queue(wait_queue_head_t *q, wait_queue_t *wait);
void lsadrv_wake_up_interruptible(wait_queue_head_t *q);
void lsadrv_modlock(struct lsadrv_device *xdev);
void lsadrv_modunlock(struct lsadrv_device *xdev);
void lsadrv_spin_lock_init(spinlock_t **lock);
void lsadrv_spin_lock_term(spinlock_t *lock);
void lsadrv_spin_lock(spinlock_t *lock, unsigned long *flags);
void lsadrv_spin_unlock(spinlock_t *lock, unsigned long *flags);
int lsadrv_write_ok(void *addr, unsigned long size);
unsigned long lsadrv_copy_to_user(void *, const void *, unsigned long);
unsigned long lsadrv_copy_from_user(void *, const void *, unsigned long);
pid_t lsadrv_getpgrp(pid_t *pidp);
struct task_struct *lsadrv_find_task_by_pid(int pid);
void lsadrv_input_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);
void lsadrv_input_sync(struct input_dev *dev);
void lsadrv_input_report_key(struct input_dev *dev, unsigned int code, int value);
void lsadrv_input_report_abs(struct input_dev *dev, unsigned int code, int value);
void lsadrv_input_report_rel(struct input_dev *dev, unsigned int code, int value);
struct urb *lsadrv_usb_alloc_urb(int iso_packets);
void lsadrv_fill_isoc_urb(
	struct urb *urb,
	struct usb_device *dev,
	unsigned int pipe,
	void *context,
	void *buffer,
	unsigned int num_packets,
	unsigned int packet_size,
	unsigned int buffer_inc);	// buffer address increment per packet
void lsadrv_get_isoc_desc(struct urb *urb, unsigned int idx, unsigned int *status, unsigned int *actual_length);
void lsadrv_usb_free_urb (struct urb *urb);
int lsadrv_usb_submit_urb(struct urb *urb);
int lsadrv_usb_resubmit_urb(struct urb *urb, struct usb_device *dev);
int lsadrv_usb_unlink_urb(struct urb *urb);
int lsadrv_usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe, void *data, int len, int *actual_length, int timeout);
int lsadrv_usb_control_msg(struct usb_device *dev, unsigned int pipe, __u8 request, __u8 requesttype, __u16 value, __u16 index, void *data, __u16 size, int timeout);
int lsadrv_usb_reset_device(struct usb_device *dev);
int lsadrv_usb_set_interface(struct usb_device *dev, int ifnum, int alternate);
int lsadrv_usb_clear_halt(struct usb_device *dev, int pipe);
int lsadrv_usb_get_current_frame_number (struct usb_device *usb_dev);
int lsadrv_usb_check_epnum(struct usb_device *dev, unsigned epnum);
void lsadrv_get_device_descriptor(struct usb_device *dev, struct usb_device_descriptor *desc);
int lsadrv_get_configuration_descriptor(struct usb_device *dev, void *buf, int size);
int lsadrv_usb_maxpacket(struct usb_device *dev, unsigned int pipe, int out);
int lsadrv_resetpipe(struct usb_device *dev, unsigned int ep);
unsigned int lsadrv_usb_sndctrlpipe(struct usb_device *dev, unsigned int ep);
unsigned int lsadrv_usb_rcvctrlpipe(struct usb_device *dev, unsigned int ep);
unsigned int lsadrv_usb_sndbulkpipe(struct usb_device *dev, unsigned int ep);
unsigned int lsadrv_usb_rcvbulkpipe(struct usb_device *dev, unsigned int ep);
unsigned int lsadrv_usb_rcvisocpipe(struct usb_device *dev, unsigned int ep);
int lsadrv_get_pipe_info(struct usb_device *dev, struct lsadrv_interface_info *info);
int lsadrv_check_recip(struct usb_device *dev, unsigned int requesttype, unsigned int index);

#ifdef __cplusplus
}
#endif


#endif
