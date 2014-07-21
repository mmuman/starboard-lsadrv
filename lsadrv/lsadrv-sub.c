/*==========================================================================
 * lsadrv-sub.c : Linux kernel driver for eIT-Xiroku optical touch sensor
 *
 * Copyright (C) 2009  eIT Co., Ltd. and Xiroku Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
============================================================================*/

#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/kernel.h> 	/* for linux kernel */
#include <linux/slab.h>
#include <linux/version.h>
#include <asm/uaccess.h>

#include <linux/sched.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 22)) & (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31))
#define find_task_by_pid(pid) find_task_by_pid_type_ns(PIDTYPE_PID, pid, &init_pid_ns)
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 30)
#define find_task_by_pid(pid) pid_task(find_pid_ns((pid_t)pid, &init_pid_ns), PIDTYPE_PID)
#endif

#include "lsadrv.h"
#include "lsadrv-ioctl.h"

#define MAXLINE	256
#define OFFSETOF(member, type)      ((size_t) &((type *)0)->member)
#define usb_settoggle(dev, ep, out, bit) \
                ((dev)->toggle[out] = ((dev)->toggle[out] & ~(1 << (ep))) | \
                 ((bit) << (ep)))

void lsadrv_printk(const char *fmt, ...)
{
	va_list	arglist;
	char buf[MAXLINE];

	va_start(arglist, fmt);
	vsnprintf(buf, sizeof(buf), fmt, arglist);
	va_end(arglist);
	printk("%s", buf);
}

void lsadrv_free(const void *p)
{
	kfree(p);
}

void *lsadrv_malloc(size_t n)
{
	return kmalloc(n, GFP_KERNEL);
}

void lsadrv_set_current_state(int state)
{
	set_current_state(state);
}

void lsadrv_schedule(void)
{
	schedule();
}

signed long lsadrv_schedule_timeout(signed long timeout)
{
	return schedule_timeout(timeout);
}

/* convert timeout from msec to jiffies */
signed long lsadrv_msec_to_jiffies(__u32 msec)
{
	signed long jiff;
	if (msec == (__u32)(-1)) {
		jiff = MAX_SCHEDULE_TIMEOUT;
	}
	else {
#if 0
		jiff = (long) (((__u64) msec * HZ + 999) / 1000);
#else
		if (msec < (ULONG_MAX - 999) / HZ) {
			jiff = (long) ((msec * HZ + 999) / 1000);
		}
		else {
			jiff = (long) (msec / (1000/HZ));
		}
#endif
	}
	return jiff;
}

void lsadrv_init_waitqueue_head(wait_queue_head_t **q)
{
	*q = kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
	if (*q) {
		init_waitqueue_head(*q);
	}
}

void lsadrv_free_waitqueue_head(wait_queue_head_t *q)
{
	if (q) {
		kfree(q);
	}
}

void lsadrv_init_waitqueue_entry(void *buf, int size)
{
	wait_queue_t *wait = (wait_queue_t *) buf;
	if (size < sizeof(wait_queue_t)) {
		BUG();
	}
	memset(wait, 0, sizeof(wait_queue_t));
	init_waitqueue_entry(wait, current);
}

void lsadrv_add_wait_queue(wait_queue_head_t *q, wait_queue_t *wait)
{
	add_wait_queue(q, wait);
}

void lsadrv_remove_wait_queue(wait_queue_head_t *q, wait_queue_t *wait)
{
	remove_wait_queue(q, wait);
}

void lsadrv_wake_up_interruptible(wait_queue_head_t *q)
{
	wake_up_interruptible(q);
}

void lsadrv_modlock(struct lsadrv_device *xdev)
{
	down(&xdev->modlock);
}

void lsadrv_modunlock(struct lsadrv_device *xdev)
{
	up(&xdev->modlock);
}

void lsadrv_spin_lock_init(spinlock_t **lock)
{
	*lock = kmalloc(sizeof(spinlock_t), GFP_KERNEL);
	if (*lock) {
		spin_lock_init(*lock);
	}
}

void lsadrv_spin_lock_term(spinlock_t *lock)
{
	if (lock) {
		kfree(lock);
	}
}

void lsadrv_spin_lock(spinlock_t *lock, unsigned long *flags)
{
	spin_lock_irqsave(lock, *flags);
}

void lsadrv_spin_unlock(spinlock_t *lock, unsigned long *flags)
{
	spin_unlock_irqrestore(lock, *flags);
}

int lsadrv_write_ok(void *addr, unsigned long size)
{
	return access_ok(VERIFY_WRITE, addr, size);
}

unsigned long lsadrv_copy_to_user(void *to, const void *from, unsigned long n)
{
	return copy_to_user(to, from, n);
}

unsigned long lsadrv_copy_from_user(void *to, const void *from, unsigned long n)
{
	return copy_from_user(to, from, n);
}

pid_t lsadrv_getpgrp(pid_t *pidp)
{
	if (pidp)	*pidp  = current->pid;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
	return process_group(current);
#else
	return task_pgrp_nr(current);
#endif
}

struct task_struct *
lsadrv_find_task_by_pid(int pid)
{
	return find_task_by_pid(pid);
}

void lsadrv_input_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	input_event(dev, type, code, value);
}

void lsadrv_input_sync(struct input_dev *dev)
{
	input_sync(dev);
}

void lsadrv_input_report_key(struct input_dev *dev, unsigned int code, int value)
{
	input_report_key(dev, code, value);
}

void lsadrv_input_report_abs(struct input_dev *dev, unsigned int code, int value)
{
	input_report_abs(dev, code, value);
}

void lsadrv_input_report_rel(struct input_dev *dev, unsigned int code, int value)
{
	input_report_rel(dev, code, value);
}

struct urb *lsadrv_usb_alloc_urb(int iso_packets)
{
	return usb_alloc_urb(iso_packets, GFP_KERNEL);
}

/*
 * Isochronous transfer urb completion routine
 */
static void
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
lsadrv_isoc_complete(struct urb *urb)
#else
lsadrv_isoc_complete(struct urb *urb, struct pt_regs *regs)
#endif
{
	lsadrv_isoc_handler(urb->context, urb->status);
}

void lsadrv_fill_isoc_urb(
	struct urb *urb,
	struct usb_device *dev,
	unsigned int pipe,
	void *context,
	void *buffer,
	unsigned int num_packets,
	unsigned int packet_size,
	unsigned int buffer_inc)	// buffer address increment per packet
{
	unsigned int buffer_length = buffer_inc * num_packets;
	int j;

	urb->dev = dev;
        urb->pipe = pipe;
	urb->transfer_flags = URB_ISO_ASAP;
        urb->transfer_buffer = buffer;
        urb->transfer_buffer_length = buffer_length;
        urb->complete = lsadrv_isoc_complete;
        urb->context = context;
	urb->start_frame = 0;
	urb->number_of_packets = num_packets;
	for (j = 0; j < num_packets; j++) {
		urb->iso_frame_desc[j].offset = j * buffer_inc;
		urb->iso_frame_desc[j].length = packet_size;
	}
	urb->interval = 1;
}

void lsadrv_get_isoc_desc(struct urb *urb, unsigned int idx, unsigned int *status, unsigned int *actual_length)
{
	*status = urb->iso_frame_desc[idx].status;
	*actual_length = urb->iso_frame_desc[idx].actual_length;
}

void lsadrv_usb_free_urb(struct urb *urb)
{
	usb_free_urb(urb);
}

int lsadrv_usb_submit_urb(struct urb *urb)
{
	return usb_submit_urb(urb, GFP_ATOMIC);
}

int lsadrv_usb_resubmit_urb(struct urb *urb, struct usb_device *dev)
{
	urb->dev = dev;
	return usb_submit_urb(urb, GFP_ATOMIC);
}

int lsadrv_usb_unlink_urb(struct urb *urb)
{
	return usb_unlink_urb(urb);
}

int lsadrv_usb_bulk_msg(struct usb_device *dev, unsigned int pipe, void *data, int len, int *actlen, int timeout)
{
	return usb_bulk_msg(dev, pipe, data, len, actlen, timeout);
}

int lsadrv_usb_control_msg(struct usb_device *dev, unsigned int pipe, __u8 request, __u8 requesttype, __u16 value, __u16 index, void *data, __u16 size, int timeout)
{
	return usb_control_msg(dev, pipe, request, requesttype, value, index, data, size, timeout);
}

int lsadrv_usb_reset_device(struct usb_device *dev)
{
	return usb_reset_device(dev);
}

int lsadrv_usb_set_interface(struct usb_device *dev, int ifnum, int alt)
{
	return usb_set_interface(dev, ifnum, alt);
}

int lsadrv_usb_clear_halt(struct usb_device *dev, int pipe)
{
	return usb_clear_halt(dev, pipe);
}

int lsadrv_usb_get_current_frame_number(struct usb_device *dev)
{
	return usb_get_current_frame_number(dev);
}

int lsadrv_usb_check_epnum(struct usb_device *dev, unsigned epnum)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
	struct usb_endpoint_descriptor *ep;
	if (epnum & ~0x8f) {
		return -1;
	}
	ep = usb_epnum_to_ep_desc(dev, epnum);
	if (ep == NULL) {
		return -1;
	}
#else
	struct usb_host_endpoint *ep;
	if (epnum & ~0x8f) {
		return -1;
	}
	if (epnum & 0x80) {
		ep = dev->ep_in[epnum & 0x0f];
	}
	else {
		ep = dev->ep_out[epnum];
	}
	if (ep == NULL) {
		return -1;
	}
#endif
	return 0;
}

void lsadrv_get_device_descriptor(struct usb_device *dev, struct usb_device_descriptor *desc)
{
	memcpy(desc, &dev->descriptor, sizeof(struct usb_device_descriptor));
}

int lsadrv_get_configuration_descriptor(struct usb_device *dev, void *buf, int size)
{
 	struct usb_host_config *config = dev->actconfig;
	int cfgno;
	if (!config) {
		return -ENODEV;		/* device is not configured */
	}
	if (size > config->desc.wTotalLength) {
		size = config->desc.wTotalLength;
	}
	/* search cfgno */
	cfgno = config - &dev->config[0];
	memcpy(buf, dev->rawdescriptors[cfgno], size);
	return 0;
}

int lsadrv_usb_maxpacket(struct usb_device *dev, unsigned int pipe, int out)
{
	return usb_maxpacket(dev, pipe, out);
}

int lsadrv_resetpipe(struct usb_device *dev, unsigned int epnum)
{ 					/* ep: endpoint address + direction */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
	struct usb_endpoint_descriptor *ep;
	/* check validity of endpoint number */
	if (epnum & ~(USB_DIR_IN|0xf)) {
		return -EINVAL;
	}
	ep = usb_epnum_to_ep_desc(dev, epnum);
	if (ep == NULL) {
		return -ENOENT;
	}
#else
	struct usb_host_endpoint *ep;
	/* check validity of endpoint number */
	if (epnum & ~(USB_DIR_IN|0xf)) {
		return -EINVAL;
	}
	if (epnum & USB_DIR_IN) {
		ep = dev->ep_in[epnum & 0x0f];
	}
	else {
		ep = dev->ep_out[epnum];
	}
	if (ep == NULL) {
		return -ENOENT; 
	}
#endif
	usb_settoggle(dev, (epnum & 0xf), !(epnum & USB_DIR_IN), 0);
	return 0;
}

unsigned int lsadrv_usb_sndctrlpipe(struct usb_device *dev, unsigned int ep)
{
	return usb_sndctrlpipe(dev, ep);
}
unsigned int lsadrv_usb_rcvctrlpipe(struct usb_device *dev, unsigned int ep)
{
	return usb_rcvctrlpipe(dev, ep);
}
unsigned int lsadrv_usb_sndbulkpipe(struct usb_device *dev, unsigned int ep)
{
	return usb_sndbulkpipe(dev, ep);
}
unsigned int lsadrv_usb_rcvbulkpipe(struct usb_device *dev, unsigned int ep)
{
	return usb_rcvbulkpipe(dev, ep);
}
unsigned int lsadrv_usb_rcvisocpipe(struct usb_device *dev, unsigned int ep)
{
	return usb_rcvisocpipe(dev, ep);
}

/* get pipe information in current setting */
int lsadrv_get_pipe_info(struct usb_device *dev, struct lsadrv_interface_info *info)
{
	struct usb_interface *interface;
	struct usb_interface_descriptor *intf;
	int n, i;

	interface = usb_ifnum_to_if(dev, 0);	/* interface 0 */
	if (!interface || !interface->cur_altsetting) {
		return -ENODEV;		/* device is not configured */
	}
	intf = &interface->cur_altsetting->desc;
	n = intf->bNumEndpoints;
	info->wLength = OFFSETOF(Pipes[n], struct lsadrv_interface_info);	/* OUT: length of valid data */
	info->bInterfaceNumber   = intf->bInterfaceNumber;
	info->bAlternateSetting  = intf->bAlternateSetting;
	info->bInterfaceClass    = intf->bInterfaceClass;
	info->bInterfaceSubClass = intf->bInterfaceSubClass;
	info->bInterfaceProtocol = intf->bInterfaceProtocol;
	info->bNumEndpoints      = intf->bNumEndpoints;

	for (i = 0; i < n; i++) {
  		struct usb_endpoint_descriptor *ep = &interface->cur_altsetting->endpoint[i].desc;
		struct lsadrv_pipe_info *pp = &info->Pipes[i];
		pp->bEndpointAddress	= ep->bEndpointAddress;
		pp->bmAttributes	= ep->bmAttributes;
		pp->wMaxPacketSize	= ep->wMaxPacketSize;
		pp->bInterval		= ep->bInterval;
		pp->bRefresh		= ep->bRefresh;
		pp->bSynchAddress	= ep->bSynchAddress;
		pp->bReserved		= 0;
	}

	return 0;
}

/* check validity of index for standard/class request */
int lsadrv_check_recip(struct usb_device *dev, unsigned int requesttype, unsigned int index)
{
	struct usb_interface *intf;

	/* vendor request is always OK */
	if (USB_TYPE_VENDOR == (USB_TYPE_MASK & requesttype))
		return 0;

	switch (requesttype & USB_RECIP_MASK) {
	case USB_RECIP_INTERFACE:
		if (index > 0xff) {
			return -EINVAL;
		}
		intf = usb_ifnum_to_if(dev, index);
		if (intf == NULL) {
			return -ENOENT; 
		}
		break;

	case USB_RECIP_ENDPOINT:
		if (index & ~(USB_DIR_IN|0xf)) {
			return -EINVAL;
		}
		else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
			struct usb_endpoint_descriptor *ep;
			ep = usb_epnum_to_ep_desc(dev, index);
#else
			struct usb_host_endpoint *ep;
			if (index & USB_DIR_IN) {
				ep = dev->ep_in[index & 0x0f];
			}
			else {
				ep = dev->ep_out[index];
			}
#endif
			if (ep == NULL) {
				return -ENOENT; 
			}
		}
		break;
	}

	return 0;
}

