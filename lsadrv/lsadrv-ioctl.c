/*==========================================================================
 * lsadrv-ioctl.c : Linux driver for eIT-Xiroku optical touch sensor
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

#include <linux/compat.h> 	/* for 32bit compatibility */
#include <linux/slab.h>		/* for kmalloc */

#include "lsadrv.h"
#include "lsadrv-ioctl.h"
#include "lsadrv-vkey.h"

extern struct lsadrv_input_dev *lsadrv_idev;

/******** static functions and variables ********/
static int lsadrv_ioctl_get_driver_version(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_mouseevent(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_keybdevent(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_get_device_descriptor(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_get_configuration_descriptor(struct lsadrv_device *xdev, void *arg, int size);
static int lsadrv_ioctl_get_pipe_info(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_abortpipe(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_control(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_bulk(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_resetpipe(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_clear_halt(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_setinterface(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_reset(struct lsadrv_device *xdev);
static int lsadrv_ioctl_start_iso_stream(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_stop_iso_stream(struct lsadrv_device *xdev);
static int lsadrv_ioctl_read_iso_buffer(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_claim_stream(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_check(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_get_last_error(struct lsadrv_device *xdev, void *arg);
static int lsadrv_ioctl_get_current_frame_number(struct lsadrv_device *xdev, void *arg);

#ifdef CONFIG_COMPAT

/***************************************************************************/
/* 32bit compatibility */

struct compat_lsadrv_bulk_transfer_control
{
	unsigned int ep;
	unsigned int len;
	unsigned int timeout; /* in milliseconds */
	compat_caddr_t data; /* (void *) */
} __attribute__ ((packed));

struct compat_lsadrv_control_transfer_control
{
	u_int8_t  requesttype;
	u_int8_t  request;
	u_int16_t value;
	u_int16_t index;
	u_int16_t length;
	u_int32_t timeout;  /* in milliseconds */
	compat_caddr_t data; /* (void *) */
} __attribute__ ((packed));

struct compat_lsadrv_iso_read_control
{
	unsigned int PacketSize;
	unsigned int PacketCount;
	/* Timeout for reading ISO buffer (msec) */
	unsigned int Timeout;
	compat_caddr_t buffer; /* (unsigned char *) */
	unsigned int  bufferSize;	/* IN: buffer sizer */
		/* buffer size = (PacketSize + sizeof(struct lsadrv_iso_packet_desc)) * PacketCount */
} __attribute__ ((packed));

#define LSADRV_IOC_CONTROL32			_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 8, \
							struct compat_lsadrv_control_transfer_control)

#define LSADRV_IOC_BULK32				_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 9, \
							struct compat_lsadrv_bulk_transfer_control)

#define LSADRV_IOC_READ_ISO_BUFFER32	_IOW(LSADRV_IOC_MAGIC, \
							LSADRV_IOCTL_BASE + 18, \
							struct compat_lsadrv_iso_read_control)

#endif /* CONFIG_COMPAT */

/***************************************************************************/
/* Private functions */

/* usb ioctl - called through devio. So, copy from/to user is done outside */
int lsadrv_usb_ioctl(struct lsadrv_device *xdev, unsigned int cmd, void *arg)
{
	int ret = 0;
	void *karg = NULL;

	lsadrv_modlock(xdev);
	if (xdev->unplugged) {
		lsadrv_modunlock(xdev);
		ret = -ENODEV;
		goto l_ret;
	}
	lsadrv_modunlock(xdev);

	switch (cmd) {
		/* get driver version */
		case LSADRV_IOC_GET_DRIVER_VERSION:
			ret = lsadrv_ioctl_get_driver_version(xdev, arg);
			break;

		/* send mouse input event */
		case LSADRV_IOC_MOUSEEVENT:
			ret = lsadrv_ioctl_mouseevent(xdev, arg);
			break;

		/* get device descriptor */
		case LSADRV_IOC_GET_DEVICE_DESCRIPTOR:
			ret = lsadrv_ioctl_get_device_descriptor(xdev, arg);
			break;

		/* get pipe information in current setting */
		case LSADRV_IOC_GET_PIPE_INFO:
			ret = lsadrv_ioctl_get_pipe_info(xdev, arg);
			break;

		/* abort an endpoint */
		case LSADRV_IOC_ABORTPIPE:
			ret = lsadrv_ioctl_abortpipe(xdev, arg);
			break;

		/*** following 5 ioctls have alternatives in devio, though... ***/
		/* vendor or class request */
		case LSADRV_IOC_CONTROL:
			ret = lsadrv_ioctl_control(xdev, arg);
			break;

		/* perform an IN/OUT transfer over the specified bulk or interrupt pipe */
		case LSADRV_IOC_BULK:
			ret = lsadrv_ioctl_bulk(xdev, arg);
			break;

		/* reset an endpoint */
		case LSADRV_IOC_RESETPIPE:
			ret = lsadrv_ioctl_resetpipe(xdev, arg);
			break;

		/* clear halt status of an endpoint */
		case LSADRV_IOC_CLEAR_HALT:
			ret = lsadrv_ioctl_clear_halt(xdev, arg);
			break;

		/* set configuration and alternative interface */
		case LSADRV_IOC_SETINTERFACE:
			ret = lsadrv_ioctl_setinterface(xdev, arg);
			break;

		/* bus reset */
		case LSADRV_IOC_RESET:
			ret = lsadrv_ioctl_reset(xdev);
			break;

		/* start isochronous stream */
		case LSADRV_IOC_START_ISO_STREAM:
			ret = lsadrv_ioctl_start_iso_stream(xdev, arg);
			break;

		/* stop isochronous stream */
		case LSADRV_IOC_STOP_ISO_STREAM:
			ret = lsadrv_ioctl_stop_iso_stream(xdev);
			break;

		/* read data from isochronous stream data buffer */
		case LSADRV_IOC_READ_ISO_BUFFER:
			ret = lsadrv_ioctl_read_iso_buffer(xdev, arg);
			break;

		/* claim/unclaim device for use of isochronous stream channel */
		case LSADRV_IOC_CLAIM_STREAM:
			ret = lsadrv_ioctl_claim_stream(xdev, arg);
			break;

		/* check if the device is ready */
		case LSADRV_IOC_CHECK:
			ret = lsadrv_ioctl_check(xdev, arg);
			break;

		/* get usb error information */
		case LSADRV_IOC_GET_LAST_ERROR:
			ret = lsadrv_ioctl_get_last_error(xdev, arg);
			break;

		/* retrieve the current USB frame number from the host controller */
		case LSADRV_IOC_GET_CURRENT_FRAME_NUMBER:
			ret = lsadrv_ioctl_get_current_frame_number(xdev, arg);
			break;

		/* send keyboard input event */
		case LSADRV_IOC_KEYBDEVENT:
			ret = lsadrv_ioctl_keybdevent(xdev, arg);
			break;

#ifdef CONFIG_COMPAT
		/* 32bit compatibility */
		/* no need for get_user/put_user here */

		/* vendor or class request */
		case LSADRV_IOC_CONTROL32:
		{
			struct compat_lsadrv_control_transfer_control *ua32 = arg;
			struct lsadrv_control_transfer_control *a;

			Trace(LSADRV_TRACE_IOCTL, "LSADRV_IOC_CONTROL32\n");
			a = karg = kmalloc(sizeof(*a), GFP_KERNEL);
			if (!karg)
				return -ENOMEM;

			a->requesttype = ua32->requesttype;
			a->request = ua32->request;
			a->value = ua32->value;
			a->index = ua32->index;
			a->length = ua32->length;
			a->timeout = ua32->timeout;
			a->data = compat_ptr(ua32->data);

			ret = lsadrv_ioctl_control(xdev, a);
			break;
		}

		/* perform an IN/OUT transfer over the specified bulk or interrupt pipe */
		case LSADRV_IOC_BULK32:
		{
			struct compat_lsadrv_bulk_transfer_control *ua32 = arg;
			struct lsadrv_bulk_transfer_control *a;

			Trace(LSADRV_TRACE_IOCTL, "LSADRV_IOC_BULK32\n");
			a = karg = kmalloc(sizeof(*a), GFP_KERNEL);
			if (!karg)
				return -ENOMEM;

			a->ep = ua32->ep;
			a->len = ua32->len;
			a->timeout = ua32->timeout;
			a->data = compat_ptr(ua32->data);

			ret = lsadrv_ioctl_bulk(xdev, a);
			break;
		}

		/* read data from isochronous stream data buffer */
		case LSADRV_IOC_READ_ISO_BUFFER32:
		{
			struct compat_lsadrv_iso_read_control *ua32 = arg;
			struct lsadrv_iso_read_control *a;

			Trace(LSADRV_TRACE_IOCTL, "LSADRV_IOC_READ_ISO_BUFFER32\n");
			a = karg = kmalloc(sizeof(*a), GFP_KERNEL);
			if (!karg)
				return -ENOMEM;

			a->PacketSize = ua32->PacketSize;
			a->PacketCount = ua32->PacketCount;
			a->Timeout = ua32->Timeout;
			a->buffer = compat_ptr(ua32->buffer);
			a->bufferSize = ua32->bufferSize;

			ret = lsadrv_ioctl_read_iso_buffer(xdev, a);
			break;
		}

#endif /* CONFIG_COMPAT */

		default:
			/* get configuration descriptor */
			if ((cmd & ~IOCSIZE_MASK) == LSADRV_IOC_GET_CONFIGURATION_DESCRIPTOR(0)) {
				ret = lsadrv_ioctl_get_configuration_descriptor(xdev, arg, _IOC_SIZE(cmd));
				break;
			}
			Warning("invalid ioctl: 0x%x\n", cmd);
			ret = -EINVAL;
			break;
	} /* ..switch (cmd) */

	kfree(karg);

l_ret:
	return ret;
}

/* get driver version */
static int lsadrv_ioctl_get_driver_version(struct lsadrv_device *xdev, void *arg)
{
	struct lsadrv_driver_version *ver = (struct lsadrv_driver_version *) arg;
	ver->MajorVersion = LSADRV_KDRIVER_MAJOR;
	ver->MinorVersion = LSADRV_KDRIVER_MINOR;
	ver->BuildVersion = LSADRV_KDRIVER_BUILD;
	return 0;
}

#if 1
#ifdef PREVENT_MOUSE_DRIVER_MATCH
#define BTN_LEFT	BTN_TOOL_PEN
#endif //PREVENT_MOUSE_DRIVER_MATCH
/* send mouse input event */
int lsadrv_ioctl_mouseevent_dispatch(void *arg);

static int lsadrv_ioctl_mouseevent(struct lsadrv_device *xdev, void *arg)
{
  return lsadrv_ioctl_mouseevent_dispatch(arg);
}

int lsadrv_ioctl_mouseevent_dispatch(void *arg)
{
	struct lsadrv_mouse_input *inp = (struct lsadrv_mouse_input*)arg;
	struct lsadrv_input_dev *xidev = lsadrv_idev;
	struct input_dev *idev = xidev->idev;
	int dx = 0, dy = 0;

	Trace(LSADRV_TRACE_MOUSE, "LSADRV_IOC_MOUSEEVENT: (%d,%d,0x%x)\n", inp->dx, inp->dy, inp->flags);

	if (idev == NULL) {
		Err("idev is NULL\n");
		return -EFAULT;
	}

#ifndef PREVENT_MOUSE_DRIVER_MATCH
	lsadrv_input_report_key(idev, BTN_TOUCH, 1);
#endif //PREVENT_MOUSE_DRIVER_MATCH

	if (inp->flags & MOUSEEVENTF_MOVE) {
		if (inp->flags & MOUSEEVENTF_ABSOLUTE) {
			xidev->mouse_data[1] = inp->dx;
			xidev->mouse_data[2] = inp->dy;
		}
		else {
			dx = inp->dx;
			dy = inp->dy;
			xidev->mouse_data[1] += dx;
			xidev->mouse_data[2] += dy;
		}
		if (xidev->mouse_data[1] < 0)		xidev->mouse_data[1] = 0;
		else if (xidev->mouse_data[1] > 0xffff)	xidev->mouse_data[1] = 0xffff;
		if (xidev->mouse_data[2] < 0)		xidev->mouse_data[2] = 0;
		else if (xidev->mouse_data[2] > 0xffff)	xidev->mouse_data[2] = 0xffff;

		if (inp->flags & MOUSEEVENTF_ABSOLUTE) {
			lsadrv_input_report_abs(idev, ABS_X, xidev->mouse_data[1]);
			lsadrv_input_report_abs(idev, ABS_Y, xidev->mouse_data[2]);
		}
		else {
			lsadrv_input_report_rel(idev, REL_X, dx);
			lsadrv_input_report_rel(idev, REL_Y, dy);
		}
	}

	if (inp->flags & MOUSEEVENTF_LEFTDOWN) {
		xidev->mouse_data[0] |= 1;
		lsadrv_input_report_key(idev, BTN_LEFT, 1);
	}
	if (inp->flags & MOUSEEVENTF_LEFTUP) {
		xidev->mouse_data[0] &= ~1;
		lsadrv_input_report_key(idev, BTN_LEFT, 0);
	}

	if (inp->flags & MOUSEEVENTF_RIGHTDOWN) {
		xidev->mouse_data[0] |= 2;
		lsadrv_input_report_key(idev, BTN_RIGHT, 1);
	}
	if (inp->flags & MOUSEEVENTF_RIGHTUP) {
		xidev->mouse_data[0] &= ~2;
		lsadrv_input_report_key(idev, BTN_RIGHT, 0);
	}

	if (inp->flags & MOUSEEVENTF_MIDDLEDOWN) {
		xidev->mouse_data[0] |= 4;
		lsadrv_input_report_key(idev, BTN_MIDDLE, 1);
	}
	if (inp->flags & MOUSEEVENTF_MIDDLEUP) {
		xidev->mouse_data[0] &= ~4;
		lsadrv_input_report_key(idev, BTN_MIDDLE, 0);
	}

    	//lsadrv_input_event(idev, EV_MSC, MSC_SERIAL, 0);
    	lsadrv_input_sync(idev);
	return 0;
}
#else /*0*/
/* send mouse input event */
static int lsadrv_ioctl_mouseevent(struct lsadrv_device *xdev, void *arg)
{
	struct lsadrv_mouse_input *inp = (struct lsadrv_mouse_input*)arg;
	struct lsadrv_input_dev *xidev = lsadrv_idev;
	struct input_dev *idev = xidev->idev;
	int dx = 0, dy = 0;

	Trace(LSADRV_TRACE_MOUSE, "LSADRV_IOC_MOUSEEVENT: (%d,%d,0x%x)\n", inp->dx, inp->dy, inp->flags);

	if (idev == NULL) {
		Err("idev is NULL\n");
		return -EFAULT;
	}

	lsadrv_input_report_key(idev, BTN_TOUCH, 1);
	if (inp->flags & MOUSEEVENTF_MOVE) {
		if (inp->flags & MOUSEEVENTF_ABSOLUTE) {
			xidev->mouse_data[1] = inp->dx;
			xidev->mouse_data[2] = inp->dy;
		}
		else {
			dx = inp->dx;
			dy = inp->dy;
			xidev->mouse_data[1] += dx;
			xidev->mouse_data[2] += dy;
		}
		if (xidev->mouse_data[1] < 0)		xidev->mouse_data[1] = 0;
		else if (xidev->mouse_data[1] > 0xffff)	xidev->mouse_data[1] = 0xffff;
		if (xidev->mouse_data[2] < 0)		xidev->mouse_data[2] = 0;
		else if (xidev->mouse_data[2] > 0xffff)	xidev->mouse_data[2] = 0xffff;
	}
	if (inp->flags & MOUSEEVENTF_ABSOLUTE) {
		if (inp->flags & MOUSEEVENTF_MOVE) {
			xidev->mouse_data[1] = inp->dx;
			xidev->mouse_data[2] = inp->dy;
		}
		lsadrv_input_report_abs(idev, ABS_X, xidev->mouse_data[1]);
		lsadrv_input_report_abs(idev, ABS_Y, xidev->mouse_data[2]);
	}
	else {
		if (inp->flags & MOUSEEVENTF_MOVE) {
			dx = inp->dx;
			dy = inp->dy;
			xidev->mouse_data[1] += dx;
			xidev->mouse_data[2] += dy;
			if (xidev->mouse_data[1] < 0)		xidev->mouse_data[1] = 0;
			else if (xidev->mouse_data[1] > 0xffff)	xidev->mouse_data[1] = 0xffff;
			if (xidev->mouse_data[2] < 0)		xidev->mouse_data[2] = 0;
			else if (xidev->mouse_data[2] > 0xffff)	xidev->mouse_data[2] = 0xffff;
		}
		lsadrv_input_report_rel(idev, REL_X, dx);
		lsadrv_input_report_rel(idev, REL_Y, dy);
	}

	if (inp->flags & MOUSEEVENTF_LEFTDOWN) {
		xidev->mouse_data[0] |= 1;
	}
	if (inp->flags & MOUSEEVENTF_LEFTUP) {
		xidev->mouse_data[0] &= ~1;
	}
	lsadrv_input_report_key(idev, BTN_LEFT, xidev->mouse_data[0] & 1);

	if (inp->flags & MOUSEEVENTF_RIGHTDOWN) {
		xidev->mouse_data[0] |= 2;
	}
	if (inp->flags & MOUSEEVENTF_RIGHTUP) {
		xidev->mouse_data[0] &= ~2;
	}
	lsadrv_input_report_key(idev, BTN_RIGHT, xidev->mouse_data[0] & 2);

	if (inp->flags & MOUSEEVENTF_MIDDLEDOWN) {
		xidev->mouse_data[0] |= 4;
	}
	if (inp->flags & MOUSEEVENTF_MIDDLEUP) {
		xidev->mouse_data[0] &= ~4;
	}
	lsadrv_input_report_key(idev, BTN_MIDDLE, xidev->mouse_data[0] & 4);

    	//lsadrv_input_event(idev, EV_MSC, MSC_SERIAL, 0);
    	lsadrv_input_sync(idev);
	return 0;
}
#endif /*0*/

/* send keyboard input event */
static int lsadrv_ioctl_keybdevent(struct lsadrv_device *xdev, void *arg)
{
	struct lsadrv_keybd_input *inp = (struct lsadrv_keybd_input*)arg;
	struct lsadrv_input_dev *xidev = lsadrv_idev;
	struct input_dev *idev = xidev->idev;
	int key = KEY_ESC;	//dummy

	Trace(LSADRV_TRACE_MOUSE, "LSADRV_IOC_KEYBDEVENT: (vk=0x%x,ext=%d,%s)\n",
		inp->vkey, (inp->flags & KEYEVENTF_EXTENDEDKEY),
		(inp->flags & KEYEVENTF_KEYUP) ? "off" : "on");

	if (idev == NULL) {
		Err("idev is NULL\n");
		return -EFAULT;
	}

	if (inp->flags & KEYEVENTF_INPUT_KEY) {
		key = inp->vkey;
	}
	else {
		key = lsadrv_vkeytokey(inp->vkey, (inp->flags & KEYEVENTF_EXTENDEDKEY));
	}

	if (inp->flags & KEYEVENTF_KEYUP) {
		lsadrv_input_report_key(idev, key, 0);
	}
	else {
		lsadrv_input_report_key(idev, key, 1);
	}
    	//lsadrv_input_event(idev, EV_MSC, MSC_SERIAL, 0);
    	lsadrv_input_sync(idev);
	return 0;
}

/* get device descriptor */
static int lsadrv_ioctl_get_device_descriptor(struct lsadrv_device *xdev, void *arg)
{
	lsadrv_get_device_descriptor(xdev->udev, (struct usb_device_descriptor *) arg);
	return 0;
}

static int lsadrv_ioctl_get_configuration_descriptor(struct lsadrv_device *xdev, void *arg, int size)
{
	Trace(LSADRV_TRACE_IOCTL, "%s: arg=0x%p, size=%d\n", __func__, arg, size);
	return lsadrv_get_configuration_descriptor(xdev->udev, arg, size);
}

 
/* get pipe information in current setting */
static int lsadrv_ioctl_get_pipe_info(struct lsadrv_device *xdev, void *arg)
{
	Trace(LSADRV_TRACE_IOCTL, "%s\n", __func__);
	return lsadrv_get_pipe_info(xdev->udev, (struct lsadrv_interface_info *) arg);
}

/* abort an endpoint */
static int lsadrv_ioctl_abortpipe(struct lsadrv_device *xdev, void *arg)
{
	Trace(LSADRV_TRACE_IOCTL, "%s\n", __func__);
	return 0;
}

/* vendor or class request */
static int lsadrv_ioctl_control(struct lsadrv_device *xdev, void *arg)
{
	struct lsadrv_control_transfer_control *ctrl = (struct lsadrv_control_transfer_control *) arg;
	struct usb_device *udev = xdev->udev;
	unsigned char *tbuf = NULL;
	int ret;
	long timeout;	/* jiffies */
	int i;

	Trace(LSADRV_TRACE_IOCTL, "ioctl_control: [%02x,%02x,%04x,%04x,%04x]\n",
		ctrl->requesttype, ctrl->request,
		ctrl->value, ctrl->index, ctrl->length);

	/* check validity of index for standard/class request */
	if ((ret = lsadrv_check_recip(udev, ctrl->requesttype, ctrl->index))) {
		return ret;
	}

	if (ctrl->length) {
		if (!(tbuf = (unsigned char *)lsadrv_malloc(ctrl->length))) {
			return -ENOMEM;
		}
	}

	timeout = lsadrv_msec_to_jiffies(ctrl->timeout);

	if (ctrl->requesttype & 0x80) {
		/* IN transfer */
		if (ctrl->length && !lsadrv_write_ok(ctrl->data, ctrl->length)) {
			lsadrv_free(tbuf);
			return -EINVAL;
		}
		i = lsadrv_usb_control_msg(udev, lsadrv_usb_rcvctrlpipe(udev, 0), ctrl->request, ctrl->requesttype,
				       ctrl->value, ctrl->index, tbuf, ctrl->length, timeout);
		if (i > 0) {
			if ((ret=lsadrv_copy_to_user(ctrl->data, tbuf, i))) {
				Err("%s: copy_to_user error(%d)", __func__, ret);
				lsadrv_free(tbuf);
				return -EFAULT;
			}
		}
	} else {
		/* OUT transfer */
		if (ctrl->length) {
			if (lsadrv_copy_from_user(tbuf, ctrl->data, ctrl->length)) {
				lsadrv_free(tbuf);
				return -EFAULT;
			}
		}
		i = lsadrv_usb_control_msg(udev, lsadrv_usb_sndctrlpipe(udev, 0), ctrl->request, ctrl->requesttype,
				       ctrl->value, ctrl->index, tbuf, ctrl->length, timeout);
	}
	if (i < 0) {
		xdev->LastFailedUrbStatus = i;
	}
	lsadrv_free(tbuf);
	return i;
}

/* perform an IN/OUT transfer over the specified bulk or interrupt pipe */
static int lsadrv_ioctl_bulk(struct lsadrv_device *xdev, void *arg)
{
	struct lsadrv_bulk_transfer_control *bulk = (struct lsadrv_bulk_transfer_control*) arg;
	struct usb_device *udev = xdev->udev;
	unsigned int pipe;
	unsigned char *tbuf = NULL;
	long timeout;	/* jiffies */
	int i;
	int actlen = 0;
	int ret;

	Trace(LSADRV_TRACE_IOCTL, "ioctl_bulk: ep=%02x, len=%d\n",
		bulk->ep, bulk->len);

	/* check validity of endpoint number */
	if (bulk->ep & ~(USB_DIR_IN|0xf)) {
		return -EINVAL;
	}
	if (lsadrv_usb_check_epnum(udev, bulk->ep)) {
		return -ENOENT; 
	}

	if (bulk->ep & USB_DIR_IN) {
		pipe = lsadrv_usb_rcvbulkpipe(udev, bulk->ep & 0xf);
	}
	else {
		pipe = lsadrv_usb_sndbulkpipe(udev, bulk->ep & 0xf);
	}

	if (!lsadrv_usb_maxpacket(udev, pipe, !(bulk->ep & USB_DIR_IN))) {
		return -EINVAL;
	}

	if (bulk->len) {
		if (!(tbuf = (unsigned char *)lsadrv_malloc(bulk->len))) {
			return -ENOMEM;
		}
	}

	timeout = lsadrv_msec_to_jiffies(bulk->timeout);

	if (bulk->ep & USB_DIR_IN) {
		/* IN transfer */
		if (bulk->len && !lsadrv_write_ok(bulk->data, bulk->len)) {
			lsadrv_free(tbuf);
			return -EINVAL;
		}
		i = lsadrv_usb_bulk_msg(udev, pipe, tbuf, bulk->len, &actlen, timeout);
		if (i == 0 && actlen) {
			if ((ret=lsadrv_copy_to_user(bulk->data, tbuf, actlen))) {
				Err("%s: copy_to_user error(%d)", __func__, ret);
				lsadrv_free(tbuf);
				return -EFAULT;
			}
		}
	} else {
		/* OUT transfer */
		if (bulk->len) {
			if (lsadrv_copy_from_user(tbuf, bulk->data, bulk->len)) {
				lsadrv_free(tbuf);
				return -EFAULT;
			}
		}
		i = lsadrv_usb_bulk_msg(udev, pipe, tbuf, bulk->len, &actlen, timeout);
	}
	lsadrv_free(tbuf);

	if (i < 0) {	/* error */
		xdev->LastFailedUrbStatus = i;
		return i;
	}

	return actlen;
}

/* reset an endpoint */
static int lsadrv_ioctl_resetpipe(struct lsadrv_device *xdev, void *arg)
{
	unsigned int ep = * (unsigned int*) arg; /* endpoint address + direction */
	Trace(LSADRV_TRACE_IOCTL, "ioctl_resetpipe: ep=%02x\n", ep);
	return lsadrv_resetpipe(xdev->udev, ep);
}

/* clear halt status of an endpoint */
static int lsadrv_ioctl_clear_halt(struct lsadrv_device *xdev, void *arg)
{
	unsigned int ep = * (unsigned int*) arg; /* endpoint address + direction */
	struct usb_device *udev = xdev->udev;
	int pipe;
	int ret;

	Trace(LSADRV_TRACE_IOCTL, "ioctl_clear_halt: ep=%02x\n", ep);

	/* check validity of endpoint number */
	if (ep & ~(USB_DIR_IN|0xf)) {
		return -EINVAL;
	}
	if (lsadrv_usb_check_epnum(udev, ep)) {
		return -ENOENT; 
	}
	if (ep & USB_DIR_IN) {
                pipe = lsadrv_usb_rcvbulkpipe(udev, ep & 0x7f);
        }
	else {
                pipe = lsadrv_usb_sndbulkpipe(udev, ep & 0x7f);
	}
	ret = lsadrv_usb_clear_halt(udev, pipe);
	if (ret < 0) {	/* error */
		xdev->LastFailedUrbStatus = ret;
	}
	return ret;
}

/* set configuration and alternative interface */
static int lsadrv_ioctl_setinterface(struct lsadrv_device *xdev, void *arg)
{
	struct lsadrv_setinterface *setif = (struct lsadrv_setinterface*) arg;
	int ret;

	Trace(LSADRV_TRACE_IOCTL, "ioctl_setinterface: if=%d, alt=%d\n",
		 setif->interface, setif->altsetting);

	ret = lsadrv_usb_set_interface(xdev->udev, setif->interface, setif->altsetting);
	if (ret < 0) {	/* error */
		xdev->LastFailedUrbStatus = ret;
	}
	return ret;
}

/* bus reset */
static int lsadrv_ioctl_reset(struct lsadrv_device *xdev)
{
	int ret;

	Trace(LSADRV_TRACE_IOCTL, "ioctl_reset\n");

	ret = lsadrv_usb_reset_device(xdev->udev);
	if (ret < 0) {
		xdev->LastFailedUrbStatus = ret;
		return ret;
	}

	/* We should simulate a disconnect() and probe() for other interfaces
 	   to insure other drivers have a chance to re-setup their interface.
	   Here, however, it is not necessary because our devices have only
	   one interface as far as today.
	*/

	return 0;
}

/* start isochronous stream */
static int lsadrv_ioctl_start_iso_stream(struct lsadrv_device *xdev, void *arg)
{
	struct lsadrv_iso_transfer_control *iso = (struct lsadrv_iso_transfer_control*) arg;
	int ret = 0;
	pid_t pgrp;

	pgrp = lsadrv_getpgrp(NULL);

	Trace(LSADRV_TRACE_IOCTL, "ioctl_start_iso_stream\n");

	/* check if claimed by this process */
	lsadrv_modlock(xdev);
	if (xdev->iso_claim != pgrp) {
		if (xdev->iso_claim && !lsadrv_find_task_by_pid(xdev->iso_claim)) {
			Trace(LSADRV_TRACE_IOCTL, "ioctl_start_iso_stream: was used by dead process %d\n", xdev->iso_claim);
			xdev->iso_claim = 0;
		}
		if (!xdev->iso_claim) {
			Info("ioctl_start_iso_stream: not claimed by process %d\n", pgrp);
		}
		else {
			Info("ioctl_start_iso_stream: claimed by other process %d\n", xdev->iso_claim);
		}
		ret = -EAGAIN;
	}
	lsadrv_modunlock(xdev);

	if (ret == 0) {
		ret = lsadrv_start_iso_stream(xdev,
				iso->Pipe,
				iso->PacketSize,
				iso->PacketCount,
				iso->FramesPerBuffer,
				iso->BufferCount);
	}
	return ret;
}

static int lsadrv_ioctl_stop_iso_stream(struct lsadrv_device *xdev)
{
	int ret;
	Trace(LSADRV_TRACE_IOCTL, "ioctl_stop_iso_stream\n");
	ret = lsadrv_stop_iso_stream(xdev);
	return 0;
}

/* read data from isochronous stream data buffer */
/* 	return value: >=0: length of data transfered; <0:error */
static int lsadrv_ioctl_read_iso_buffer(struct lsadrv_device *xdev, void *arg)
{
	struct lsadrv_iso_read_control* isor = (struct lsadrv_iso_read_control*) arg;
	int ret;
	unsigned int recSize;
	unsigned int bufsize;
	unsigned int bytesRead = 0;
	unsigned char* kbuf;
	long timeout;	/* jiffies */

	recSize = isor->PacketSize + sizeof(struct lsadrv_iso_packet_desc);
	bufsize = recSize * isor->PacketCount;
	if (isor->bufferSize < bufsize) {
		Err("read_iso_buffer: too short buffer: buffer size %u must be >= %u\n", isor->bufferSize, bufsize);
		return -EINVAL;
	}

	if (isor->buffer == NULL || !lsadrv_write_ok(isor->buffer, isor->bufferSize)) {
		Err("%s: can't access buffer: 0x%p, size=%d\n", __func__, isor->buffer, isor->bufferSize);
		return -EINVAL;
	}

	kbuf = lsadrv_malloc(bufsize);
	if (kbuf == NULL) {
		return -ENOMEM;
	}

	timeout = lsadrv_msec_to_jiffies(isor->Timeout);

	ret = lsadrv_read_iso_buffer(xdev,
			isor->PacketCount, 
			isor->PacketSize, 
			kbuf, 
			&bytesRead, 
			timeout);
	if (ret == 0 && bytesRead) {
		if ((ret=lsadrv_copy_to_user(isor->buffer, kbuf, bytesRead))) {
			Err("%s: copy_to_user error(%d)", __func__, ret);
			lsadrv_free(kbuf);
			return -EFAULT;
		}
		ret = bytesRead;
	}
	lsadrv_free(kbuf);
	return ret;
}

static int lsadrv_ioctl_claim_stream(struct lsadrv_device *xdev, void *arg)
{
	int flg = *(int*)arg;
	int ret = 0;
	pid_t pgrp;
	pid_t pid;

	pgrp = lsadrv_getpgrp(&pid);

	Trace(LSADRV_TRACE_IOCTL, "ioctl_claim_stream: flg=%d, pid=%d,pgrp=%d\n", flg, pid, pgrp);
	lsadrv_modlock(xdev);
	if (flg) {
		if (!xdev->iso_claim) {
			xdev->iso_claim = pgrp;
		}
		else if (xdev->iso_claim != pgrp) {
			if (lsadrv_find_task_by_pid(xdev->iso_claim)) {
				Info("ioctl_claim_stream: already used by process %d\n", xdev->iso_claim);
				ret = -EAGAIN;
			}
			else {
				Trace(LSADRV_TRACE_IOCTL, "ioctl_claim_stream: was used by dead process %d\n", xdev->iso_claim);
				xdev->iso_claim = pgrp;
			}
		}
	}
	else {
		if (xdev->iso_claim && xdev->iso_claim != pgrp) {
			Warning("unclaimed by other than owner(%d), pid=%d,pgrp=%d",
				xdev->iso_claim, pid, pgrp);
		}
		xdev->iso_claim = 0;
	}
	lsadrv_modunlock(xdev);
	return ret;
}

static int lsadrv_ioctl_check(struct lsadrv_device *xdev, void *arg)
{
	if (xdev->unplugged) {
		*(int*)arg = 0;
	}
	else {
		*(int*)arg = 1;
	}
	return sizeof(int);
}

static int lsadrv_ioctl_get_last_error(struct lsadrv_device *xdev, void *arg)
{
	int *status = (int*)arg;
	status[0] = xdev->LastFailedUrbStatus;
	status[1] = xdev->LastFailedStreamUrbStatus;
	return sizeof(int) * 2;
}

static int lsadrv_ioctl_get_current_frame_number(struct lsadrv_device *xdev, void *arg)
{
	int frameno;
	Trace(LSADRV_TRACE_IOCTL, "ioctl_get_current_frame_number\n");
	frameno = lsadrv_usb_get_current_frame_number(xdev->udev);
	if (frameno >= 0) {
		*(int*)arg = frameno;
		return sizeof(int);
	}
	else {
		xdev->LastFailedUrbStatus = frameno;
		return frameno;
	}
}
