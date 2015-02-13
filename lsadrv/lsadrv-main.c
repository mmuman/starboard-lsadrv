/*==========================================================================
 * lsadrv-main.c : Linux kernel driver for eIT-Xiroku optical touch sensor
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

#include <linux/sched.h>
#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/kernel.h> 	/* for linux kernel */
#include <linux/slab.h> 	/* for kmalloc */
#include "fakemouse.h"
#include <linux/module.h> 	/* for linux kernel module */
#include <linux/proc_fs.h> 	/* for use of /proc */
#include <linux/input.h>	/* for input devide */
#include <linux/kmod.h>		/* for request_module */
#include <linux/seq_file.h>	/* for single_open */
#include <linux/usbdevice_fs.h>		/* for USBDEVFS_HUB_PORTINFO */
#include <linux/errno.h>
#include <linux/version.h>

#include "lsadrv.h"
#include "lsadrv-ioctl.h"

/******** USB device ********/

/* hotplug device table support */
static struct usb_device_id lsadrv_device_table [] = {
	{ USB_DEVICE(0x0611, 0x0009) }, /* eIT-TOTOKU touch sensor */
	{ USB_DEVICE(0x1477, 0x0001) }, /* eIT-Xiroku touch sensor */
	{ USB_DEVICE(0x1477, 0x0002) }, /* eIT-Xiroku small touch sensor */
	{ USB_DEVICE(0x1477, 0x0003) }, /* eIT-Xiroku touch sensor 100K/250K */
	{ USB_DEVICE(0x1477, 0x0004) }, /* eIT-Xiroku touch sensor NL */
	{ USB_DEVICE(0x1477, 0x0005) }, /* eIT-Xiroku touch sensor NL9638 */
	{ USB_DEVICE(0x1477, 0x0006) }, /* eIT-Xiroku touch sensor NLM001 */
	{ USB_DEVICE(0x1477, 0x0007) }, /* eIT-Xiroku touch sensor HT5000 */
	{ USB_DEVICE(0x1477, 0x0008) }, /* eIT-Xiroku touch sensor HTM131 */
	{ }
};
MODULE_DEVICE_TABLE(usb, lsadrv_device_table);

static int usb_lsadrv_probe(struct usb_interface *intf, const struct usb_device_id *id);
static void usb_lsadrv_disconnect(struct usb_interface *intf);
static int usb_lsadrv_ioctl(struct usb_interface *intf, unsigned int cmd, void *arg);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
static struct usb_driver lsadrv_driver =
{
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,20)
// 	owner:			THIS_MODULE,
//#endif
	.name =			"lsadrv",
	.id_table =		lsadrv_device_table,
	.probe =		usb_lsadrv_probe,	/* probe() */
	.disconnect =		usb_lsadrv_disconnect,	/* disconnect() */
	.unlocked_ioctl =		usb_lsadrv_ioctl	/* through usbdevfs (devio) driver */
};
#else
static struct usb_driver lsadrv_driver =
{
        .name =                 "lsadrv",
        .id_table =             lsadrv_device_table,
        .probe =                usb_lsadrv_probe,       /* probe() */
        .disconnect =           usb_lsadrv_disconnect,  /* disconnect() */
        .unlocked_ioctl =                usb_lsadrv_ioctl        /* through usbdevfs (devio) driver */
};
#endif

static LIST_HEAD(device_list);
static LIST_HEAD(memleak_list);
static struct semaphore device_list_lock;


/******** input device ********/
static int  lsadrv_input_open(struct input_dev *idev);
static void lsadrv_input_close(struct input_dev *idev);

struct lsadrv_input_dev *lsadrv_idev;

/******** global/static variables ********/
int lsadrv_trace = 0;


/***************************************************************************/
/* Private functions */
static inline void
usb_to_input_id(const struct usb_device *dev, struct input_id *id)
{
	id->bustype = BUS_USB;
	id->vendor = le16_to_cpu(dev->descriptor.idVendor);
	id->product = le16_to_cpu(dev->descriptor.idProduct);
	id->version = le16_to_cpu(dev->descriptor.bcdDevice);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
static struct input_dev *
input_allocate_device(void)
{
	struct input_dev *idev = kmalloc(sizeof(struct input_dev), GFP_KERNEL);
	if (idev) {
		memset(idev, 0, sizeof(struct input_dev));
		init_input_dev(idev);
	}
	return idev;
}
static inline void input_free_device(struct input_dev *dev)
{
	kfree(dev);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
static inline void *input_get_drvdata(struct input_dev *dev)
{
	return dev->private;
}

static inline void input_set_drvdata(struct input_dev *dev, void *data)
{
	dev->private = data;
}
#endif

/***************************************************************************/
/* input subsystem functions */

/* called from event handler when input device file (eventNN) is opened */
static int lsadrv_input_open(struct input_dev *idev)
{
	int rc = 0;
	struct lsadrv_input_dev *xidev = lsadrv_idev;

	Trace(LSADRV_TRACE_OPEN, ">> input_open: ptr=0x%p\n", idev);

	if (idev == NULL) {
		BUG();
	}
	if (xidev->idev != idev) {
		BUG();
	}

	/* just check open count */
	/*  (never called after unregistered from input subsystem) */
	if (xidev->open++) {
		rc = -EBUSY;	/* though nobody seems to use this return code... */
	}
	Trace(LSADRV_TRACE_OPEN, "<< input_open: count=%d\n", xidev->open);

	return rc;
}

/* called from event handler when input device file (eventNN) is closed */
static void lsadrv_input_close(struct input_dev *idev)
{
	struct lsadrv_input_dev *xidev = lsadrv_idev;

	Trace(LSADRV_TRACE_OPEN, ">> input_close: ptr=0x%p\n", idev);
	if (idev == NULL) {
		BUG();
	}
	if (xidev->idev != idev) {
		BUG();
	}

	if (--xidev->open < 0) {
		Warning("input_close: closed too many times? : count=%d\n", xidev->open);
	}

	Trace(LSADRV_TRACE_OPEN, "<< input_close: count=%d\n", xidev->open);
}

static void lsadrv_set_key_bits(struct input_dev *idev)
{
	const int *list;
	int n, i;
	n = lsadrv_get_key_list(&list);
	for (i = 0; i < n; ++i) {
		set_bit(list[i], idev->keybit);
	}
}

static int fill_input_dev(struct lsadrv_input_dev *xidev)
{
	struct input_dev *idev;
	memset(xidev, 0, sizeof(*xidev));
	idev = input_allocate_device();
	if (idev == NULL) {
		return -1;
	}
	xidev->idev = idev;
	input_set_drvdata(idev, xidev);
	/* idev->int number;  //set in input_register_device() */
	idev->name = "lsadrv";

#ifdef PREVENT_MOUSE_DRIVER_MATCH
	idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	idev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_RIGHT) | BIT_MASK(BTN_MIDDLE);
  	idev->keybit[BIT_WORD(BTN_DIGI)] = BIT_MASK(BTN_TOOL_PEN);
#else //!PREVENT_MOUSE_DRIVER_MATCH
	idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);// | BIT_MASK(EV_MSC);// | BIT_MASK(EV_REL);
//	idev->relbit[BIT_WORD(REL_X)]  = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
	idev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT) | BIT_MASK(BTN_MIDDLE);
  	idev->keybit[BIT_WORD(BTN_DIGI)] = BIT_MASK(BTN_TOOL_PEN) | BIT_MASK(BTN_TOUCH);
  	//idev->mscbit[0] = BIT_MASK(MSC_SERIAL);
#endif //!PREVENT_MOUSE_DRIVER_MATCH

	lsadrv_set_key_bits(idev);

	input_set_abs_params(idev, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, 65535, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, 1023, 0, 0);

	/* operations */
	idev->open = lsadrv_input_open;
	idev->close = lsadrv_input_close;

	return 0;
}

/***************************************************************************
 *
 * USB functions
 *
 ***************************************************************************/

/* This function gets called when a new device is plugged in or the usb driver is loaded. */
static int usb_lsadrv_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct lsadrv_device *xdev = NULL;

	Trace(LSADRV_TRACE_PROBE, ">> probe: VID/PID=%04X/%04X\n", udev->descriptor.idVendor, udev->descriptor.idProduct);

	/* Check if we can handle this device */
	/* 	-> omit --- filtered by device table */

	/* Allocate lsadrv_device structure */
	xdev = kmalloc(sizeof(struct lsadrv_device), GFP_KERNEL);
	if (xdev == NULL) {
		Err("could not allocate memory for lsadrv_device\n");
		return -ENOMEM;
	}
	memset(xdev, 0, sizeof(struct lsadrv_device));

	xdev->udev = udev;
	lsadrv_spin_lock_init(&xdev->streamLock);
	sema_init(&xdev->modlock, 1); 
	init_waitqueue_head(&xdev->remove_ok);

	/* set ids as input device */
	if (usb_make_path(xdev->udev, lsadrv_idev->phys_path, sizeof(lsadrv_idev->phys_path)) > 0) {
		lsadrv_idev->idev->phys = lsadrv_idev->phys_path;
	}
	usb_to_input_id(udev, &lsadrv_idev->idev->id);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 10)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 24))
	lsadrv_idev->idev->cdev.dev = &intf->dev;
#endif

	/* Add it to the device list */
	down(&device_list_lock);
	list_add(&xdev->device_list, &device_list);
	up(&device_list_lock);

	Trace(LSADRV_TRACE_PROBE, "<< probe: returning 0x%p\n", xdev);
	usb_set_intfdata(intf, xdev);
	return 0;
}

/* Usb device is unplugged or driver is shutting down ... */
static void usb_lsadrv_disconnect(struct usb_interface *intf)
{
	DECLARE_WAITQUEUE(wait, current);
	struct lsadrv_device *xdev;
	int usage;

	xdev = (struct lsadrv_device *) usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);

	Trace(LSADRV_TRACE_PROBE, ">> disconnect: xdev=0x%p\n", xdev);

	if (xdev == NULL) {
		Err("lsadrv_disconnect() Called without private pointer.\n");
		return;
	}

	if (xdev->udev != interface_to_usbdev(intf)) {
		Err("lsadrv_disconnect(): pointer mismatch.\n");
		return;
	}

	/* remove from the device list */
	down(&device_list_lock);
	list_del(&xdev->device_list);
	up(&device_list_lock);

	if (xdev->udev == NULL) {
		Err("disconnect: already called for %p\n", xdev);
		return;
	}

	xdev->unplugged = 1;

	lsadrv_stop_iso_stream(xdev);

	/* wait for i/o in progress done */
	add_wait_queue(&xdev->remove_ok, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	while ((usage=xdev->usage)) {
		Trace(LSADRV_TRACE_PROBE, "waiting io: usage=%d\n", usage);
		schedule();
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&xdev->remove_ok, &wait);

	lsadrv_spin_lock_term(xdev->streamLock);

	/* free memory */
	Trace(LSADRV_TRACE_PROBE, "disconnect: cleaning up memories.\n");
	if (lsadrv_idev) {
		input_set_drvdata(lsadrv_idev->idev, NULL);
		lsadrv_idev->idev->phys = NULL;
		lsadrv_idev->idev->id.vendor = 0;
		lsadrv_idev->idev->id.product = 0;
		lsadrv_idev->idev->id.version = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 10)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 24))
		lsadrv_idev->idev->cdev.dev = NULL;
#endif
	}
	kfree(xdev);

	Trace(LSADRV_TRACE_PROBE, "<< disconnect\n");
}

/* usb ioctl - called through devio. So, copy from/to user is done outside */
static int usb_lsadrv_ioctl(struct usb_interface *intf, unsigned int cmd, void *arg)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct lsadrv_device *xdev = (struct lsadrv_device *) usb_get_intfdata(intf);
	int ret = 0;

	Trace(LSADRV_TRACE_IOCTL, ">>lsadrv_ioctl(%d) size=%d\n", _IOC_NR(cmd), _IOC_SIZE(cmd));

	if (arg == NULL && _IOC_DIR(cmd) != _IOC_NONE && _IOC_SIZE(cmd) != 0) {
		Err("ioctl: arg is NULL: cmd=0x%x\n", cmd);
		ret = -EINVAL;
		goto l_ret;
	}

	if (udev == NULL) {
		Err("ioctl: udev is NULL\n");
		ret = -EFAULT;
		goto l_ret;
	}

	if (xdev == NULL) {
		Err("ioctl: xdev is NULL\n");
		ret = -EFAULT;
		goto l_ret;
	}

	/* this IOCTL may be sent often... */
	if (cmd == USBDEVFS_HUB_PORTINFO) {
		/* just returns error because this is not a hub */
		ret = -EINVAL;
		goto l_ret;
	}

	lsadrv_modlock(xdev);
	xdev->usage++;
	lsadrv_modunlock(xdev);

	ret = lsadrv_usb_ioctl(xdev, cmd, arg);

	lsadrv_modlock(xdev);
	xdev->usage--;
	lsadrv_modunlock(xdev);

	/* wake up the waiting threads */
	wake_up_interruptible(&xdev->remove_ok);

l_ret:
	Trace(LSADRV_TRACE_IOCTL, "<<lsadrv_ioctl(%d): ret=%d\n", _IOC_NR(cmd), ret);
	return ret;
}


static struct lsadrv_proc_files {
	struct proc_dir_entry *lsadrvDirEntry;			/* procfs/driver/lsadrv */
	struct proc_dir_entry *devicesFileEntry;		/* devices file */
	//struct proc_dir_entry *mappingDirEntry;		/* procfs/driver/lsadrv/X-X:X */
} lsadrv_files;


/*** seq_file show operation for 'devices' file ***/
static int lsadrv_devices_show(struct seq_file *m, void *v)
{
	struct list_head *tmp;

	down(&device_list_lock);
	tmp = device_list.next;
	while (tmp != &device_list) {
		struct lsadrv_device *xdev = list_entry(tmp, struct lsadrv_device, device_list);
		tmp = tmp->next;
		seq_printf(m, "%03d/%03d\n", xdev->udev->bus->busnum, xdev->udev->devnum);
	}
	up(&device_list_lock);
	return 0;
}


/*** open operation handler for 'devices' file ***/
static int lsadrv_devices_open(struct inode *inode, struct file *file)
{
	/* should use seq_open() but this should be sufficient for the need */
	return single_open(file, lsadrv_devices_show, NULL);
}


/*** remove procfs files and directory ***/
static void lsadrv_remove_procfs_dir(void)
{
	struct lsadrv_proc_files *files = &lsadrv_files;
	if (files->devicesFileEntry) {
		remove_proc_entry("devices", files->lsadrvDirEntry);
		files->devicesFileEntry = NULL;
	}
	if (files->lsadrvDirEntry) {
/*		remove_proc_entry("lsadrv", proc_root_driver);*/
		remove_proc_entry("driver/lsadrv", NULL);
		files->lsadrvDirEntry = NULL;
	}
}

/*** create procfs directory ***/
static void lsadrv_create_procfs_dir(void)
{
	struct lsadrv_proc_files *files = &lsadrv_files;
	static struct file_operations proc_fops = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
			.owner    = THIS_MODULE,
#endif
			.open     = lsadrv_devices_open,
			.read     = seq_read,
			.llseek   = seq_lseek,
			.release  = seq_release
	};
	/* Make procfs/driver/lsadrv directory */
/*	files->lsadrvDirEntry = create_proc_entry("lsadrv", S_IFDIR, proc_root_driver);*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
	files->lsadrvDirEntry = create_proc_entry("driver/lsadrv", S_IFDIR, NULL);
#else

	files->lsadrvDirEntry = proc_mkdir("driver/lsadrv", NULL);
#endif
	if (files->lsadrvDirEntry == NULL) {
		return;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	files->lsadrvDirEntry->owner = THIS_MODULE;
#endif
	/* procfs/driver/lsadrv/devices file */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
	files->devicesFileEntry = create_proc_entry("devices",
							   S_IFREG|S_IRUGO,
							   files->lsadrvDirEntry);
	if (files->devicesFileEntry == NULL) {
		lsadrv_remove_procfs_dir();
		return;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	files->devicesFileEntry->owner = THIS_MODULE;
#endif
	files->devicesFileEntry->data = NULL;
	files->devicesFileEntry->proc_fops = &proc_fops;
#else
	files->devicesFileEntry = proc_create("devices",
			S_IFREG|S_IRUGO,
			files->lsadrvDirEntry,
			&proc_fops
			);
	if (files->devicesFileEntry == NULL) {
		lsadrv_remove_procfs_dir();
		return;
	}
#endif
}

/***************************************************************************
 *
 * Initialization code & module stuff 
 *
 ***************************************************************************/

static int trace = -1;

module_param(trace, int, 0644);
MODULE_PARM_DESC(trace, "For debugging purposes");

MODULE_DESCRIPTION("lsadrv touch sensor driver");
MODULE_AUTHOR("eIT Co. Ltd. & Xiroku Inc.");
MODULE_LICENSE("GPL");

static int __init usb_lsadrv_init(void)
{
	struct lsadrv_input_dev *xidev;
	int err;

	Info("lsadrv touch sensor driver version " LSADRV_KDRIVER_VERSION " loaded.\n");

	/*** driver options ***/
	/* trace mode */
	if (trace >= 0) {
		Info("Trace options: 0x%04x\n", trace);
		lsadrv_trace = trace;
	}

	Debug("init_Mutex\n");
	sema_init(&device_list_lock, 1); 

	/*** create procfs directory and 'devices' file ***/
	Debug("creating procfs\n");
	lsadrv_create_procfs_dir();

	/*** load 'evdev' module if not loaded yet ***/
	Debug("request_module\n");
	request_module("evdev");

	/* Allocate input_dev structure */
	Debug("allocating input_dev\n");
	xidev = kmalloc(sizeof(struct lsadrv_input_dev), GFP_KERNEL);
	if (xidev == NULL) {
		Err("could not allocate memory for lsadrv_input_dev.\n");
		lsadrv_remove_procfs_dir();
		return -ENOMEM;
	}
	if (fill_input_dev(xidev)) {
		Err("could not allocate memory for input device.\n");
		kfree(xidev);
		lsadrv_remove_procfs_dir();
		return -ENOMEM;
	}

	lsadrv_idev = xidev;

	/* Register to input subsystem */
	Debug("registering input device\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 15)
	err = input_register_device(xidev->idev);
#else
	input_register_device(xidev->idev);
	err = 0;
#endif
	if (err) {
		Err("could not register input device.\n");
		input_free_device(xidev->idev);
		lsadrv_idev = NULL;
		kfree(xidev);
		lsadrv_remove_procfs_dir();
		return err;
	}
	Trace(LSADRV_TRACE_MODULE, "Registered input struct at 0x%p.\n", xidev->idev);
	Info("Registered input device\n"/*, xidev->idev.number*/);

 	Trace(LSADRV_TRACE_MODULE, "Registering driver at address 0x%p.\n", &lsadrv_driver);
	err = usb_register(&lsadrv_driver);
	if (err) {
		/*Err("failed to register usb device.\n");*/
		input_unregister_device(xidev->idev); 
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
		input_free_device(xidev->idev);
#endif
		lsadrv_idev = NULL;
		kfree(xidev);
		lsadrv_remove_procfs_dir();
	}

	if (!err) create_fakemouse();

	return err;
}

static void __exit usb_lsadrv_exit(void)
{
	struct list_head *tmp;

	/* unregister usb device */
	Trace(LSADRV_TRACE_MODULE, "Deregistering usb driver.\n");
	usb_deregister(&lsadrv_driver);

	/* unregister input devide */
	if (lsadrv_idev) {
		Trace(LSADRV_TRACE_MODULE, "Unregistering input device.\n");
		input_unregister_device(lsadrv_idev->idev); 
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10)
		input_free_device(lsadrv_idev->idev);
#endif
		kfree(lsadrv_idev);
		lsadrv_idev = NULL;
	}

	/*** remove procfs files and directory ***/
	Debug("removing procfs dir\n");
	lsadrv_remove_procfs_dir();

	/*** free remained memories ***/
	tmp = memleak_list.next;
	if (tmp != &memleak_list) {
		Debug("freeing remained memories\n");
		while (tmp != &memleak_list) {
			struct lsadrv_device *xdev = list_entry(tmp, struct lsadrv_device, device_list);
			tmp = tmp->next;
			Trace(LSADRV_TRACE_MODULE, "cleaning up memories:0x%p.\n", xdev);
			list_del(&xdev->device_list);
			kfree(xdev);
		}
	}
	// destroy
        destroy_fakemouse();
	Info("lsadrv driver removed.\n");
}

module_init(usb_lsadrv_init);
module_exit(usb_lsadrv_exit);

