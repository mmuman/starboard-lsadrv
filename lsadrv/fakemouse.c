#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/kernel.h>       /* for linux kernel */
#include <linux/module.h>       /* for linux kernel module */
#include <linux/proc_fs.h>      /* for use of /proc */
#include <linux/input.h>        /* for input devide */
#include <linux/kmod.h>         /* for request_module */
#include <linux/usbdevice_fs.h>         /* for USBDEVFS_HUB_PORTINFO */
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/ioctl.h>

#include <linux/cdev.h>
#include <linux/ioctl.h>

#include "fakemouse.h"

#define MODNAME fakemouse

#define stringify(X) _stringify(X)
#define _stringify(X) #X
#define LOG(X) printk(KERN_ALERT stringify( MODNAME ) ":" X "\n");
#define REPORT(FMT,VAL) printk(KERN_ALERT stringify( MODNAME ) ":" FMT "!\n", VAL);
#define WARN(X) printk(KERN_WARNING stringify( MODNAME ) ":" X "!\n");

static int fakemouse_ioctl(struct inode *inode, struct file *filp,unsigned int command, unsigned long arg);
static int fakemouse_device_open(struct inode *inode, struct file *filp);
static int fakemouse_device_release(struct inode *inode, struct file *filp);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
struct file_operations fakemouse_fops =
  {
    .owner   = THIS_MODULE,
    .llseek  = 0,
    .read    = 0,
    .write   = 0,
    .ioctl   = fakemouse_ioctl,
    .open    = fakemouse_device_open,
    .release = fakemouse_device_release,
  };
#else
struct file_operations fakemouse_fops =
  {
    .owner   = THIS_MODULE,
    .llseek  = 0,
    .read    = 0,
    .write   = 0,
    .unlocked_ioctl   = fakemouse_ioctl,
    .open    = fakemouse_device_open,
    .release = fakemouse_device_release,
  };
#endif


int lsadrv_ioctl_mouseevent_dispatch(void *arg);

static int fakemouse_ioctl(struct inode *inode, struct file *filp,unsigned int command, unsigned long arg)
{
  struct lsadrv_mouse_input *minp = (struct lsadrv_mouse_input*)arg;

  if (_IOC_TYPE(command) != FAKEMOUSE_MAGIC) return -ENOTTY;

  switch(command)
    {
    case FAKEMOUSE_IOC_MOUSEEVENT:
      /*      printk("FAKEMOUSE_IOC_MOUSEEVENT: (%d,%d,0x%x)\n", minp->dx, minp->dy, minp->flags);*/
      lsadrv_ioctl_mouseevent_dispatch((void*)arg);
      return 0;
    default:
      return -ENOTTY;
    }
}

static int fakemouse_device_open(struct inode *inode, struct file *filp)
{
  return 0;
}

static int fakemouse_device_release(struct inode *inode, struct file *filp)
{
  return 0;
}

dev_t fakemouse_dev;
int fakemouse_major=0;
unsigned int fakemouse_count=1;
const char *fakemouse_devname = "fakemouse";
struct cdev *fakemouse_cdev = 0;

int create_fakemouse(void)
{
  int result;

  fakemouse_major=0;
  fakemouse_cdev=0;

  LOG("fakemouse_init");
  result = alloc_chrdev_region(&fakemouse_dev,0,fakemouse_count,fakemouse_devname);
  fakemouse_major = MAJOR(fakemouse_dev);
  
  if (result<0)
    {
      WARN("Cannot get device");
      return result;
    }
  else
    {
      REPORT("Major = %d",fakemouse_major);
    }

  LOG("create device handler");
  fakemouse_cdev = cdev_alloc();
  cdev_init(fakemouse_cdev,&fakemouse_fops);
  fakemouse_cdev->owner = THIS_MODULE;

  LOG("add device handler");
  result = cdev_add(fakemouse_cdev,fakemouse_dev,fakemouse_count);

   // A non 0 return means init_module failed; module can't be loaded.
   return 0;
}


void destroy_fakemouse(void)
{
  if (fakemouse_cdev)
    {
      LOG("delete character device handler");
      cdev_del(fakemouse_cdev);
    }

  if (fakemouse_major)
    {
      LOG("Unregister device numbers");
      unregister_chrdev_region(fakemouse_dev,fakemouse_count);
    }

  LOG("fakemouse_exit");
}  
