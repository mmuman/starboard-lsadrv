#ifndef __FAKEMOUSE_H
#define __FAKEMOUSE_H

void destroy_fakemouse(void);
int create_fakemouse(void);

#include "lsadrv-ioctl.h"

#define FAKEMOUSE_MAGIC 'h'
#define FAKEMOUSE_IOCTL_BASE 0x80
#define FAKEMOUSE_IOC_MOUSEEVENT _IOW(FAKEMOUSE_MAGIC,FAKEMOUSE_IOCTL_BASE,struct lsadrv_mouse_input)

#endif
