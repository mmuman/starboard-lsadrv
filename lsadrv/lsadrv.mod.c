#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xa9ee6c53, "struct_module" },
	{ 0x2fce2b5c, "per_cpu__current_task" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0x608c2831, "warn_on_slowpath" },
	{ 0xa5423cc4, "param_get_int" },
	{ 0x13988844, "malloc_sizes" },
	{ 0x71356fba, "remove_wait_queue" },
	{ 0x3034ec1b, "remove_proc_entry" },
	{ 0xcb32da10, "param_set_int" },
	{ 0x712aa29b, "_spin_lock_irqsave" },
	{ 0x3c2c5af5, "sprintf" },
	{ 0x2d084055, "usb_unlink_urb" },
	{ 0x6f651bd3, "input_event" },
	{ 0xffd5a395, "default_wake_function" },
	{ 0xe23dcfa7, "usb_deregister" },
	{ 0xb72397d5, "printk" },
	{ 0xa622bb64, "usb_set_interface" },
	{ 0x2f287f0d, "copy_to_user" },
	{ 0x6208286f, "usb_control_msg" },
	{ 0x748caf40, "down" },
	{ 0x4b07e779, "_spin_unlock_irqrestore" },
	{ 0x8f2c2b85, "usb_submit_urb" },
	{ 0x4bb373ee, "kmem_cache_alloc" },
	{ 0xdff67b9d, "usb_reset_device" },
	{ 0x4459e4fe, "input_register_device" },
	{ 0xfd9ef4b1, "usb_bulk_msg" },
	{ 0x4292364c, "schedule" },
	{ 0xd62c833f, "schedule_timeout" },
	{ 0x82213f93, "usb_clear_halt" },
	{ 0xc0ed3ddf, "input_free_device" },
	{ 0xea8a9928, "create_proc_entry" },
	{ 0x64cd5d16, "init_waitqueue_head" },
	{ 0x57a6504e, "vsnprintf" },
	{ 0x642e54ac, "__wake_up" },
	{ 0x786108a3, "init_pid_ns" },
	{ 0x650fb346, "add_wait_queue" },
	{ 0x37a0cba, "kfree" },
	{ 0x753b9dcf, "input_unregister_device" },
	{ 0xe762d6e, "request_module" },
	{ 0x3f1899f1, "up" },
	{ 0xcffcd227, "usb_register_driver" },
	{ 0x84d4e425, "find_task_by_pid_type_ns" },
	{ 0x690a3542, "usb_ifnum_to_if" },
	{ 0x701d0ebd, "snprintf" },
	{ 0x1fca03aa, "usb_get_current_frame_number" },
	{ 0xd6c963c, "copy_from_user" },
	{ 0xa9d815a3, "usb_free_urb" },
	{ 0x330a0079, "usb_alloc_urb" },
	{ 0xd43920e0, "input_allocate_device" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=usbcore";

MODULE_ALIAS("usb:v0611p0009d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v1477p0001d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v1477p0002d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v1477p0003d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v1477p0004d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v1477p0005d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v1477p0006d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v1477p0007d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v1477p0008d*dc*dsc*dp*ic*isc*ip*");

MODULE_INFO(srcversion, "15E090494B7C470665F1B6E");
