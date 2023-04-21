#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xf704969, "module_layout" },
	{ 0xf247fba3, "param_ops_int" },
	{ 0xf97f0ac2, "pci_unregister_driver" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x933c4a18, "class_destroy" },
	{ 0x193e3022, "__pci_register_driver" },
	{ 0x325cb5cb, "__class_create" },
	{ 0x289828cc, "__register_chrdev" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xefc94da8, "device_create" },
	{ 0xc378cea7, "cdev_add" },
	{ 0x2d725fd4, "cdev_init" },
	{ 0xaa235e02, "cdev_alloc" },
	{ 0xde80cd09, "ioremap" },
	{ 0x85bd1608, "__request_region" },
	{ 0xe9c2ad91, "dma_set_mask" },
	{ 0x7f8d7fc1, "pci_set_master" },
	{ 0x698c665b, "pci_enable_device" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x1000e51, "schedule" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x800473f, "__cond_resched" },
	{ 0xf1969a8e, "__usecs_to_jiffies" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x46cf10eb, "cachemode2protval" },
	{ 0x6943d575, "remap_pfn_range" },
	{ 0xa92ec74, "boot_cpu_data" },
	{ 0x92997ed8, "_printk" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x7c797b6, "kmem_cache_alloc_trace" },
	{ 0xd731cdd9, "kmalloc_caches" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x1035c7c2, "__release_region" },
	{ 0x77358855, "iomem_resource" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x7b9a2346, "pci_disable_device" },
	{ 0x37a0cba, "kfree" },
	{ 0xc946dda0, "cdev_del" },
	{ 0x82e7bb9c, "device_destroy" },
	{ 0xedc03953, "iounmap" },
	{ 0xc1514a3b, "free_irq" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("pci:v000010B5d00009056sv00006000sd00000826bc*sc*i*");

MODULE_INFO(srcversion, "A7504DE3E24A0DE9DB53BB7");
