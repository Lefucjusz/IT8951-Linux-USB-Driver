#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

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
	{ 0x82d424fa, "usb_deregister" },
	{ 0x122c3a7e, "_printk" },
	{ 0x258279d5, "fb_sys_write" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0x41b01de4, "sys_fillrect" },
	{ 0x8c59ea10, "sys_copyarea" },
	{ 0xe6fc7050, "sys_imageblit" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0x24980310, "kmalloc_caches" },
	{ 0x1d199deb, "kmalloc_trace" },
	{ 0x5bd0f510, "_dev_err" },
	{ 0xf8d67447, "usb_get_dev" },
	{ 0x93c7edeb, "usb_find_common_endpoints" },
	{ 0x978b073b, "framebuffer_alloc" },
	{ 0xd6ee688f, "vmalloc" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xf3472e9d, "fb_deferred_io_init" },
	{ 0xf012d9a9, "register_framebuffer" },
	{ 0x8738c14f, "fb_sys_read" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xc512a579, "usb_register_driver" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x6d169046, "_dev_info" },
	{ 0xf92efe43, "fb_deferred_io_cleanup" },
	{ 0x217f722f, "unregister_framebuffer" },
	{ 0xaba83423, "framebuffer_release" },
	{ 0x999e8297, "vfree" },
	{ 0x37a0cba, "kfree" },
	{ 0x79853de5, "usb_bulk_msg" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x6ad2b3e, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v048Dp8951d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "F6F9F9ECCE28094D28B208B");
