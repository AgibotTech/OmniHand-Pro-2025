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
	{ 0x441180e2, "netdev_info" },
	{ 0x31f021f6, "free_candev" },
	{ 0x3d012277, "unregister_netdev" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0xfff91647, "device_remove_file" },
	{ 0x2a624fc6, "kthread_stop" },
	{ 0x962c8ae1, "usb_kill_anchored_urbs" },
	{ 0x47de82a, "usb_free_coherent" },
	{ 0x37a0cba, "kfree" },
	{ 0x478c48d3, "can_get_echo_skb" },
	{ 0x57f19d91, "netif_tx_wake_queue" },
	{ 0xe2c17b5d, "__SCT__might_resched" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0x5b420059, "open_candev" },
	{ 0x115ae74d, "usb_alloc_urb" },
	{ 0x9c2b6252, "usb_alloc_coherent" },
	{ 0x33f05afb, "usb_anchor_urb" },
	{ 0x95a97876, "usb_submit_urb" },
	{ 0x4a68e990, "usb_unanchor_urb" },
	{ 0x438ef155, "usb_free_urb" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x754d539c, "strlen" },
	{ 0x69acdf38, "memcpy" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0x42bb5ed1, "alloc_canfd_skb" },
	{ 0x896ffd12, "netif_rx" },
	{ 0x2bfbe542, "alloc_can_err_skb" },
	{ 0x3605ee1e, "alloc_can_skb" },
	{ 0x847118bf, "netif_device_detach" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0x37a99944, "kmalloc_caches" },
	{ 0x22e14f04, "kmalloc_trace" },
	{ 0x7739d699, "usb_control_msg" },
	{ 0x9ed12e20, "kmalloc_large" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x5f25142e, "alloc_candev_mqs" },
	{ 0x5f6904ad, "register_candev" },
	{ 0x1125fa9c, "netdev_err" },
	{ 0x475890ba, "kthread_create_on_node" },
	{ 0x49a3a20, "wake_up_process" },
	{ 0xba456f63, "device_create_file" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x312574ed, "close_candev" },
	{ 0xe2964344, "__wake_up" },
	{ 0x301a309, "can_dropped_invalid_skb" },
	{ 0x47438676, "can_put_echo_skb" },
	{ 0x56470118, "__warn_printk" },
	{ 0xc0739aed, "can_change_mtu" },
	{ 0x2b8e9bfe, "param_ops_uint" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x21cca84, "usb_register_driver" },
	{ 0x122c3a7e, "_printk" },
	{ 0xbd6841d4, "crc16" },
	{ 0xe198a662, "usb_bulk_msg" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x2a66d022, "usb_deregister" },
	{ 0xc6227e48, "module_layout" },
};

MODULE_INFO(depends, "can-dev");

MODULE_ALIAS("usb:v04CCp1240d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v3068p0009d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "9B8CA53C40DEF8DA75A920A");
