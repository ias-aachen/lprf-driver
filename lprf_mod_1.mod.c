#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
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
	{ 0x54702fc, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x51eafc8e, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0x57464483, __VMLINUX_SYMBOL_STR(driver_unregister) },
	{ 0x36bde218, __VMLINUX_SYMBOL_STR(spi_register_driver) },
	{ 0xe35c5fd1, __VMLINUX_SYMBOL_STR(ieee802154_register_hw) },
	{ 0x12a38747, __VMLINUX_SYMBOL_STR(usleep_range) },
	{ 0x90b7508e, __VMLINUX_SYMBOL_STR(gpiod_set_raw_value) },
	{ 0xa8a111df, __VMLINUX_SYMBOL_STR(gpio_to_desc) },
	{ 0x2aa96004, __VMLINUX_SYMBOL_STR(devm_gpio_request_one) },
	{ 0xb3b1a04e, __VMLINUX_SYMBOL_STR(regmap_write) },
	{ 0x63b87fc5, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0xb5d349d1, __VMLINUX_SYMBOL_STR(regmap_read) },
	{ 0xf14aa404, __VMLINUX_SYMBOL_STR(hrtimer_init) },
	{ 0x2532195b, __VMLINUX_SYMBOL_STR(devm_regmap_init_spi) },
	{ 0x79aa04a2, __VMLINUX_SYMBOL_STR(get_random_bytes) },
	{ 0x5cde6e71, __VMLINUX_SYMBOL_STR(ieee802154_alloc_hw) },
	{ 0x4109508a, __VMLINUX_SYMBOL_STR(of_property_read_u8_array) },
	{ 0xfa1cdfc4, __VMLINUX_SYMBOL_STR(of_get_named_gpio_flags) },
	{ 0x4a8b0931, __VMLINUX_SYMBOL_STR(cdev_add) },
	{ 0xc46bc12d, __VMLINUX_SYMBOL_STR(cdev_init) },
	{ 0xdc798d37, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x29537c9e, __VMLINUX_SYMBOL_STR(alloc_chrdev_region) },
	{ 0x12da5bb2, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xd8e484f0, __VMLINUX_SYMBOL_STR(register_chrdev_region) },
	{ 0xfac135a4, __VMLINUX_SYMBOL_STR(ieee802154_free_hw) },
	{ 0x8e4ac0a, __VMLINUX_SYMBOL_STR(ieee802154_unregister_hw) },
	{ 0x7485e15e, __VMLINUX_SYMBOL_STR(unregister_chrdev_region) },
	{ 0x4b85b0c7, __VMLINUX_SYMBOL_STR(cdev_del) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x67c2fa54, __VMLINUX_SYMBOL_STR(__copy_to_user) },
	{ 0x176d57c, __VMLINUX_SYMBOL_STR(spi_sync) },
	{ 0x8e865d3c, __VMLINUX_SYMBOL_STR(arm_delay_ops) },
	{ 0x2e92bd81, __VMLINUX_SYMBOL_STR(ieee802154_xmit_complete) },
	{ 0x60f71cfa, __VMLINUX_SYMBOL_STR(complete) },
	{ 0xb33d9781, __VMLINUX_SYMBOL_STR(regmap_update_bits) },
	{ 0x5a17e7b0, __VMLINUX_SYMBOL_STR(hrtimer_start) },
	{ 0x9fbf4f10, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x75b67c72, __VMLINUX_SYMBOL_STR(spi_async) },
	{ 0x9d669763, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0xbc10dd97, __VMLINUX_SYMBOL_STR(__put_user_4) },
	{ 0xc6cbbc89, __VMLINUX_SYMBOL_STR(capable) },
	{ 0x902c21d8, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0xfbc74f64, __VMLINUX_SYMBOL_STR(__copy_from_user) },
	{ 0xfa2a45e, __VMLINUX_SYMBOL_STR(__memzero) },
	{ 0xebdf49ef, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x2e5810c6, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr1) },
	{ 0xb1ad28e0, __VMLINUX_SYMBOL_STR(__gnu_mcount_nc) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=mac802154,regmap-spi";

MODULE_ALIAS("spi:lprf");
MODULE_ALIAS("spi:at86rf231");
MODULE_ALIAS("spi:at86rf233");
MODULE_ALIAS("spi:at86rf212");
MODULE_ALIAS("of:N*T*Catmel,at86rf233*");
MODULE_ALIAS("of:N*T*Catmel,at86rf212*");

MODULE_INFO(srcversion, "8D05E50BA97F287FAD26045");
