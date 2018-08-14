/*
 * drivers/leds/leds-an32180a.c
 *
 * Copyright (C) 2016 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/leds-pca9532.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <sharp/sh_smem.h>

#ifdef CONFIG_PWM_LED
#include <sharp/shub_driver.h>
#endif /* CONFIG_PWM_LED */
#include "leds-an32180a.h"
#include <sharp/shled_api.h>

/* sleep */
static void an32180_msleep(int msec);

/* etc */
static int an32180_set_brightness(unsigned char value, int no, int color);
static int an32180_set_blink(struct device *dev, int blinking);
static unsigned char an32180_correct_dim(unsigned char value);
static int an32180_set_pwm(int pwm_hi, int pwm_lo, int pwm_stat);

/* PM */
static int an32180_pm_init(unsigned int user);
static int an32180_pm_is_active(void);
static void an32180_pm_set_active(unsigned int user);
static void an32180_pm_set_standby(void);
static int an32180_pm_power_manager(unsigned int user, int onoff);

/* register */
static int an32180_reg_read(struct i2c_client *client, unsigned char reg, unsigned char *value);
static int an32180_reg_table_write(struct i2c_client *client, const leds_regsetting_t *regtable, int size);
static int an32180_reg_write(struct i2c_client *client, unsigned char reg, unsigned char value);

/* I2C */
static int an32180_i2c_init(void);
static int an32180_i2c_exit(void);
static int an32180_i2c_read(struct i2c_client *client, unsigned char reg);
static int an32180_i2c_write(struct i2c_client *client, unsigned char reg, unsigned char value);
static int an32180_i2c_table_write(struct i2c_client *client, const leds_regsetting_t **regtable, int size);
static int an32180_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int an32180_i2c_remove(struct i2c_client *client);

/* GPIO */
static int an32180_gpio_set_value(int num, int value);
static int an32180_gpio_get_value(int num);
static int an32180_probe(struct platform_device *pdev);
static int an32180_led_register_driver(void);

/* initialize */
static void an32180_set_clrvari(unsigned char color);
static unsigned char an32180_get_boot_ctx_handset_color(void);
static void an32180_ctx_init(void);
static void an32180_gpio_init(struct platform_device *pdev);
static void an32180_takeover_leds_status(void);
static void an32180_takeover_boot_status(struct platform_device *pdev);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define LED_MODE_FIX				(0)
#define LED_MODE_BLINK				(1)

/* PM */
#define AN32180_LED_MAX				(3)
#define AN32180_COLOR_MAX			(3)

#define AN32180_PM_MAKE_USER(id, color)		(1 << ((id * AN32180_LED_MAX) + color))

#define LED_PM_TYPE_NONE			(0x00000000)
#define LED_PM_TYPE_BRIGHTNESS		(0x0000FFFF)
#define LED_PM_TYPE_BLINK			(0x00010000)
#define LED_PM_MASK					(0xFFFFFFFF)

enum {
	LED_PM_STATE_OFF = 0,
	LED_PM_STATE_ON,
	NUM_LED_PM_STAT
};
#define LEDS_PM_REGSET(x)			(an32180_i2c_table_write(an32180_ctx->this_client, x, ARRAY_SIZE(x)))

/* register */
#define LEDS_REGSET(x)				(an32180_reg_table_write(an32180_ctx->this_client, x, ARRAY_SIZE(x)))

/* I2C */
#define LED_AN32180_I2C_DEVNAME		("leds,leds_i2c")

#define LEDS_PWM_DEFAULTSTAT		(0)
#define LEDS_PWM_PAUSE_MAX			(100)

/* GPIO */
#define LED_AN32180_DEVNAME			("leds,leds_an32180")
#define LED_GPIO_CTL_LOW			(0)
#define LED_GPIO_CTL_HIGH			(1)

#define LED_DAFAULT_MAXBRIGHTNESS	(255)
#define LED_DAFAULT_PAUSE_HILO		(5)
#define LEDS_BLINK_NUM				AN32180_COLOR_MAX

#define LED_DIM_OFF					(0)
#define LED_DIM_ON					(1)

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
/* PM */
struct leds_pm_status {
	int power_status;
	unsigned int users;
};

struct leds_pm_status ledpm_status;

static const struct i2c_device_id an32180_i2c_id[] = {
	{ LED_AN32180_I2C_DEVNAME, 0 },
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id leds_system_dt_match[] = {
	{ .compatible = LED_AN32180_I2C_DEVNAME, },
	{}
};
#endif /* CONFIG_OF */

static struct i2c_driver an32180_i2c_driver = {
	.driver = {
		.name = LED_AN32180_I2C_DEVNAME,
#ifdef CONFIG_OF
		.of_match_table = leds_system_dt_match,
#endif /* CONFIG_OF */
	},
	.class    = I2C_CLASS_HWMON,
	.probe    = an32180_i2c_probe,
	.id_table = an32180_i2c_id,
	.remove   = an32180_i2c_remove,
};

typedef struct an32180_data_tag
{
	struct i2c_client *this_client;

	int initialized;
	int rst_gpio;
	unsigned char reg_raddr;
	int write_reg_judg;
	int blinking;
	int pause_hi;
	int pause_lo;
	unsigned char brightness[AN32180_LED_MAX][AN32180_COLOR_MAX];
	unsigned char blink_brightness[AN32180_LED_MAX][AN32180_COLOR_MAX];
	int dim;
	bool from_kernel;
} an32180_i2c_data_t;

struct leds_led_data {
	struct led_classdev	cdev;
	int			no;
	int			color;
	int			num_leds;
	int			max_current;
};

static an32180_i2c_data_t *an32180_ctx = NULL;

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
int shled_api_set_brightness(struct shled_tri_led *led)
{
	int ret = 0;
	int no = 0, color = 0;
	unsigned char request_value[AN32180_COLOR_MAX] = {0, 0, 0};

	if (led == NULL) {
		pr_err("%s led is NULL.\n", __func__);
		return -EPERM;
	}

	if ((led->red == 0) && (led->green == 0) && (led->blue == 0)) {
		an32180_ctx->from_kernel = false;
	} else {
		an32180_ctx->from_kernel = true;
	}

	request_value[0] = led->red;
	request_value[1] = led->green;
	request_value[2] = led->blue;

	for (no = 0; no < AN32180_LED_MAX; no++) {
		for (color = 0; color < AN32180_COLOR_MAX; color++) {
			ret = an32180_set_brightness(request_value[color], no, color);
			if (ret != 0) {
				return SHLED_RESULT_FAILURE;
			}
		}
	}

	return 0;
}

static int __init an32180_init(void)
{
	int ret = 0;
	pr_debug("%s in\n", __func__);

	ret = an32180_led_register_driver();
	if (ret != 0) {
		pr_err("%s <ERROR>register_driver err.\n", __func__);
		return ret;
	}

	pr_debug("%s out\n", __func__);
	return ret;
}
module_init(an32180_init);

static void an32180_exit(void)
{
	an32180_i2c_exit();
}
module_exit(an32180_exit);

/* sleep */
static void an32180_msleep(int msec)
{
	pr_debug("%s wait start. expect = %u ms. caller = %pS\n", __func__,
				 msec, __builtin_return_address(0));
	msleep(msec);
	pr_debug("%s wait end.\n", __func__);
}

/* etc */
static int an32180_set_brightness(unsigned char value, int no, int color)
{
	int ret = 0;
	unsigned int user = 0;
	unsigned char write_addr = 0;
	unsigned char write_value = 0;

	pr_debug("%s in value:%d, no:%d, color:%d\n", __func__, value, no, color);

	if ((value < 0x00) || (value > 0xFF)) {
		pr_err("%s <ERROR>out of value.\n", __func__);
		return -EINVAL;
	}
	if ((no < 0x00) || (no > AN32180_LED_MAX)) {
		pr_err("%s <ERROR>out of no.\n", __func__);
		return -EINVAL;
	}
	if ((color < 0x00) || (color > AN32180_COLOR_MAX)) {
		pr_err("%s <ERROR>out of color.\n", __func__);
		return -EINVAL;
	}

	if (value == an32180_ctx->brightness[no][color]) {
		pr_debug("%s equal value. brightness:%d\n", __func__, value);
		return 0;
	}

	user = AN32180_PM_MAKE_USER(no, color);

	if (value != 0) {
		if (an32180_ctx->brightness[no][color] == 0) {
			an32180_pm_power_manager(user, LED_PM_STATE_ON);
		}
	}

	write_addr = leds_reg_get_dtaddr[no][color];
	write_value = an32180_correct_dim(value);
	ret = an32180_reg_write(an32180_ctx->this_client, write_addr, write_value);
	if (ret < 0) {
		goto set_brightness_fail;
	}
	an32180_ctx->brightness[no][color] = value;

	if (value == 0) {
		an32180_pm_power_manager(user, LED_PM_STATE_OFF);
	}

	pr_debug("%s out\n", __func__);
	return 0;

set_brightness_fail:
	if (value != 0) {
		if (an32180_ctx->brightness[no][color] == 0) {
			an32180_pm_power_manager(user, LED_PM_STATE_OFF);
		}
	}
	return -EIO;
}

static int an32180_set_blink(struct device *dev, int blinking)
{
	int ret = 0;
	int i = 0;
	int chk_blink_bright = 0;
	leds_regsetting_t w_blink_bright[LEDS_BLINK_NUM];
	unsigned char *blink_bright = &(an32180_ctx->blink_brightness[0][0]);
	const unsigned char *reg_addr = &(leds_reg_get_dtaddr[0][0]);

	memset(&w_blink_bright, 0, sizeof(w_blink_bright));

	switch (blinking) {
	case 1:
		for (i = 0; i < LEDS_BLINK_NUM; i++) {
			if (*blink_bright != 0) {
				chk_blink_bright++;
			}
			w_blink_bright[i].data = an32180_correct_dim(*blink_bright);
			w_blink_bright[i].addr = *reg_addr;
			blink_bright++;
			reg_addr++;
		}
		if (chk_blink_bright == 0) {
			pr_debug("%s blink_brightness is all zero.\n", __func__);
			ret = -EPERM;
			break;
		}

		an32180_pm_power_manager(LED_PM_TYPE_BLINK, LED_PM_STATE_ON);
		ret = LEDS_REGSET(w_blink_bright);
		if (ret < 0) {
			goto set_blink_fail;
		}
#ifdef CONFIG_PWM_LED
		ret = shub_api_enable_pwm(1);
		if (ret != 0) {
			pr_err("%s <ERROR>enable_pwm err. ret:%d\n", __func__, ret);
			goto set_blink_fail;
		}
#endif /* CONFIG_PWM_LED */
		break;

	case 0:
#ifdef CONFIG_PWM_LED
		ret = shub_api_enable_pwm(0);
		if (ret != 0) {
			pr_err("%s <ERROR>enable_pwm err. ret:%d\n", __func__, ret);
		} else {
			ret = an32180_set_pwm(an32180_ctx->pause_hi, an32180_ctx->pause_lo, LEDS_PWM_DEFAULTSTAT);
			if (ret != 0) {
				pr_err("%s <ERROR>force_stop_pwm err. ret:%d\n", __func__, ret);
			}
		}
#endif /* CONFIG_PWM_LED */
		an32180_pm_power_manager(LED_PM_TYPE_BLINK, LED_PM_STATE_OFF);
		break;

	default:
		pr_err("%s invalid blinking. value:%d\n", __func__, blinking);
		ret = -EPERM;
	}

	return ret;

set_blink_fail:
	an32180_pm_power_manager(LED_PM_TYPE_BLINK, LED_PM_STATE_OFF);
	return ret;
}

static unsigned char an32180_correct_dim(unsigned char value)
{
	unsigned char ret = value;

	if (value == 0) {
		goto correct_dim_exit;
	}

	if (an32180_ctx->dim == LED_DIM_ON) {
		ret = ((long)((value + 2) * 10) / 30);
	}
correct_dim_exit:
	pr_debug("%s: dim:%d  value:0x%02X to dim:0x%02X\n", __func__, an32180_ctx->dim, value, ret);

	return ret;
}

static int an32180_set_pwm(int pwm_hi, int pwm_lo, int pwm_stat)
{
	int ret = 0;
#ifdef CONFIG_PWM_LED
	struct shub_pwm_param pwm_param;

	if ((pwm_hi < 1) || (pwm_hi > 99)) {
		pr_err("%s <ERROR>out of PWM high.\n", __func__);
		return -EPERM;
	}
	if ((pwm_lo < 1) || (pwm_lo > 99)) {
		pr_err("%s <ERROR>out of PWM low.\n", __func__);
		return -EPERM;
	}
	if ((pwm_stat < 0) || (pwm_stat > 1)) {
		pr_err("%s <ERROR>out of PWM default.\n", __func__);
		return -EPERM;
	}

	memset(&pwm_param, 0, sizeof(pwm_param));
	pwm_param.high  = (uint8_t)pwm_hi;
	pwm_param.total = (uint8_t)(pwm_hi + pwm_lo);
	pwm_param.defaultStat = (uint8_t)pwm_stat;
	ret = shub_api_set_param_pwm(&pwm_param);
	if (ret != 0) {
		pr_err("%s <ERROR>set_param_pwm err. ret:%d\n", __func__, ret);
	}
#endif /* CONFIG_PWM_LED */

	return ret;
}

/* PM */
static int an32180_pm_init(unsigned int user)
{
	pr_debug("%s: in\n",__func__);
	if (user == LED_PM_TYPE_NONE) {
		ledpm_status.power_status = LED_PM_STATE_OFF;
		ledpm_status.users        = LED_PM_TYPE_NONE;
	} else {
		ledpm_status.power_status = LED_PM_STATE_ON;
		ledpm_status.users        = user;
	}

	pr_debug("%s out users:0x%08X, power_status:%d\n", __func__,
			 ledpm_status.users, ledpm_status.power_status);

	return 0;
}

static int an32180_pm_is_active(void)
{
	return ledpm_status.power_status;
}

static void an32180_pm_set_active(unsigned int user)
{
	int mode = LED_MODE_FIX;
	int ret = 0;

	an32180_gpio_set_value(an32180_ctx->rst_gpio, LED_GPIO_CTL_HIGH);

	if (user & LED_PM_TYPE_BLINK) {
		mode = LED_MODE_BLINK;
	}

	if (mode == LED_MODE_FIX) {
		ret = LEDS_PM_REGSET(leds_an32180_init1);
	} else {
		ret = LEDS_PM_REGSET(leds_an32180_blink_init1);
	}
	return;
}

static void an32180_pm_set_standby(void)
{
	an32180_gpio_set_value(an32180_ctx->rst_gpio, LED_GPIO_CTL_LOW);
	return;
}

static int an32180_pm_power_manager(unsigned int user, int onoff)
{
	unsigned int users_wk = 0;

	pr_debug("%s in [LED_PM] users:0x%08X, pow_status:%d, user:0x%08X, onoff:%d\n", __func__,
			 ledpm_status.users, ledpm_status.power_status,
			 user, onoff);

	if ((user != (user & LED_PM_MASK)) || (user == LED_PM_TYPE_NONE)) {
		pr_err("%s invalid user argument. user:0x%08X\n", __func__, user);
		return -EINVAL;
	}

	switch (onoff) {
	case LED_PM_STATE_ON:
		switch (ledpm_status.power_status) {
		case LED_PM_STATE_OFF:
			an32180_pm_set_active(user);
			ledpm_status.power_status = LED_PM_STATE_ON;
			break;
		}
		break;

	case LED_PM_STATE_OFF:
		switch (ledpm_status.power_status) {
		case LED_PM_STATE_ON:
			users_wk = ledpm_status.users;
			users_wk &= (~(user & LED_PM_MASK));
			if (users_wk == LED_PM_TYPE_NONE) {
				an32180_pm_set_standby();
				ledpm_status.power_status = LED_PM_STATE_OFF;
			}
			break;
		}
		break;

	default:
		pr_err("%s invalid onoff argument. onoff:%d\n", __func__, onoff);
		return -EINVAL;
	}

	if (onoff == LED_PM_STATE_ON) {
		ledpm_status.users |= user & LED_PM_MASK;
	} else if (onoff == LED_PM_STATE_OFF) {
		ledpm_status.users &= (~(user & LED_PM_MASK));
	}

	pr_debug("%s out [LED_PM] users:0x%08X, power_status:%d\n", __func__,
			 ledpm_status.users, ledpm_status.power_status);

	return 0;
}

/* register */
static int an32180_reg_read(struct i2c_client *client, unsigned char reg, unsigned char *value)
{
	int ret = 0;

	if (an32180_pm_is_active() == LED_PM_STATE_OFF) {
		pr_warn("%s LED IC is power off. request is denied.\n", __func__);
		return -EIO;
	}

	ret = an32180_i2c_read(client, reg);
	pr_debug("%s addr:0x%02x, data:0x%02x\n", __func__, reg, ret);
	if (ret < 0) {
		pr_err("%s <ERROR>i2c read err. ret:%d\n", __func__, ret);
		return ret;
	}

	*value = ret;

	return 0;
}

static int an32180_reg_table_write(struct i2c_client *client, const leds_regsetting_t *regtable, int size)
{
	int ret = 0;
	int i = 0;

	if (an32180_pm_is_active() == LED_PM_STATE_OFF) {
		pr_warn("%s LED IC is power off. request is denied.\n", __func__);
		return -EIO;
	}

	if (regtable == NULL) {
		pr_err("%s regtable is NULL.\n", __func__);
		return -EPERM;
	}

	pr_debug("%s table_size:%d\n", __func__, size);
	for (i = 0; i < size; i++) {
		ret = an32180_i2c_write(client, regtable[i].addr, regtable[i].data);
		pr_debug("%s ret:%d, addr:0x%02x, data:0x%02x\n", __func__, ret, regtable[i].addr, regtable[i].data);
		if (ret < 0) {
			pr_err("%s <ERROR>i2c table_write err. ret:%d\n", __func__, ret);
			return ret;
		}

		if (regtable[i].wait != 0) {
			an32180_msleep(regtable[i].wait);
		}
	}

	return 0;
}

static int an32180_reg_write(struct i2c_client *client, unsigned char reg, unsigned char value)
{
	int ret = 0;

	if (an32180_pm_is_active() == LED_PM_STATE_OFF) {
		pr_warn("%s LED IC is power off. request is denied.\n", __func__);
		return -EIO;
	}

	ret = an32180_i2c_write(client, reg, value);
	pr_debug("%s ret:%d, addr:0x%02x, data:0x%02x\n", __func__, ret, reg, value);
	if (ret < 0) {
		pr_err("%s <ERROR>i2c write err. ret:%d\n", __func__, ret);
	}
	return ret;
}

/* I2C */
static int an32180_i2c_init(void)
{
	int ret = i2c_add_driver(&an32180_i2c_driver);

	if (ret < 0) {
		pr_err("%s <RESULT_FAILURE>i2c_add_driver.\n", __func__);
		return ret;
	}

	return 0;
}

static int an32180_i2c_exit(void)
{
	i2c_del_driver(&an32180_i2c_driver);

	return 0;
}

static int an32180_i2c_read(struct i2c_client *client, unsigned char reg)
{
	if (an32180_ctx->initialized == false) {
		pr_err("%s <ERROR>i2c probe err.\n", __func__);
		return -EINVAL;
	}

	return i2c_smbus_read_byte_data(client, reg);
}

static int an32180_i2c_write(struct i2c_client *client, unsigned char reg, unsigned char value)
{
	if (an32180_ctx->initialized == false) {
		pr_err("%s <ERROR>i2c probe err.\n", __func__);
		return -EINVAL;
	}

	return i2c_smbus_write_byte_data(client, reg, value);
}

static int an32180_i2c_table_write(struct i2c_client *client, const leds_regsetting_t **regtable, int size)
{
	int ret = 0;
	int i = 0;

	if (regtable == NULL) {
		pr_err("%s regtable is NULL.\n", __func__);
		return -EPERM;
	}

	pr_debug("%s table_size:%d\n", __func__, size);
	for (i = 0; i < size; i++) {
		if (regtable[i] == NULL) {
			pr_err("%s regtable is NULL.\n", __func__);
			return -EPERM;
		}
		ret = an32180_i2c_write(client, regtable[i]->addr, regtable[i]->data);
		pr_debug("%s ret:%d, addr:0x%02x, data:0x%02x\n", __func__, ret, regtable[i]->addr, regtable[i]->data);
		if (ret < 0) {
			pr_err("%s <ERROR>i2c table_write err. ret:%d\n", __func__, ret);
			return ret;
		}

		if (regtable[i]->wait != 0) {
			an32180_msleep(regtable[i]->wait);
		}
	}

	return 0;
}

static int an32180_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	an32180_i2c_data_t *i2c_p = NULL;

	pr_debug("%s in\n", __func__);

	if (an32180_ctx != NULL) {
		return -EPERM;
	}

	i2c_p = (an32180_i2c_data_t *)kzalloc(sizeof(an32180_i2c_data_t), GFP_KERNEL);
	if (i2c_p == NULL) {
		return -ENOMEM;
	}

	an32180_ctx = i2c_p;

	i2c_set_clientdata(client, i2c_p);
	i2c_p->this_client = client;

	i2c_p->initialized = false;

	i2c_p->pause_hi = LED_DAFAULT_PAUSE_HILO;
	i2c_p->pause_lo = LED_DAFAULT_PAUSE_HILO;

	pr_debug("%s out\n", __func__);
	return 0;
}

static int an32180_i2c_remove(struct i2c_client *client)
{
	an32180_i2c_data_t *i2c_p = i2c_get_clientdata(client);

	an32180_pm_set_standby();

	kfree(i2c_p);
	an32180_ctx = NULL;

	return 0;
}

/* GPIO */
static int an32180_gpio_set_value(int num, int value)
{
	if (!gpio_is_valid(num)) {
		pr_err("%s <ERROR>gpio probe err.\n", __func__);
		return -EINVAL;
	}

	if ((value != LED_GPIO_CTL_LOW) && (value != LED_GPIO_CTL_HIGH)) {
		pr_err("%s <ERROR>out of value. value:%d\n", __func__, value);
		return -EINVAL;
	}

	pr_debug("%s num:%d, value:%d\n", __func__, num, value);
	gpio_set_value(num, value);

	if (value == LED_GPIO_CTL_HIGH) {
		an32180_msleep(3);
	}
	if (value == LED_GPIO_CTL_LOW) {
		an32180_msleep(1);
	}

	return 0;
}

static int an32180_gpio_get_value(int num)
{
	int ret = 0;

	if (!gpio_is_valid(num)) {
		pr_err("%s <ERROR>gpio probe err.\n", __func__);
		return -EINVAL;
	}

	ret = gpio_get_value(num);

	pr_debug("%s num:%d, ret:%d\n", __func__, num, ret);
	return ret;
}

static void an32180_led_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct leds_led_data *led = container_of(led_cdev, struct leds_led_data, cdev);

#if defined(CONFIG_ANDROID_ENGINEERING)
	if (an32180_ctx->from_kernel == true) {
		return;
	}
#endif /* CONFIG_ANDROID_ENGINEERING */

	if (led != NULL) {
		an32180_set_brightness(value, led->no, led->color);
	}

	return;
}

static enum led_brightness an32180_led_get(struct led_classdev *led_cdev)
{
	struct leds_led_data *led = container_of(led_cdev, struct leds_led_data, cdev);
	unsigned char read_addr = 0;
	unsigned char data = 0;

	if (led != NULL) {
		if (an32180_ctx->initialized == false) {
			data = an32180_ctx->brightness[led->no][led->color];
			goto an32180_led_get_exit;
		}

		switch (an32180_ctx->dim) {
		case LED_DIM_OFF:
			read_addr = leds_reg_get_dtaddr[led->no][led->color];
			an32180_reg_read(an32180_ctx->this_client, read_addr, &data);
			break;
		case LED_DIM_ON:
			data = an32180_ctx->brightness[led->no][led->color];
			break;
		}
	}

an32180_led_get_exit:
	return (int)data;
}

static ssize_t an32180_blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long data = 0;
	ssize_t ret = -EINVAL;
	int blink_ret = 0;

	pr_debug("%s in\n", __func__);

	ret = kstrtoul(buf, 10, &data);
	if (ret) {
		return ret;
	}

	if ((data < 0) || (data > 1)) {
		pr_err("%s <ERROR>out of value.\n", __func__);
		return count;
	}

	if ((int)data == an32180_ctx->blinking) {
		pr_debug("%s equal value. blinking:%d\n", __func__, an32180_ctx->blinking);
		return count;
	}

	blink_ret = an32180_set_blink(dev, (int)data);
	if (blink_ret == 0) {
		an32180_ctx->blinking = (int)data;
	}

	pr_debug("%s out blink:%d\n", __func__, an32180_ctx->blinking);

	return count;
}

static ssize_t an32180_blink_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", an32180_ctx->blinking);
}

static ssize_t an32180_blink_brightness_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct leds_led_data *led = container_of(led_cdev, struct leds_led_data, cdev);
	unsigned long data = 0;
	ssize_t ret = -EINVAL;
	unsigned char write_addr = 0;
	int reg_ret = 0;

	if (led == NULL) {
		pr_err("%s container is NULL.\n", __func__);
		goto blink_brightness_fail;
	}

	pr_debug("%s in name:%s, no:%d, color:%d\n", __func__, led->cdev.name, led->no, led->color);

	ret = kstrtoul(buf, 10, &data);
	if (ret) {
		return ret;
	}

	if ((unsigned char)data == an32180_ctx->blink_brightness[led->no][led->color]) {
		pr_debug("%s equal value. blink_brightness:%d\n", __func__, an32180_ctx->blink_brightness[led->no][led->color]);
		return count;
	}

	if (an32180_ctx->blinking) {
		write_addr = leds_reg_get_dtaddr[led->no][led->color];
		reg_ret = an32180_reg_write(an32180_ctx->this_client, write_addr, an32180_correct_dim((unsigned char)data));
		if (reg_ret < 0) {
			goto blink_brightness_fail;
		}
	}
	an32180_ctx->blink_brightness[led->no][led->color] = (unsigned char)data;

	pr_debug("%s out blink_brightness:%d\n", __func__, an32180_ctx->blink_brightness[led->no][led->color]);

blink_brightness_fail:
	return count;
}

static ssize_t an32180_blink_brightness_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct leds_led_data *led = container_of(led_cdev, struct leds_led_data, cdev);
	unsigned char data = 0;

	if (led != NULL) {
		data = an32180_ctx->blink_brightness[led->no][led->color];
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t an32180_pause_lo_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long data = 0;
	ssize_t ret = -EINVAL;
	int pause_lo = 0, pause_hi = 0;
	int pwm_ret = 0;

	pr_debug("%s in\n", __func__);

	ret = kstrtoul(buf, 10, &data);
	if (ret) {
		return ret;
	}

	if ((data < 1) || (data > 99)) {
		pr_err("%s <ERROR>out of value.\n", __func__);
		return count;
	}

	pause_hi = an32180_ctx->pause_hi;
	if (((int)data + pause_hi) > LEDS_PWM_PAUSE_MAX) {
		pr_err("%s <ERROR>out of total value.\n", __func__);
		return count;
	}
	pause_lo = (int)data;

	if (pause_lo == an32180_ctx->pause_lo) {
		pr_debug("%s equal value. pause_lo:%d\n", __func__, pause_lo);
		return count;
	}

	pwm_ret = an32180_set_pwm(pause_hi, pause_lo, LEDS_PWM_DEFAULTSTAT);
	if (pwm_ret == 0) {
		an32180_ctx->pause_lo = pause_lo;
	}

	return count;
}

static ssize_t an32180_pause_lo_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int pwm_lo = 0;
#ifdef CONFIG_PWM_LED
	struct shub_pwm_param pwm_param;

	memset(&pwm_param, 0, sizeof(pwm_param));
	shub_api_get_param_pwm(&pwm_param);
	pwm_lo = (int)(pwm_param.total - pwm_param.high);
#endif /* CONFIG_PWM_LED */

	return snprintf(buf, PAGE_SIZE, "%d\n", pwm_lo);
}

static ssize_t an32180_pause_hi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	ssize_t ret = -EINVAL;
	unsigned long data = 0;
	int pause_lo = 0, pause_hi = 0;
	int pwm_ret = 0;

	pr_debug("%s in\n", __func__);

	ret = kstrtoul(buf, 10, &data);
	if (ret) {
		return ret;
	}

	if ((data < 1) || (data > 99)) {
		pr_err("%s <ERROR>out of value.\n", __func__);
		return count;
	}

	pause_lo = an32180_ctx->pause_lo;
	if (((int)data + pause_lo) > LEDS_PWM_PAUSE_MAX) {
		pr_err("%s <ERROR>out of total value.\n", __func__);
		return count;
	}
	pause_hi = (int)data;

	if (pause_hi == an32180_ctx->pause_hi) {
		pr_debug("%s equal value. pause_hi:%d\n", __func__, pause_hi);
		return count;
	}

	pwm_ret = an32180_set_pwm(pause_hi, pause_lo, LEDS_PWM_DEFAULTSTAT);
	if (pwm_ret == 0) {
		an32180_ctx->pause_hi = pause_hi;
	}

	return count;
}

static ssize_t an32180_pause_hi_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int pwm_hi = 0;
#ifdef CONFIG_PWM_LED
	struct shub_pwm_param pwm_param;

	memset(&pwm_param, 0, sizeof(pwm_param));
	shub_api_get_param_pwm(&pwm_param);
	pwm_hi = (int)(pwm_param.high);
#endif /* CONFIG_PWM_LED */

	return snprintf(buf, PAGE_SIZE, "%d\n", pwm_hi);
}

static ssize_t an32180_dim_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	ssize_t ret = -EINVAL;
	unsigned long data = 0;

	pr_debug("%s in\n", __func__);

	ret = kstrtoul(buf, 10, &data);
	if (ret) {
		return ret;
	}

	switch (data) {
	case LED_DIM_OFF:
	case LED_DIM_ON:
		break;
	default:
		pr_err("%s <ERROR>out of value.\n", __func__);
		return count;
	}

	if ((int)data == an32180_ctx->dim) {
		pr_debug("%s equal value. dim:%d\n", __func__, an32180_ctx->dim);
		return count;
	}

	an32180_ctx->dim = (int)data;
	pr_debug("%s out dim:%d\n", __func__, an32180_ctx->dim);

	return count;
}

static ssize_t an32180_dim_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", an32180_ctx->dim);
}

#if defined(CONFIG_ANDROID_ENGINEERING)
static ssize_t an32180_write_reg_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long store_data = 0;
	unsigned char w_addr = 0;
	unsigned char w_data = 0;
	ssize_t ret = -EINVAL;

	pr_debug("%s in\n", __func__);

	an32180_ctx->write_reg_judg = false;

	ret = kstrtoul(buf, 16, &store_data);
	if (ret) {
		return ret;
	}

	w_data = store_data & 0xFF;
	w_addr = (store_data >> 8) & 0xFF;

	if ((w_addr < LEDS_REG_RST) || (w_addr > LEDS_REG_D4)) {
		pr_err("%s <ERROR>out of address.\n", __func__);
		goto w_reg_fail;
	}

	ret = an32180_reg_write(an32180_ctx->this_client, w_addr, w_data);
	if (ret < 0) {
		goto w_reg_fail;
	}
	an32180_ctx->write_reg_judg = true;

	pr_debug("%s out addr:0x%02X, data:0x%02X\n", __func__, w_addr, w_data);

w_reg_fail:
	return count;
}

static ssize_t an32180_write_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	pr_debug("%s in judg:%d\n", __func__, an32180_ctx->write_reg_judg);

	return snprintf(buf, PAGE_SIZE, "%s\n", (an32180_ctx->write_reg_judg ? "OK" : "NG"));
}

static ssize_t an32180_read_reg_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long r_addr = 0;
	ssize_t ret = -EINVAL;

	pr_debug("%s in\n", __func__);

	an32180_ctx->reg_raddr = 0x00;

	ret = kstrtoul(buf, 16, &r_addr);
	if (ret) {
		return ret;
	}

	if ((r_addr < LEDS_REG_RST) || (r_addr > LEDS_REG_D4)) {
		pr_err("%s <ERROR>out of address.\n", __func__);
		goto r_reg_fail;
	}

	an32180_ctx->reg_raddr = (unsigned char)r_addr;

	pr_debug("%s out addr:0x%02X\n", __func__, an32180_ctx->reg_raddr);

r_reg_fail:
	return count;
}

static ssize_t an32180_read_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char addr = an32180_ctx->reg_raddr;
	unsigned char data = 0;

	pr_debug("%s in addr:0x%02X\n", __func__, addr);

	if ((addr < LEDS_REG_RST) || (addr > LEDS_REG_D4)) {
		pr_err("%s <ERROR>out of address.\n", __func__);
		goto r_reg_fail;
	}

	ret = an32180_reg_read(an32180_ctx->this_client, addr, &data);
	if (ret < 0) {
		goto r_reg_fail;
	}

	pr_debug("%s out data:0x%02X\n", __func__, data);
	return snprintf(buf, PAGE_SIZE, "%02X,%02X,%s\n", addr, data, "OK");


r_reg_fail:
	data = 0;
	return snprintf(buf, PAGE_SIZE, "%02X,%02X,%s\n", addr, data, "NG");
}
#endif /* CONFIG_ANDROID_ENGINEERING */

static void an32180_set_clrvari(unsigned char color)
{
	int index = 0;
	int tbl_size = ARRAY_SIZE(leds_an32180_clrvari_tbl);

	for (index = 0; index < tbl_size ; index++) {
		if (leds_an32180_clrvari_tbl[index].handset_color == color) {
			break;
		}
	}
	if (index >= tbl_size) {
		index = 0;
	}

	reg_mtxon.data  = leds_an32180_clrvari_tbl[index].imax;

	reg_b1.data = leds_an32180_clrvari_tbl[index].brt_b;
	reg_b2.data = leds_an32180_clrvari_tbl[index].brt_b;
	reg_b3.data = leds_an32180_clrvari_tbl[index].brt_b;

	reg_c1.data = leds_an32180_clrvari_tbl[index].brt_c;
	reg_c2.data = leds_an32180_clrvari_tbl[index].brt_c;
	reg_c3.data = leds_an32180_clrvari_tbl[index].brt_c;

	reg_d1.data = leds_an32180_clrvari_tbl[index].brt_d;
	reg_d2.data = leds_an32180_clrvari_tbl[index].brt_d;
	reg_d3.data = leds_an32180_clrvari_tbl[index].brt_d;
}

static unsigned char an32180_get_boot_ctx_handset_color(void)
{
	sharp_smem_common_type *sh_smem_common = NULL;
	unsigned char handset_color = 0;


	sh_smem_common = (sharp_smem_common_type *)sh_smem_get_common_address();

	if (sh_smem_common != NULL) {
		handset_color = sh_smem_common->conf_clrvari[0];
	}
	return handset_color;
}

static void an32180_ctx_init(void)
{
	unsigned char handset_color = 0;
	an32180_ctx->reg_raddr = 0;
	an32180_ctx->write_reg_judg = false;
	an32180_ctx->blinking = 0;
	memset(an32180_ctx->brightness,       0, sizeof(an32180_ctx->brightness));
	memset(an32180_ctx->blink_brightness, 0, sizeof(an32180_ctx->blink_brightness));
	an32180_ctx->dim = LED_DIM_OFF;
	an32180_ctx->from_kernel = false;

	handset_color = an32180_get_boot_ctx_handset_color();
	an32180_set_clrvari(handset_color);
	pr_debug("%s handset_color:%d\n", __func__, handset_color);
}

static void an32180_gpio_init(struct platform_device *pdev)
{
	struct pinctrl *pinctrl = NULL;
	struct pinctrl_state *state_active = NULL;

	gpio_direction_output(an32180_ctx->rst_gpio, LED_GPIO_CTL_LOW);

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		pr_err("leds_rst pinctrl not specified\n");
		return;
	}
	state_active = pinctrl_lookup_state(pinctrl, "leds_rst_active");
	if (IS_ERR_OR_NULL(state_active)) {
		pr_err("leds_rst gpio_state_active not specified\n");
		return;
	}
	pinctrl_select_state(pinctrl, state_active);
}

static void an32180_takeover_leds_status(void)
{
	int no = 0, color = 0;
	unsigned char read_addr = 0;
	int read_reg = 0;
	unsigned int user = 0;

	for (no = 0; no < AN32180_LED_MAX; no++) {
		for (color = 0; color < AN32180_COLOR_MAX; color++) {
			read_addr = leds_reg_get_dtaddr[no][color];
			read_reg = i2c_smbus_read_byte_data(an32180_ctx->this_client, read_addr);
			pr_debug("%s no:%d, color:%d, addr:0x%02x, dat:0x%02x\n", __func__, no, color, read_addr, read_reg);
			if (read_reg > 0) {
				an32180_ctx->brightness[no][color] = read_reg;
				user = user | AN32180_PM_MAKE_USER(no, color);
			} else {
				if (read_reg != 0) {
					pr_err("%s <ERROR>i2c read err. ret:%d\n", __func__, read_reg);
				}
			}
		}
	}
	an32180_pm_init(user);
}

static void an32180_takeover_boot_status(struct platform_device *pdev)
{
	int check_gpio = 0;

	gpio_request(an32180_ctx->rst_gpio, "an32180_reset");
	check_gpio = an32180_gpio_get_value(an32180_ctx->rst_gpio);
	pr_debug("%s: check_gpio:%d\n", __func__, check_gpio);
	switch (check_gpio) {
	case LED_GPIO_CTL_LOW:
		an32180_gpio_init(pdev);
		break;
	case LED_GPIO_CTL_HIGH:
		an32180_takeover_leds_status();
		break;
	default:
		break;
	}
}

static DEVICE_ATTR(blink, 0664, an32180_blink_show, an32180_blink_store);
static DEVICE_ATTR(blink_brightness, 0664, an32180_blink_brightness_show, an32180_blink_brightness_store);
static DEVICE_ATTR(pause_lo, 0664, an32180_pause_lo_show, an32180_pause_lo_store);
static DEVICE_ATTR(pause_hi, 0664, an32180_pause_hi_show, an32180_pause_hi_store);

static struct attribute *blink_attrs[] = {
	&dev_attr_blink.attr,
	&dev_attr_blink_brightness.attr,
	&dev_attr_pause_lo.attr,
	&dev_attr_pause_hi.attr,
	NULL
};

static const struct attribute_group blink_attr_group = {
	.attrs = blink_attrs,
};

static DEVICE_ATTR(dim, 0664, an32180_dim_show, an32180_dim_store);
static struct attribute *dim_attrs[] = {
	&dev_attr_dim.attr,
	NULL
};

static const struct attribute_group dim_attr_group = {
	.attrs = dim_attrs,
};

#if defined(CONFIG_ANDROID_ENGINEERING)
static DEVICE_ATTR(write_reg, 0664, an32180_write_reg_show, an32180_write_reg_store);
static DEVICE_ATTR(read_reg, 0664, an32180_read_reg_show, an32180_read_reg_store);

static struct attribute *debug_attrs[] = {
	&dev_attr_write_reg.attr,
	&dev_attr_read_reg.attr,
	NULL
};

static const struct attribute_group debug_attr_group = {
	.attrs = debug_attrs,
};
#endif /* CONFIG_ANDROID_ENGINEERING */

static int an32180_probe(struct platform_device *pdev)
{
	int rc = 0, ret = 0;
	int num_leds = 0, parsed_leds = 0;
	int pause_lo = 0, pause_hi = 0;
	struct leds_led_data *led = NULL, *led_array = NULL;
	struct device_node *node = NULL, *temp = NULL;

	pr_debug("%s in pdev:0x%p\n", __func__, pdev);

	ret = an32180_i2c_init();
	if (ret != 0) {
		return ret;
	}

	if (pdev) {
		if (&(pdev->dev) != NULL) {
			an32180_ctx->rst_gpio = of_get_named_gpio(pdev->dev.of_node, "leds_rst_gpio", 0);
			if (!gpio_is_valid(an32180_ctx->rst_gpio)) {
				pr_err("%s rst gpio not specified.\n", __func__);
			} else {
				pr_debug("%s rst gpio succusess! gpio_no:%d\n", __func__, an32180_ctx->rst_gpio);
			}
		} else {
			pr_err("%s pdev->dev is NULL.\n", __func__);
		}
	} else {
		pr_err("%s pdev is NULL.\n", __func__);
	}

	an32180_ctx_init();

	an32180_takeover_boot_status(pdev);

	node = pdev->dev.of_node;
	if (node == NULL) {
		pr_err("%s node NULL.\n", __func__);
		return -ENODEV;
	}

	temp = NULL;
	while ((temp = of_get_next_child(node, temp))) {
		num_leds++;
	}

	if (!num_leds) {
		return -ECHILD;
	}
	pr_debug("%s chk num_leds:%d\n", __func__, num_leds);

	led_array = devm_kzalloc(&pdev->dev,
		(sizeof(struct leds_led_data) * num_leds), GFP_KERNEL);
	if (!led_array) {
		pr_err("%s Unable to allocate memory.\n", __func__);
		return -ENOMEM;
	}

	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->num_leds = num_leds;

		rc = of_property_read_string(temp, "label", &led->cdev.name);
		if (rc < 0) {
			pr_err("%s Failure reading label. rc:%d\n", __func__, rc);
		}

		rc = of_property_read_u32(temp, "no", &led->no);
		if (rc < 0) {
			pr_err("%s Failure reading led no. rc:%d\n", __func__, rc);
		}

		rc = of_property_read_u32(temp, "color", &led->color);
		if (rc < 0) {
			pr_err("%s Failure reading led color. rc:%d\n", __func__, rc);
		}

		rc = of_property_read_u32(temp, "max-current", &led->max_current);
		if (rc < 0) {
			pr_err("%s Failure reading max_current. rc:%d\n", __func__, rc);
			led->max_current = LED_DAFAULT_MAXBRIGHTNESS;
		}
		led->cdev.max_brightness = led->max_current;
		led->cdev.usr_brightness_req = an32180_ctx->brightness[led->no][led->color];

		rc = of_property_read_u32(temp, "pause-hi", &pause_hi);
		if (rc < 0) {
			pr_debug("%s Failure reading pause_hi. rc:%d\n", __func__, rc);
		} else {
			an32180_ctx->pause_hi = pause_hi;
		}
		rc = of_property_read_u32(temp, "pause-lo", &pause_lo);
		if (rc < 0) {
			pr_debug("%s Failure reading pause_lo. rc:%d\n", __func__, rc);
		} else {
			an32180_ctx->pause_lo = pause_lo;
		}

		led->cdev.brightness_set = an32180_led_brightness_set;
		led->cdev.brightness_get = an32180_led_get;

		rc = led_classdev_register(&pdev->dev, &led->cdev);
		if (rc) {
			pr_err("%s unable to register. led:%s, rc:%d\n", __func__, led->cdev.name, rc);
		}
		pr_debug("%s rc:%d led_reg name:%s\n", __func__, rc, led->cdev.name);

		if ((!rc) && (&led->cdev.dev->kobj != NULL)) {
			rc = sysfs_create_group(&led->cdev.dev->kobj,
				&blink_attr_group);
			if (rc) {
				pr_err("%s error sysfs blink_attr_group.\n", __func__);
			}

			rc = sysfs_create_group(&led->cdev.dev->kobj,
				&dim_attr_group);
			if (rc) {
				pr_err("%s error sysfs dim_attr_group.\n", __func__);
			}

#if defined(CONFIG_ANDROID_ENGINEERING)
			rc = sysfs_create_group(&led->cdev.dev->kobj,
				&debug_attr_group);
			if (rc) {
				pr_err("%s error sysfs debug_attr_group.\n", __func__);
			}
#endif /* CONFIG_ANDROID_ENGINEERING */
		} else {
			pr_err("%s: kobj NULL!!\n", __func__);
		}

		parsed_leds++;
	}

	an32180_ctx->initialized = true;

	pr_debug("%s out\n", __func__);
	return 0;
}
static const struct of_device_id leds_an32180_dt_match[] = {
	{ .compatible = LED_AN32180_DEVNAME, },
	{}
};

static struct platform_driver led_an32180_driver = {
	.probe = an32180_probe,
	.remove = NULL,
	.shutdown = NULL,
	.driver = {
		/*
		 * Driver name must match the device name added in
		 * platform.c.
		 */
		.name = LED_AN32180_DEVNAME,
		.of_match_table = leds_an32180_dt_match,
	},
};

static int an32180_led_register_driver(void)
{
	return platform_driver_register(&led_an32180_driver);
}

MODULE_AUTHOR("SHARP CORPORATION");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("AN32180A 4x4dot matrix LED");
