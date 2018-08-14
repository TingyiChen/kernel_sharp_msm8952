/* drivers/sharp/shsys/sh_gpiodump.c
 *
 * Copyright (C) 2015 Sharp Corporation
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
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/io.h>
//#include <mach/msm_iomap.h>
//#include <mach/gpiomux.h>
#include <asm/uaccess.h>
#include <asm/io.h>
//#include <mach/io.h>
#include "../../../../arch/arm/mach-msm/include/mach/gpiomux.h"
#include <sharp/sh_gpio.h>
//#include <sharp/sh_systime.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>

/*
 * Register Address
 */
// LINUX/android/bootable/bootloader/lk/platform/msm8952/include/platform/iomap.h 
#define TLMM_BASE_ADDR                     0x1000000
#define SH_GPIO_CFG(n)    	(TLMM_BASE_ADDR + (n)*0x1000)
#define SH_GPIO_IN_OUT(n) 	(TLMM_BASE_ADDR + 0x00000004 + (n)*0x1000)
#define SH_GPIO_INTR_CFG(n) (TLMM_BASE_ADDR + 0x00000008 + (n)*0x1000)

#define HWIO_GPIO_CFGn_GPIO_OE_BMSK          0x200
#define HWIO_GPIO_CFGn_GPIO_OE_SHFT          0x9
#define HWIO_GPIO_CFGn_DRV_STRENGTH_BMSK     0x1c0
#define HWIO_GPIO_CFGn_DRV_STRENGTH_SHFT     0x6
#define HWIO_GPIO_CFGn_FUNC_SEL_BMSK         0x3c
#define HWIO_GPIO_CFGn_FUNC_SEL_SHFT         0x2
#define HWIO_GPIO_CFGn_GPIO_PULL_BMSK        0x3
#define HWIO_GPIO_CFGn_GPIO_PULL_SHFT        0


#define HWIO_GPIO_IN_OUTn_GPIO_OUT_BMSK      0x2
#define HWIO_GPIO_IN_OUTn_GPIO_OUT_SHFT      0x1
#define HWIO_GPIO_IN_OUTn_GPIO_IN_BMSK       0x1
#define HWIO_GPIO_IN_OUTn_GPIO_IN_SHFT       0

#define HWIO_GPIO_INTR_CFGn_INTR_RAW_STATUS_EN_BMSK    0x8
#define HWIO_GPIO_INTR_CFGn_INTR_RAW_STATUS_EN_SHFT    0x3
#define HWIO_GPIO_INTR_CFGn_INTR_DECT_CTL_BMSK         0xC
#define HWIO_GPIO_INTR_CFGn_INTR_DECT_CTL_SHFT         0x2
#define HWIO_GPIO_INTR_CFGn_INTR_POL_CTL_BMSK          0x2
#define HWIO_GPIO_INTR_CFGn_INTR_POL_CTL_SHFT          0x1
#define HWIO_GPIO_INTR_CFGn_INTR_ENABLE_BMSK           0x1
#define HWIO_GPIO_INTR_CFGn_INTR_ENABLE_SHFT           0

/*
 * LOCAL
 */
#define SYSPROBE_GPIO_DUMP_GPIOINT_TRIG_DECT_LOW_HIGH  0x0
#define SYSPROBE_GPIO_DUMP_GPIOINT_TRIG_DECT_RISING    0x1
#define SYSPROBE_GPIO_DUMP_GPIOINT_TRIG_DECT_FALLING   0x2
#define SYSPROBE_GPIO_DUMP_GPIOINT_TRIG_DECT_DUAL      0x3

/*
 * AMSS/MDM/modem_proc/core/sharp/shsys/src/sysprobe_gpio_dump.c
 * AMSS/MDM/modem_proc/core/sharp/shsys/src/sysprobe_gpio_config.c
 */
/* --------------------------------------------------------------------------
 *  Types
 * --------------------------------------------------------------------------*/
typedef enum
{
  HAL_TLMM_INPUT  = 0,
  HAL_TLMM_OUTPUT = 1,
}HAL_tlmm_DirType;

typedef enum
{
  HAL_TLMM_USER_1 = 0,
  HAL_TLMM_USER_2,
  HAL_TLMM_OUTPUT_LOW,
  HAL_TLMM_OUTPUT_HIGH,
  HAL_TLMM_OUTPUT_SZ = 0x7FFFFFFF,
}HAL_tlmm_OutputValueType;

/*
 * See LINUX/android/hardware/sharp/shsys/sysprobe/sysprobe_v01.h
 */
typedef enum {
  SYSPROBE_GPIOINT_TRIGGER_MIN_ENUM_VAL_V01 = -2147483647, /**< To force a 32 bit signed enum.  Do not change or use*/
  SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_HIGH_V01 = 0, 
  SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_LOW_V01 = 1, 
  SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_RISING_V01 = 2, 
  SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_FALLING_V01 = 3, 
  SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_DUAL_V01 = 4, 
  SYSPROBE_GPIOINT_TRIGGER_MAX_ENUM_VAL_V01 = 2147483647 /**< To force a 32 bit signed enum.  Do not change or use*/
}sysprobe_gpioint_trigger_v01;

static uint32_t sh_gpio_reg_read(unsigned int physaddr)
{
    int ret = 0;
    void __iomem *regadr = ioremap_nocache(physaddr, 4);

    if (regadr == NULL) {
        printk("sh_gpio_reg_read: ioremap_nocache ret error\n");
        return 0;
    }

    ret = ioread32(regadr);
    iounmap(regadr);

    return ret;
}

static void sh_gpio_reg_write(uint32_t config, unsigned int physaddr)
{
    void __iomem *regadr = ioremap_nocache(physaddr, 4);

    if (regadr == NULL) {
        printk("sh_gpio_reg_write: ioremap_nocache ret error\n");
        return;
    }

    iowrite32(config, regadr);
    iounmap(regadr);

    return;
}

/* 
 * source-fle-name : AMSS/MDM/modem_proc/core/sharp/shsys/src/sysprobe_gpio_dump.c
 * function-name   : void sysprobe_gpio_dump_get_sysprobe_gpio_info(int gpio, sysprobe_gpio_info_v01 *info)
 */
static void sh_gpio_info(int gpio, struct sh_gpio_read_write *info)
{
    uint32_t config, in_out , intr ;
    uint32_t val, enable , detect, polarity;
    unsigned int address;

    if (info == NULL) {
        return;
    };

    info->gpio_config.gpio = gpio;

    /*
     * GPIO configuration and control register
     * GPIO_CFGn
     */
    address =  SH_GPIO_CFG(gpio);
    config = sh_gpio_reg_read(address);
    info->gpio_config.func = ((config & HWIO_GPIO_CFGn_FUNC_SEL_BMSK) >> HWIO_GPIO_CFGn_FUNC_SEL_SHFT);
    info->gpio_config.dir = ((config & HWIO_GPIO_CFGn_GPIO_OE_BMSK) >> HWIO_GPIO_CFGn_GPIO_OE_SHFT);
    info->gpio_config.pull = ((config & HWIO_GPIO_CFGn_GPIO_PULL_BMSK) >> HWIO_GPIO_CFGn_GPIO_PULL_SHFT);
    info->gpio_config.drvstr = ((config & HWIO_GPIO_CFGn_DRV_STRENGTH_BMSK) >> HWIO_GPIO_CFGn_DRV_STRENGTH_SHFT);

    /*
     * GPIO_IN_OUTn
     */
    address = SH_GPIO_IN_OUT(gpio);
    in_out = sh_gpio_reg_read(address);
    if (info->gpio_config.dir == HAL_TLMM_INPUT) {
        val = ((in_out & HWIO_GPIO_IN_OUTn_GPIO_IN_BMSK) >> HWIO_GPIO_IN_OUTn_GPIO_IN_SHFT);
    }
    else {
        val = ((in_out & HWIO_GPIO_IN_OUTn_GPIO_OUT_BMSK) >> HWIO_GPIO_IN_OUTn_GPIO_OUT_SHFT);
    }
    if (val == 0) {
        info->gpio_config.outval = HAL_TLMM_OUTPUT_LOW;
    }
    else {
        info->gpio_config.outval = HAL_TLMM_OUTPUT_HIGH;
    }

   /*
    * GPIO_INTR_CFGn
    */
    
    address = SH_GPIO_INTR_CFG(gpio);
    intr = sh_gpio_reg_read(address);
    
    /*
     * GPIO_INTR_CFGn
     */
    enable = ( ( intr & HWIO_GPIO_INTR_CFGn_INTR_ENABLE_BMSK ) >> HWIO_GPIO_INTR_CFGn_INTR_ENABLE_SHFT);

    /* INTR_ENABLE */
    if ( enable == 0 ) {
        info->gpioint_info.enable = 0;
    }
    else {
        info->gpioint_info.enable = 1;
    }

    /* INTR_DECT_CTL,INTR_POL_CTL */
    detect   = ( ( intr & HWIO_GPIO_INTR_CFGn_INTR_DECT_CTL_BMSK ) >> HWIO_GPIO_INTR_CFGn_INTR_DECT_CTL_SHFT ) ;
    polarity = ( ( intr & HWIO_GPIO_INTR_CFGn_INTR_POL_CTL_BMSK  ) >> HWIO_GPIO_INTR_CFGn_INTR_POL_CTL_SHFT ) ;

    switch (detect) {
        case SYSPROBE_GPIO_DUMP_GPIOINT_TRIG_DECT_LOW_HIGH:
            if (polarity == 0) {
                info->gpioint_info.trigger = SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_LOW_V01;
            }
            else {
                info->gpioint_info.trigger = SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_HIGH_V01;
            }
            break;

        case SYSPROBE_GPIO_DUMP_GPIOINT_TRIG_DECT_RISING:
            info->gpioint_info.trigger = SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_RISING_V01;
            break;

        case SYSPROBE_GPIO_DUMP_GPIOINT_TRIG_DECT_FALLING:
            info->gpioint_info.trigger = SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_FALLING_V01;
            break;

        case SYSPROBE_GPIO_DUMP_GPIOINT_TRIG_DECT_DUAL:
            info->gpioint_info.trigger = SYSPROBE_GPIO_DUMP_GPIOINT_TRIGGER_DUAL_V01;
            break;
    }
}

static int sh_gpio_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int sh_gpio_ioctl_read(unsigned long arg)
{
    int rc = 0;
    struct sh_gpio_read_write gpio_req;

    if (arg == 0) {
        return -EINVAL;
    }

    if (copy_from_user(&gpio_req, (void __user *)arg, sizeof(gpio_req)) != 0) {
        return -EINVAL;
    }

	sh_gpio_info(gpio_req.gpio_config.gpio, &gpio_req) ;

	//printk("read: [%02d] func(%d) dir(%d) pull(%d) drvstr(%d) outval(%d) ena(%d) trg(%d)\n"
	//	,(int)gpio_req.gpio_config.gpio
	//	,(int)gpio_req.gpio_config.func
	// 	,(int)gpio_req.gpio_config.dir
	// 	,(int)gpio_req.gpio_config.pull
	// 	,(int)gpio_req.gpio_config.drvstr
	// 	,(int)gpio_req.gpio_config.outval
	// 	,(int)gpio_req.gpioint_info.enable
	// 	,(int)gpio_req.gpioint_info.trigger);

    if (copy_to_user((u8*)arg, (u8*)&gpio_req, sizeof(gpio_req)) != 0) {
        rc = -EFAULT;
    }

    return rc;
}

static int sh_gpio_ioctl_write(unsigned long arg)
{
    struct sh_gpio_read_write gpio_req;
    struct sh_gpio_read_write current_info;
    struct sh_gpio_read_write resp_config;
    uint32_t gpio, func, dir, pull, drvstr, outval, config;
    int rc = 0 ;
    unsigned int address;

    if (arg == 0) {
        return -EINVAL;
    }

    if (copy_from_user(&gpio_req, (void __user *)arg, sizeof(gpio_req)) != 0) {
        printk("sh_gpio_ioctl_write: copy form user error\n");
        return -EINVAL;
    }

    /*
     * Get Current GPIO Value
     */
    sh_gpio_info(gpio_req.gpio_config.gpio, &current_info) ;

    /*
     * Setting GPIO Value
     */
    gpio = gpio_req.gpio_config.gpio ;

    /* Function Value */
    if (gpio_req.flag & SH_GPIO_CONFIG_BIT_FLAG_FUNC_V01) {
        func = gpio_req.gpio_config.func;
    }
    else {
        func = current_info.gpio_config.func;
    }

    /* Direction Value  */
    if (gpio_req.flag & SH_GPIO_CONFIG_BIT_FLAG_DIR_V01) {
        dir = gpio_req.gpio_config.dir;
    }
    else {
        dir = current_info.gpio_config.dir;
    }

    /* Pull Value */
    if (gpio_req.flag & SH_GPIO_CONFIG_BIT_FLAG_PULL_V01) {
        pull = gpio_req.gpio_config.pull;
    }
    else {
        pull = current_info.gpio_config.pull;
    }

    /* DriverStrength Value */
    if (gpio_req.flag & SH_GPIO_CONFIG_BIT_FLAG_DRIVE_V01) {
        drvstr = gpio_req.gpio_config.drvstr;
    }
    else {
        drvstr = current_info.gpio_config.drvstr;
    }

    /*
     * Write GPIO Value
     */
    config = (dir << HWIO_GPIO_CFGn_GPIO_OE_SHFT) | (drvstr << HWIO_GPIO_CFGn_DRV_STRENGTH_SHFT) | (func << HWIO_GPIO_CFGn_FUNC_SEL_SHFT) | (pull << HWIO_GPIO_CFGn_GPIO_PULL_SHFT);

    /* OutValue */
    if (gpio_req.flag & SH_GPIO_CONFIG_BIT_FLAG_RMT_V01) {
        outval = gpio_req.gpio_config.outval;
        if (func == GPIOMUX_FUNC_GPIO) {                    /* GPIO Mode */
            config |= outval > GPIOMUX_IN ? BIT(HWIO_GPIO_CFGn_GPIO_OE_SHFT) : 0;     /* Output Enable ON */
        }
    }
    else {
        outval = current_info.gpio_config.outval;
    }

    address = SH_GPIO_CFG(gpio);
    sh_gpio_reg_write(config, address);

    if (func == GPIOMUX_FUNC_GPIO) {
        if (gpio_req.flag & SH_GPIO_CONFIG_BIT_FLAG_RMT_V01) {
            address = SH_GPIO_IN_OUT(gpio);
            sh_gpio_reg_write(dir == GPIOMUX_OUT_HIGH ? BIT(HWIO_GPIO_IN_OUTn_GPIO_OUT_SHFT) : 0, address);
        }
    }

    mb();

    /*
     * Response
     */
    sh_gpio_info(gpio, &resp_config) ;
    if (copy_to_user((u8*)arg, (u8*)&resp_config, sizeof(resp_config)) != 0) {
        printk("sh_gpio_ioctl_write: copy to user error\n");
        rc = -EFAULT;
    }

    return rc;
}

static int sh_gpio_ioctl_write_pmic_gpio(unsigned long arg)
{
    int rc = 0;
    struct sh_qpnp_gpio gpio_req;
	struct qpnp_pin_cfg	pin_cfg;

    if (arg == 0) {
        printk("sh_gpio_ioctl_write_pmic_gpio: paramter error (%d line)\n", __LINE__);
        return -EINVAL;
    }

    if (copy_from_user(&gpio_req, (void __user *)arg, sizeof(struct sh_qpnp_gpio)) != 0) {
        printk("sh_gpio_ioctl_write_pmic_gpio: copy_from_user() error (%d line)\n", __LINE__);
        return -EINVAL;
    }

	pin_cfg.mode		= gpio_req.mode			;
	pin_cfg.output_type	= gpio_req.output_type	;
	pin_cfg.invert		= gpio_req.invert		;
	pin_cfg.pull		= gpio_req.pull			;
	pin_cfg.vin_sel		= gpio_req.vin_sel		;
	pin_cfg.out_strength= gpio_req.out_strength;
	pin_cfg.src_sel		= gpio_req.src_sel		;
	pin_cfg.master_en	= gpio_req.master_en	;
	pin_cfg.aout_ref	= gpio_req.aout_ref	;
	pin_cfg.ain_route	= gpio_req.ain_route	;
	pin_cfg.cs_out		= gpio_req.cs_out		;


	rc = gpio_request( gpio_req.gpio , "sysprobe" );
	if ( rc != -EBUSY ) {
		gpio_free(gpio_req.gpio);
		rc = gpio_request( gpio_req.gpio , "sysprobe");
		if ( rc ) {
        	printk("sh_gpio_ioctl_write_pmic_gpio: gpio_request() error rc(%d)\n", rc);
			return rc ;
		}
	}

	if ( pin_cfg.mode  == QPNP_PIN_MODE_DIG_IN  ) {
		rc = gpio_direction_input(gpio_req.gpio );
	} else
	if ( pin_cfg.mode  == QPNP_PIN_MODE_DIG_OUT ) {
		rc = gpio_direction_output(gpio_req.gpio , gpio_req.out_strength) ;
	}
	if ( rc ) {
       	printk("sh_gpio_ioctl_write_pmic_gpio: gpio_direction_xxxx() error rc(%d)\n", rc);
		return rc ;
	}

	if ( gpio_cansleep(gpio_req.gpio) ) {
		/* Effectively */
		gpio_set_value_cansleep(gpio_req.gpio , gpio_req.out_strength );
	} else {
		/* Invalidity */
		gpio_set_value(gpio_req.gpio , gpio_req.out_strength );
	}

	//rc = qpnp_pin_config(gpio_req.gpio , &pin_cfg);
	//if ( rc ) {
    //    printk("sh_gpio_ioctl_write_pmic_gpio: qpnp_pin_config() error (%d)\n", rc);
	//}

	/*
	 * Because GPIO using is deleted, I do not do the free of GPIO
	 *
	 * gpio_free(gpio_req.gpio);
	 */

    return rc;
}
static long sh_gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int rc = 0;

    switch (cmd) {
    case SH_GPIO_READ:
        rc = sh_gpio_ioctl_read(arg);
        break;
    case SH_GPIO_WRITE:
        rc = sh_gpio_ioctl_write(arg);
        break;
   case SH_GPIO_WRITE_PMIC_GPIO:
        rc = sh_gpio_ioctl_write_pmic_gpio(arg);
        break;
    default:
        printk("sh_gpio_ioctl: paramter error %d\n", cmd);
        rc = -EPERM;
        break;
    }
    return rc;
}

static int sh_gpio_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static struct file_operations sh_gpio_fops = {
    .owner          = THIS_MODULE,
    .open           = sh_gpio_open,
    .release        = sh_gpio_release,
    .unlocked_ioctl = sh_gpio_ioctl,
};

static struct miscdevice sh_gpio_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sh_gpio",
    .fops = &sh_gpio_fops,
};

static int __init sh_gpio_init( void )
{
    int ret;

    ret = misc_register(&sh_gpio_dev);
    if (ret != 0) {
        printk("sh_gpio_init: fail to misc_register ret %d\n", ret);
    }

    return ret;
}

module_init(sh_gpio_init);

MODULE_DESCRIPTION("sh_gpio");
MODULE_LICENSE("GPL v2");

