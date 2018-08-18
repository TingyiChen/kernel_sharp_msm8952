/* drivers/sharp/shdisp/shdisp_sazabi.c  (Display Driver)
 *
 * Copyright (C) 2015 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/msm_mdp.h>
#include <linux/clk.h>
#include "shdisp_panel.h"
#include "shdisp_sazabi.h"
#include "shdisp_system.h"
#include "shdisp_io.h"
#ifdef SHDISP_IR2E71Y8
#include "shdisp_bdic.h"
#endif /* SHDISP_IR2E71Y8 */
#include "shdisp_type.h"
#ifdef SHDISP_IR2E71Y8
#include "shdisp_pm.h"
#endif /* SHDISP_IR2E71Y8 */
#include "shdisp_dbg.h"
#include "shdisp_kerl_priv.h"
#include <sharp/sh_smem.h>
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_POWER_MODE_CHK
#define SHDISP_SAZABI_VIDEO_STOP
#define SHDISP_SAZABI_VDDIGPIO_ID  "sazabi_vddi_gpio"
#define SHDISP_SAZABI_VDDA_ID      "sazabi_vdda"
#define SHDISP_SAZABI_RST_ID       "sazabi_rst_gpio"
#define SHDISP_SAZABI_MIPIERR_ID   "sazabi_mipierr_gpio"
#define SHDISP_SAZABI_POWER_MODE_ADDR   (0x0A)


#define VCOM_H_POS   (0)
#define VCOM1_POS    (1)
#define VCOM2_POS    (2)
#define VCOMOFF_POS  (3)
#define VCOM_POS_MAX (4)

#define SHDISP_SAZABI_VCOM_ADDR         (0xC6)
#define IS_FLICKER_ADJUSTED(param)      (((param & 0xF000) == 0x9000) ? 1 : 0)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
struct shdisp_sazabi_calc_vcom_in {
    unsigned short vcom;
};

struct shdisp_sazabi_calc_vcom_out {
    char vcom_h;
    char vcom1;
    char vcom2;
    char vcomoff;
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_sazabi_API_init_io(struct shdisp_panel_context *panel_ctx);
static int shdisp_sazabi_API_exit_io(void);
static int shdisp_sazabi_API_power_on(int mode);
static int shdisp_sazabi_API_power_off(int mode);
static int shdisp_sazabi_API_disp_on(void);
static int shdisp_sazabi_API_disp_off(void);
static int shdisp_sazabi_API_start_display(void);
static int shdisp_sazabi_API_check_recovery(void);
static int shdisp_sazabi_API_resister_write(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_sazabi_API_resister_read(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_sazabi_API_diag_set_flicker_param(struct shdisp_diag_flicker_param flicker_param);
static int shdisp_sazabi_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *flicker_param);
static int shdisp_sazabi_API_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_sazabi_API_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_sazabi_API_set_gmm(struct shdisp_diag_gamma *gmm);
static int shdisp_sazabi_API_set_irq(int enable);

static void shdisp_sazabi_hw_reset(bool);
static int shdisp_sazabi_probe(struct platform_device *pdev);
static int shdisp_sazabi_remove(struct platform_device *pdev);
static int shdisp_sazabi_register_driver(void);
static int shdisp_sazabi_cmd_lcd_on(void);
static int shdisp_sazabi_cmd_lcd_off(void);
static int shdisp_sazabi_cmd_display_on(void);
static void shdisp_sazabi_calc_vcom_param(struct shdisp_sazabi_calc_vcom_in in, struct shdisp_sazabi_calc_vcom_out *out);
static void shdisp_sazabi_init_flicker_param(unsigned short vcom);
static int shdisp_sazabi_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param);
static int shdisp_sazabi_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param);
static int shdisp_sazabi_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_sazabi_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_sazabi_set_gmm(struct shdisp_diag_gamma *gmm);
static int shdisp_sazabi_set_gmm_and_voltage(void);
static int shdisp_sazabi_set_gmmtable_param(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_sazabi_set_gmm_param(unsigned short *gmm_in, char *gmm_out);
static int shdisp_sazabi_set_voltage_param(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_sazabi_set_voltage_initseq(void);
static int shdisp_sazabi_get_gmmtable(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_sazabi_get_gmm(unsigned char addr, unsigned short *gmm);
static int shdisp_sazabi_get_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_sazabi_chk_gmm_chksum(struct shdisp_lcddr_phy_gmm_reg *phy_gmm);
static int shdisp_sazabi_init_phy_gmm(struct shdisp_lcddr_phy_gmm_reg *phy_gmm);

static int shdisp_panel_sazabi_reg_read(unsigned char addr, unsigned char *out_data);
static int shdisp_sazabi_vddi_power_on(void);
static int shdisp_sazabi_vddi_power_off(void);
static int shdisp_sazabi_vdda_power_on(void);
static int shdisp_sazabi_vdda_power_off(void);
static int shdisp_sazabi_vcom_setting(void);
static int shdisp_sazabi_gmm_setting(bool divide);
static int shdisp_sazabi_read_power_mode(unsigned char addr);
#if defined(CONFIG_ANDROID_ENGINEERING)
static void shdisp_sazabi_check_dbg_disp_on_read_err(int * ret);
static void shdisp_sazabi_check_dbg_disp_on_err(unsigned char * read_data);
static void shdisp_sazabi_check_dbg_det_low_err(int * ret);
#else /* CONFIG_ANDROID_ENGINEERING */
#define shdisp_sazabi_check_dbg_disp_on_read_err(ret);
#define shdisp_sazabi_check_dbg_disp_on_err(read_data);
#define shdisp_sazabi_check_dbg_det_low_err(ret);
#endif /* CONFIG_ANDROID_ENGINEERING */
static int shdisp_sazabi_is_succed_display(void);
static void shdisp_sazabi_err_output(char code, char subcode);

static int shdisp_sazabi_digital_power_on(void);
static int shdisp_sazabi_analog_power_on(void);
static int shdisp_sazabi_power_on_reset_and_initial_setting(void);
static int shdisp_sazabi_display_on(void);
static int shdisp_sazabi_display_on_check(void);
static int shdisp_sazabi_display_off(void);
static int shdisp_sazabi_analog_power_off(void);
static int shdisp_sazabi_reset_low(void);
static int shdisp_sazabi_digital_power_off(void);
static int shdisp_sazabi_reset_high(void);
static int shdisp_sazabi_cog_sleep_out(struct timespec *ts_start);
static int shdisp_sazabi_cog_display_on(struct timespec *ts_start);
static int shdisp_sazabi_sleep_out_init2(void);

static bool shdisp_sazabi_is_user_cmd(unsigned char addr);
static void shdisp_sazabi_workqueue_handler(struct work_struct *work);
static void shdisp_sazabi_int_isr_common(void);
static irqreturn_t shdisp_sazabi_int_isr(int irq_num, void *data);
static int shdisp_sazabi_check_mipi_err(void);
static int shdisp_sazabi_clear_dsi_error(void);

static int shdisp_sazabi_resister_write(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_sazabi_resister_read(unsigned char addr, unsigned char *read_data, unsigned char size);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct shdisp_panel_context shdisp_panel_ctx;

static struct regulator * shdisp_sazabi_vdda = NULL;
static struct shdisp_gpio_info shdisp_sazabi_vddi_gpio;
static int shdisp_sazabi_rst_gpio      = 0;
static int shdisp_sazabi_mipi_err_gpio = 0;
static int shdisp_sazabi_irq_gpio      = 0;
static struct workqueue_struct    *shdisp_wq_sazabi = NULL;
static struct work_struct         shdisp_wq_sazabi_wk;
static struct platform_device *pshdisp_sazabi_irq_port_dev = NULL;
static struct wake_lock shdisp_sazabi_wakelock;
static char shdisp_sazabi_user_cmd_addr[] = { 0x00,0x01,0x04,0x05,0x06,0x07,0x08,0x0A,0x0B,0x0D,
                                              0x0E,0x10,0x11,0x13,0x20,0x21,0x22,0x23,0x28,0x29,
                                              0x34,0x35,0x36,0x38,0x39,0x44,0x4F,0x51,0x52,0x53,
                                              0x54,0x55,0x56,0x5E,0x5F,0xDA,0xDB,0xDC };

#ifdef SHDISP_POWER_MODE_CHK
static int shdisp_sazabi_dispon_chk = SHDISP_PANEL_DISPON_CHK_INIT;
#endif /* SHDISP_POWER_MODE_CHK */

static char shdisp_sazabi_adjusted_flicker_value[VCOM_POS_MAX];
static bool reset_status = false;
/* ------------------------------------------------------------------------- */
/* EXTERN                                                                    */
/* ------------------------------------------------------------------------- */
extern int mdss_shdisp_dsi_bus_clk_ctrl(bool enable);
#ifdef SHDISP_SAZABI_VIDEO_STOP
extern void mdss_shdisp_video_transfer_ctrl(int onoff, int commit);
#endif /* SHDISP_SAZABI_VIDEO_STOP */


/* ------------------------------------------------------------------------- */
/*      packet header                                                        */
/* ------------------------------------------------------------------------- */
/*      LCD ON                                                               */
/*      Initial Setting                                                      */

#include "./data/shdisp_sazabi_data_default.h"

static struct shdisp_panel_operations shdisp_sazabi_fops = {
    shdisp_sazabi_API_init_io,
    shdisp_sazabi_API_exit_io,
    NULL,
    shdisp_sazabi_API_power_on,
    shdisp_sazabi_API_power_off,
    shdisp_sazabi_API_disp_on,
    shdisp_sazabi_API_disp_off,
    shdisp_sazabi_API_start_display,
    NULL,
    NULL,
    shdisp_sazabi_API_resister_write,
    shdisp_sazabi_API_resister_read,
    shdisp_sazabi_API_diag_set_flicker_param,
    shdisp_sazabi_API_diag_get_flicker_param,
    NULL,
    shdisp_sazabi_API_check_recovery,
    shdisp_sazabi_API_set_gmmtable_and_voltage,
    shdisp_sazabi_API_get_gmmtable_and_voltage,
    shdisp_sazabi_API_set_gmm,
    NULL,
    NULL,
    shdisp_sazabi_API_set_irq,
    NULL,
    NULL,
    NULL,
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define MIPI_DSI_COMMAND_TX(x)              (shdisp_panel_API_mipi_dsi_cmds_tx(0, x, ARRAY_SIZE(x)))
#define MIPI_DSI_COMMAND_TX_COMMIT(x)       (shdisp_panel_API_mipi_dsi_cmds_tx(1, x, ARRAY_SIZE(x)))

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_create                                                  */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_sazabi_API_create(void)
{
    return &shdisp_sazabi_fops;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_init_io                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_init_io(struct shdisp_panel_context *panel_ctx)
{
    int funcret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    memcpy(&(shdisp_panel_ctx), panel_ctx, sizeof(struct shdisp_panel_context));

#ifndef SHDISP_NOT_SUPPORT_NO_OS
    if (IS_FLICKER_ADJUSTED(shdisp_panel_ctx.vcom_nvram)) {
        shdisp_sazabi_init_flicker_param(shdisp_panel_ctx.vcom);
    }
#endif /* SHDISP_NOT_SUPPORT_NO_OS */
    shdisp_sazabi_init_phy_gmm(&(shdisp_panel_ctx.lcddr_phy_gmm));
#ifdef SHDISP_NOT_SUPPORT_NO_OS
#else  /* SHDISP_NOT_SUPPORT_NO_OS */
    shdisp_sazabi_dispon_chk = shdisp_panel_ctx.disp_on_status;
#endif /* SHDISP_NOT_SUPPORT_NO_OS */
    shdisp_wq_sazabi = create_singlethread_workqueue("shdisp_sazabi_wq");
    if (!shdisp_wq_sazabi) {
        SHDISP_ERR("failed to create_singlethread_workqueue().");
        funcret = SHDISP_RESULT_FAILURE;
        goto exit_with_error;
    }

    INIT_WORK(&shdisp_wq_sazabi_wk, shdisp_sazabi_workqueue_handler);

    wake_lock_init(&shdisp_sazabi_wakelock, WAKE_LOCK_SUSPEND, "sazabi_wake_lock");
    shdisp_sazabi_register_driver();

    if (shdisp_api_get_boot_disp_status() == SHDISP_MAIN_DISP_ON) {
        shdisp_sazabi_vddi_power_on();
        shdisp_sazabi_vdda_power_on();
        shdisp_sazabi_hw_reset(false);
    }
    goto exit;
exit_with_error:
    destroy_workqueue(shdisp_wq_sazabi);
    shdisp_wq_sazabi = NULL;

exit:

    SHDISP_TRACE("out");
    return funcret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_exit_io                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_exit_io(void)
{
    SHDISP_TRACE("in");

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_power_on                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_power_on(int mode)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in mode=%d", mode);

    ret = shdisp_sazabi_digital_power_on();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1 ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_sazabi_analog_power_on();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2 ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    }

    if (mode == SHDISP_PANEL_POWER_TP_ON) {
        ret = shdisp_sazabi_reset_high();
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out3 ret=%d", ret);
            return ret;
        }
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_power_off                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_power_off(int mode)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in mode=%d", mode);

    ret = shdisp_sazabi_reset_low();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1 ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_sazabi_analog_power_off();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2 ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_sazabi_digital_power_off();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3 ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_disp_on                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_disp_on(void)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_sazabi_cmd_lcd_on();

    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_disp_off                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_disp_off(void)
{
    int ret = 0;

    SHDISP_TRACE("in");
#ifdef SHDISP_IR2E71Y8
    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_OFF);
#endif /* SHDISP_IR2E71Y8 */
    ret = shdisp_sazabi_cmd_lcd_off();

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_start_display                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_start_display(void)
{
    int ret = 0;
    SHDISP_TRACE("in");

    ret = shdisp_sazabi_cmd_display_on();

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_resister_write                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_resister_write(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret = 0;
    ret = shdisp_sazabi_resister_write(addr, write_data, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_resister_read                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_resister_read(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret = 0;
    ret = shdisp_sazabi_resister_read(addr, read_data, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_diag_set_flicker_param                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_diag_set_flicker_param(struct shdisp_diag_flicker_param flicker_param)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_sazabi_diag_set_flicker_param_internal(flicker_param);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_sazabi_diag_set_flicker_param_internal.");
        return SHDISP_RESULT_FAILURE;
    } else {
        shdisp_sazabi_diag_set_flicker_param_ctx(flicker_param);
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_diag_get_flicker_param                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *flicker_param)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char rdbuf[VCOM_POS_MAX];

    SHDISP_TRACE("in");

    ret = MIPI_DSI_COMMAND_TX(switchtopage1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("switch to page1 error.");
        return ret;
    }

    memset(rdbuf, 0, sizeof(rdbuf));
    ret = shdisp_sazabi_resister_read(SHDISP_SAZABI_VCOM_ADDR, rdbuf, sizeof(rdbuf));

    flicker_param->master_alpha  = (rdbuf[VCOM_H_POS] & 0x01) << 8;
    flicker_param->master_alpha |= rdbuf[VCOM1_POS];

    SHDISP_TRACE("out master_alpha=0x%04X", flicker_param->master_alpha);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_check_recovery                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_check_recovery(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    ret = shdisp_sazabi_is_succed_display();

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_set_gmmtable_and_voltage                                */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    mdss_shdisp_dsi_bus_clk_ctrl(true);
    ret = shdisp_sazabi_set_gmmtable_and_voltage(gmm_info);
    mdss_shdisp_dsi_bus_clk_ctrl(false);

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_get_gmmtable_and_voltage                                */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    mdss_shdisp_dsi_bus_clk_ctrl(true);
    ret = shdisp_sazabi_get_gmmtable_and_voltage(gmm_info);
    mdss_shdisp_dsi_bus_clk_ctrl(false);

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_set_gmm                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_set_gmm(struct shdisp_diag_gamma *gmm)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    mdss_shdisp_dsi_bus_clk_ctrl(true);
    ret = shdisp_sazabi_set_gmm(gmm);
    mdss_shdisp_dsi_bus_clk_ctrl(false);

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_API_set_irq                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_API_set_irq(int enable)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in (enable=%d)", enable);

    switch (enable) {
    case SHDISP_IRQ_ENABLE:
        ret = devm_request_irq(&pshdisp_sazabi_irq_port_dev->dev,
                                   shdisp_sazabi_irq_gpio, shdisp_sazabi_int_isr,
                                   IRQF_TRIGGER_RISING, "shdisp_sazabi", NULL);
        if (ret) {
            ret = SHDISP_RESULT_FAILURE;
            SHDISP_ERR("failed to request_irq(). (ret=%d irq=%d)", ret, shdisp_sazabi_irq_gpio);
        }
        shdisp_sazabi_check_mipi_err();
        break;
    case SHDISP_IRQ_DISABLE:
        disable_irq(shdisp_sazabi_irq_gpio);
        free_irq(shdisp_sazabi_irq_gpio, NULL);
        break;
    default:
        ret = SHDISP_RESULT_FAILURE;
        SHDISP_ERR("invalid argument. (enable=%d)", enable);
    }


    SHDISP_TRACE("out (ret=%d)", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_cmd_display_on                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_cmd_display_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");
    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_cmd_lcd_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_cmd_lcd_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    ret = shdisp_sazabi_display_off();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1 ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_cmd_lcd_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_cmd_lcd_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    ret = shdisp_sazabi_power_on_reset_and_initial_setting();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1 ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_sazabi_display_on();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2 ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_sazabi_display_on_check();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3 ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_sazabi reg_read                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_sazabi_reg_read(unsigned char addr, unsigned char *out_data)
{
    struct shdisp_dsi_cmd_desc cmd[1];
    char cmd_buf[1 + 2];

    memset(cmd_buf, 0x00, sizeof(cmd_buf));
    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;

    memset(cmd, 0x00, sizeof(cmd));
    cmd[0].dtype = SHDISP_DTYPE_DCS_READ;
    cmd[0].wait = 0x00;
    cmd[0].dlen = 1;
    cmd[0].payload = cmd_buf;
    cmd[0].mode = 0;

    if (shdisp_panel_API_mipi_dsi_cmds_rx(out_data, cmd, 1) != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_hw_reset                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_sazabi_hw_reset(bool reset)
{
    SHDISP_TRACE("call reset=%d", reset);
    if (reset) {
        shdisp_IO_API_set_Host_gpio(shdisp_sazabi_rst_gpio, SHDISP_GPIO_CTL_LOW);
        shdisp_IO_API_Host_gpio_free(shdisp_sazabi_rst_gpio);
    } else {
        shdisp_IO_API_Host_gpio_request(shdisp_sazabi_rst_gpio, "PANEL_RST_N");
        shdisp_IO_API_set_Host_gpio(shdisp_sazabi_rst_gpio, SHDISP_GPIO_CTL_HIGH);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_vddi_power_on                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_vddi_power_on(void)
{
    gpio_request(shdisp_sazabi_vddi_gpio.gpio, "PANEL_VDDI_Npin");
    gpio_set_value(shdisp_sazabi_vddi_gpio.gpio, 1);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_vddi_power_off                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_vddi_power_off(void)
{
    gpio_set_value(shdisp_sazabi_vddi_gpio.gpio, 0);
    gpio_free(shdisp_sazabi_vddi_gpio.gpio);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_vdda_power_on                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_vdda_power_on(void)
{
    int res = 0;
    res = regulator_enable(shdisp_sazabi_vdda);
    if (res) {
        SHDISP_ERR("regulator_enable(shdisp_sazabi_vdda) failure. (%d)", res);
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_vdda_power_off                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_vdda_power_off(void)
{
    int res = 0;
    res = regulator_disable(shdisp_sazabi_vdda);
    if (res) {
        SHDISP_ERR("regulator_disable(shdisp_sazabi_vdda) failure. (%d)", res);
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_vcom_setting                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_vcom_setting(void)
{
    struct shdisp_dsi_cmd_req C6_write_req;
    struct shdisp_dsi_cmd_req C6_read_req;
    char wrbuffer[4];
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    if (IS_FLICKER_ADJUSTED(shdisp_panel_ctx.vcom_nvram)) {
        SHDISP_DEBUG("flicker param was adjusted.");
        memcpy(&voltage_settings_adrC6_adjusted_payloads[1], shdisp_sazabi_adjusted_flicker_value,
            sizeof(shdisp_sazabi_adjusted_flicker_value));
        ret = MIPI_DSI_COMMAND_TX_COMMIT(adjusted_setting_vcom_cmds);
    } else {
        memset(&C6_read_req, 0, sizeof(C6_read_req));
        memset(wrbuffer, 0, sizeof(wrbuffer));

        C6_read_req.dtype = SHDISP_DTYPE_GEN_READ2;
        C6_read_req.addr  = voltage_settings_adrC6_unadjust_payloads[0];
        C6_read_req.size  = sizeof(wrbuffer);
        C6_read_req.mode  = 0;
        C6_read_req.data  = wrbuffer;
        shdisp_panel_API_dsi_read_reg(&C6_read_req);

        if (wrbuffer[3] == 0x64) {
            SHDISP_DEBUG("flicker param was unadjusted.");
            memset(&C6_write_req, 0, sizeof(C6_write_req));
            wrbuffer[3] = 0x13;
            C6_write_req.dtype = SHDISP_DTYPE_GEN_LWRITE;
            C6_write_req.addr    = voltage_settings_adrC6_unadjust_payloads[0];
            C6_write_req.size    = sizeof(wrbuffer);
            C6_write_req.mode    = 0;
            C6_write_req.data    = wrbuffer;
            ret = shdisp_panel_API_dsi_write_reg(&C6_write_req);
        }
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_gmm_setting                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_gmm_setting(bool divide)
{
    int ret = SHDISP_RESULT_FAILURE;

    if (shdisp_panel_ctx.lcddr_phy_gmm.status == SHDISP_LCDDR_GMM_STATUS_OK) {
            if (divide) {
                ret = MIPI_DSI_COMMAND_TX_COMMIT(adjusted_setting_D1_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(adjusted_setting_D2_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(adjusted_setting_D3_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(adjusted_setting_D4_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(adjusted_setting_D5_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(adjusted_setting_D6_gmm_cmds);
            } else {
                ret = MIPI_DSI_COMMAND_TX_COMMIT(adjusted_setting_page1_gmm_cmds);
            }
    } else {
            if (divide) {
                ret = MIPI_DSI_COMMAND_TX_COMMIT(initial_setting_D1_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(initial_setting_D2_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(initial_setting_D3_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(initial_setting_D4_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(initial_setting_D5_gmm_cmds);
                ret |= MIPI_DSI_COMMAND_TX_COMMIT(initial_setting_D6_gmm_cmds);
            } else {
                ret = MIPI_DSI_COMMAND_TX_COMMIT(initial_setting_page1_gmm_cmds);
            }
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_voltage_setting                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_voltage_setting(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    if (shdisp_panel_ctx.lcddr_phy_gmm.status == SHDISP_LCDDR_GMM_STATUS_OK) {
        ret = MIPI_DSI_COMMAND_TX_COMMIT(adjusted_voltage_setting_cmds);
    } else {
        ret = MIPI_DSI_COMMAND_TX_COMMIT(voltage_setting_cmds);
    }
    shdisp_sazabi_set_voltage_initseq();

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_read_power_mode                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_read_power_mode(unsigned char addr)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifdef SHDISP_POWER_MODE_CHK
    unsigned char read_data = 0;
    ret = shdisp_panel_sazabi_reg_read(addr, &read_data);
    SHDISP_DEBUG("addr = 0x%02x.read_data = 0x%02x", addr, read_data);
    shdisp_sazabi_check_dbg_disp_on_read_err(&ret);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
        shdisp_sazabi_err_output(SHDISP_DBG_CODE_READ_ERROR, SHDISP_DBG_SUBCODE_DISPON_CHK);
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_sazabi_check_dbg_disp_on_err(&read_data);

    if (read_data != 0x9C) {
        SHDISP_ERR("POWER_MODE error.addr = 0x%02x.read_data = 0x%02x", addr, read_data);
        shdisp_sazabi_err_output(SHDISP_DBG_CODE_ERROR_DETECT, SHDISP_DBG_SUBCODE_DISPON_CHK);
        return SHDISP_RESULT_FAILURE;
    }
#endif /*  SHDISP_POWER_MODE_CHK */
    return ret;

}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_check_dbg_disp_on_read_err                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_sazabi_check_dbg_disp_on_read_err(int * ret)
{
    if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DISPON_READ) {
        shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force disp on read error.");
        *ret = SHDISP_RESULT_FAILURE;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_check_dbg_disp_on_err                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_sazabi_check_dbg_disp_on_err(unsigned char * read_data)
{
    if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DISPON) {
        shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force disp on error.");
        *read_data = 0xFF;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_check_dbg_det_low_err                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_sazabi_check_dbg_det_low_err(int * ret)
{
    if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DETLOW) {
        shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force lcd det low.");
        *ret = SHDISP_RESULT_FAILURE;
    }
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_is_succed_display                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_is_succed_display(void)
{
    int retrisevol = SHDISP_RESULT_SUCCESS;
    int ret = SHDISP_RESULT_SUCCESS;
    if (shdisp_sazabi_dispon_chk == SHDISP_PANEL_DISPON_CHK_INIT) {
        shdisp_sazabi_read_power_mode(SHDISP_SAZABI_POWER_MODE_ADDR);
    }

    SHDISP_DEBUG("shdisp_sazabi_dispon_chk=%d", shdisp_sazabi_dispon_chk);
    if (shdisp_sazabi_dispon_chk == SHDISP_PANEL_DISPON_CHK_NG) {
        SHDISP_ERR("dispon failed.");
        ret = SHDISP_RESULT_FAILURE;
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_DISPON_CHK);
    }
#ifdef SHDISP_IR2E71Y8
    retrisevol = shdisp_bdic_API_RECOVERY_check_restoration();
#endif /* SHDISP_IR2E71Y8 */
    shdisp_sazabi_check_dbg_det_low_err(&retrisevol);

    if (retrisevol != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_RECOVERY_check_restoration.");
        ret = SHDISP_RESULT_FAILURE;
        shdisp_sazabi_err_output(SHDISP_DBG_CODE_ERROR_DETECT, SHDISP_DBG_SUBCODE_DET_LOW);
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_DET_LOW);
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_err_output                                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_sazabi_err_output(char code, char subcode)
{
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;

    err_code.mode = SHDISP_DBG_MODE_LINUX;
    err_code.type = SHDISP_DBG_TYPE_PANEL;
    err_code.code = code;
    err_code.subcode = subcode;
    shdisp_dbg_API_err_output(&err_code, 0);
#endif  /* SHDISP_RESET_LOG */
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_digital_power_on                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_digital_power_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");
    ret = shdisp_sazabi_vddi_power_on();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1 ret=%d", ret);
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_sazabi_vdda_power_on();
    if (ret != SHDISP_RESULT_SUCCESS) {
        shdisp_sazabi_vddi_power_off();
        SHDISP_DEBUG("out2 ret=%d", ret);
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_IO_API_delay_us(1000);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_analog_power_on                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_analog_power_on(void)
{
    SHDISP_TRACE("in");
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_LCD_power_on();

    shdisp_bdic_API_LCD_m_power_on();
#endif /* SHDISP_IR2E71Y8 */
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_power_on_reset_and_initial_setting                          */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_power_on_reset_and_initial_setting(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    ret = shdisp_sazabi_reset_high();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1 ret=%d", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(initial_setting_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2 ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_display_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_display_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct timespec ts1;

    SHDISP_TRACE("in");

    ret = MIPI_DSI_COMMAND_TX_COMMIT(exit_sleep_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("exit_sleep_cmds error.");
        return ret;
    }

    getnstimeofday(&ts1);

    ret = shdisp_sazabi_cog_sleep_out(&ts1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("sleep_out error.");
        return ret;
    }

    ret = shdisp_sazabi_cog_display_on(&ts1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("disp_on error.");
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_display_on_check                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_display_on_check(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    ret = shdisp_sazabi_read_power_mode(SHDISP_SAZABI_POWER_MODE_ADDR);
    shdisp_sazabi_dispon_chk = ret == SHDISP_RESULT_SUCCESS ?
        SHDISP_PANEL_DISPON_CHK_OK : SHDISP_PANEL_DISPON_CHK_NG;

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_display_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_display_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");
    ret = MIPI_DSI_COMMAND_TX_COMMIT(disp_off_enter_sleep_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("disp_off_enter_sleep error.");
        return ret;
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_analog_power_off                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_analog_power_off(void)
{
    SHDISP_TRACE("in");
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_LCD_m_power_off();

    shdisp_bdic_API_LCD_power_off();
#endif /* SHDISP_IR2E71Y8 */
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_reset_high                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_reset_high(void)
{
    SHDISP_TRACE("in");
    if (reset_status) {
        SHDISP_DEBUG("out1 reset_status=%d", reset_status);
        return SHDISP_RESULT_SUCCESS;
    }
    reset_status = true;
    shdisp_sazabi_hw_reset(false);
    shdisp_IO_API_delay_us(1 * 1000);

    shdisp_sazabi_hw_reset(true);
    shdisp_IO_API_delay_us(10);

    shdisp_sazabi_hw_reset(false);
    shdisp_IO_API_delay_us(2 * 1000);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_cog_sleep_out                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_cog_sleep_out(struct timespec *ts_start)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct timespec ts2, ts3;
    unsigned long long wtime = 0;

    SHDISP_TRACE("in");

#ifdef SHDISP_IR2E71Y8
    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_INIT);
#endif /* SHDISP_IR2E71Y8 */

    getnstimeofday(&ts2);
    wtime = (ts2.tv_sec - ts_start->tv_sec) * 1000000;
    wtime += (ts2.tv_nsec - ts_start->tv_nsec) / 1000;
    SHDISP_PERFORMANCE("COG sleep_out wait=%lld, wtime=%llu", (SHDISP_SLEEP_OUT_WAIT- wtime), wtime);

    if (wtime < SHDISP_SLEEP_OUT_WAIT) {
        shdisp_IO_API_delay_us(SHDISP_SLEEP_OUT_WAIT - wtime);
    }

    ret = shdisp_sazabi_sleep_out_init2();
    if (ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }

    getnstimeofday(&ts3);
    wtime = (ts3.tv_sec - ts2.tv_sec) * 1000000;
    wtime += (ts3.tv_nsec - ts2.tv_nsec) / 1000;
    SHDISP_PERFORMANCE("COG sleep_out wait=%lld, wtime=%llu", (SHDISP_ALS_POW_ON_WAIT - wtime), wtime);

    if (wtime < SHDISP_ALS_POW_ON_WAIT) {
        shdisp_IO_API_delay_us(SHDISP_ALS_POW_ON_WAIT - wtime);
    }

#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_update_led_value();
    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_ON);
#endif /* SHDISP_IR2E71Y8 */
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_cog_display_on                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_cog_display_on(struct timespec *ts_start)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct timespec ts4;
    unsigned long long wtime = 0;

    SHDISP_TRACE("in");

    getnstimeofday(&ts4);
    wtime = (ts4.tv_sec - ts_start->tv_sec) * 1000000;
    wtime += (ts4.tv_nsec - ts_start->tv_nsec) / 1000;
    SHDISP_PERFORMANCE("COG wait=%lld, wtime=%llu", (SHDISP_DISP_ON_WAIT - wtime), wtime);

    if (wtime < SHDISP_DISP_ON_WAIT) {
        shdisp_IO_API_delay_us(SHDISP_DISP_ON_WAIT - wtime);
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(disp_on_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("disp_on_cmds error.");
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_sleep_out_init2                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_sleep_out_init2(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    ret = MIPI_DSI_COMMAND_TX_COMMIT(switchtopage1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("switch to page1 error.");
        return ret;
    }

    ret = shdisp_sazabi_vcom_setting();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("VCOM_setting error.");
        return ret;
    }

    ret = shdisp_sazabi_gmm_setting(false);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("initial_setting_page1_gmm error.");
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_reset_low                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_reset_low(void)
{
    SHDISP_TRACE("in");

    reset_status = false;
    shdisp_sazabi_hw_reset(true);
    shdisp_IO_API_delay_us(1 * 1000);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_digital_power_off                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_digital_power_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int vddret = SHDISP_RESULT_FAILURE;
    SHDISP_TRACE("in");
    vddret = shdisp_sazabi_vdda_power_off();
    if (vddret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1 ret=%d", vddret);
        ret = SHDISP_RESULT_FAILURE;
    }


    vddret = shdisp_sazabi_vddi_power_off();
    if (vddret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2 ret=%d", vddret);
        ret = SHDISP_RESULT_FAILURE;
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_clear_dsi_error                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_clear_dsi_error(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");
    ret = MIPI_DSI_COMMAND_TX_COMMIT(clear_dsi_error_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("failed to clear dsi errror.");
        return ret;
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_calc_vcom_param                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_sazabi_calc_vcom_param(struct shdisp_sazabi_calc_vcom_in in, struct shdisp_sazabi_calc_vcom_out *out)
{
    SHDISP_TRACE("in");
    SHDISP_DEBUG("vcom=0x%04x", in.vcom);

    out->vcom_h  = (in.vcom & 0x100) ? 0x03 : 0x00;
    out->vcom1   = in.vcom & 0xFF;
    out->vcom2   = out->vcom1;
    out->vcomoff = in.vcom/2;

    SHDISP_DEBUG("VCOM_H=0x%02x VCOM1=0x%02x VCOM2=0x%02x VCOMOFF=0x%02x",
                        out->vcom_h,
                        out->vcom1,
                        out->vcom2,
                        out->vcomoff);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_init_flicker_param                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_sazabi_init_flicker_param(unsigned short vcom)
{
    struct shdisp_sazabi_calc_vcom_in in;
    struct shdisp_sazabi_calc_vcom_out out;

    SHDISP_TRACE("in");
    SHDISP_DEBUG("vcom=0x%04x", vcom);

    in.vcom = vcom;
    shdisp_sazabi_calc_vcom_param(in, &out);

    shdisp_sazabi_adjusted_flicker_value[VCOM_H_POS]  = out.vcom_h;
    shdisp_sazabi_adjusted_flicker_value[VCOM1_POS]   = out.vcom1;
    shdisp_sazabi_adjusted_flicker_value[VCOM2_POS]   = out.vcom2;
    shdisp_sazabi_adjusted_flicker_value[VCOMOFF_POS] = out.vcomoff;

    SHDISP_DEBUG("adjusted param was stored. vcom_h=0x%02x, vcom1=0x%02x, "
                    "vcom2=0x%02x, vcomoff=0x%02x",
                    shdisp_sazabi_adjusted_flicker_value[VCOM_H_POS],
                    shdisp_sazabi_adjusted_flicker_value[VCOM1_POS],
                    shdisp_sazabi_adjusted_flicker_value[VCOM2_POS],
                    shdisp_sazabi_adjusted_flicker_value[VCOMOFF_POS]);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_diag_set_flicker_param_internal                             */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param)
{
    int ret;
    struct shdisp_sazabi_calc_vcom_in in;
    struct shdisp_sazabi_calc_vcom_out out;
    char write_data[4];

    SHDISP_TRACE("in");

    if (flicker_param.request & SHDISP_REG_WRITE) {
        in.vcom = flicker_param.master_alpha;
        shdisp_sazabi_calc_vcom_param(in, &out);
        write_data[VCOM_H_POS]  = out.vcom_h;
        write_data[VCOM1_POS]   = out.vcom1;
        write_data[VCOM2_POS]   = out.vcom2;
        write_data[VCOMOFF_POS] = out.vcomoff;

        ret = MIPI_DSI_COMMAND_TX(switchtopage1_cmds);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("switch to page1 error.");
            return ret;
        }

        ret = shdisp_sazabi_resister_write(SHDISP_SAZABI_VCOM_ADDR, write_data, sizeof(write_data));
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("vcom write err ret=%d", ret);
            return ret;
        }
    }

    if (flicker_param.request & SHDISP_SAVE_VALUE) {
        shdisp_sazabi_init_flicker_param(flicker_param.master_alpha);
    }

    if (flicker_param.request & SHDISP_RESET_VALUE) {
        shdisp_sazabi_init_flicker_param(0);
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_diag_set_flicker_param_ctx                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param)
{
    SHDISP_TRACE("in");

    if (flicker_param.request & SHDISP_SAVE_VALUE) {
        shdisp_panel_ctx.vcom = flicker_param.master_alpha;
        shdisp_panel_ctx.vcom_nvram = 0x9000 | flicker_param.master_alpha;
        SHDISP_DEBUG("stored flicker param to panel_ctx. vcom=0x%02x, vcom_nvram=0x%02x.",
                        shdisp_panel_ctx.vcom, shdisp_panel_ctx.vcom_nvram);
    }

    if (flicker_param.request & SHDISP_RESET_VALUE) {
        shdisp_panel_ctx.vcom = 0;
        shdisp_panel_ctx.vcom_nvram = 0;
        SHDISP_DEBUG("reset flicker param of panel_ctx. vcom=0x%02x, vcom_nvram=0x%02x.",
                        shdisp_panel_ctx.vcom, shdisp_panel_ctx.vcom_nvram);
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_set_gmmtable_and_voltage                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    shdisp_sazabi_set_gmmtable_param(gmm_info);
    shdisp_sazabi_set_voltage_param(gmm_info);

    shdisp_panel_ctx.lcddr_phy_gmm.status = SHDISP_LCDDR_GMM_STATUS_OK;

    ret = shdisp_sazabi_set_gmm_and_voltage();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_set_gmmtable_param                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_set_gmmtable_param(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int adr_offset = 1;

    SHDISP_TRACE("in");
    shdisp_sazabi_set_gmm_param(&gmm_info->gammaR[0],
                                adjusted_setting_page1_gmm_D1_payloads+adr_offset);
    shdisp_sazabi_set_gmm_param(&gmm_info->gammaR[SHDISP_PANEL_GMM_TBL_SIZE/2],
                                adjusted_setting_page1_gmm_D4_payloads+adr_offset);
    shdisp_sazabi_set_gmm_param(&gmm_info->gammaG[0],
                                adjusted_setting_page1_gmm_D2_payloads+adr_offset);
    shdisp_sazabi_set_gmm_param(&gmm_info->gammaG[SHDISP_PANEL_GMM_TBL_SIZE/2],
                                adjusted_setting_page1_gmm_D5_payloads+adr_offset);

    shdisp_sazabi_set_gmm_param(&gmm_info->gammaB[0],
                                adjusted_setting_page1_gmm_D3_payloads+adr_offset);
    shdisp_sazabi_set_gmm_param(&gmm_info->gammaB[SHDISP_PANEL_GMM_TBL_SIZE/2],
                                adjusted_setting_page1_gmm_D6_payloads+adr_offset);

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_set_gmm_param                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_set_gmm_param(unsigned short *gmm_in, char *gmm_out)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int i = 0;
    SHDISP_TRACE("in");

    for (i = 0; i < SHDISP_PANEL_GMM_TBL_SIZE/2; i++) {
        gmm_out[i*2] = (*(gmm_in+i) >> 8) & 0x0003;
        gmm_out[i*2+1] = (*(gmm_in+i)) & 0x00FF;
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_set_voltage_param                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_set_voltage_param(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");
    memcpy(adjusted_voltage_settings_adrB3_payloads,
        voltage_settings_adrB3_payloads, sizeof(voltage_settings_adrB3_payloads));
    memcpy(adjusted_voltage_settings_adrB4_payloads,
        voltage_settings_adrB4_payloads, sizeof(voltage_settings_adrB4_payloads));
    *ptr_vgh = gmm_info->vgh;
    *ptr_vgl = gmm_info->vgl;
    *ptr_vgho1 = voltage_settings_adrB3_payloads[VGHO1_ADR];
    *ptr_vgho2 = gmm_info->vgho2;
    *ptr_vglo1 = voltage_settings_adrB4_payloads[VGLO1_ADR];
    *ptr_vglo2 = gmm_info->vglo2;
    *ptr_vgmp2 = gmm_info->vgmp2;
    *ptr_vgmp1 = gmm_info->vgmp1;
    *ptr_vgsp = gmm_info->vgsp;
    *ptr_vgmn2 = gmm_info->vgmn2;
    *ptr_vgmn1 = gmm_info->vgmn1;
    *ptr_vgsn = gmm_info->vgsn;
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_set_voltage_initseq                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_set_voltage_initseq(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    if (shdisp_panel_ctx.lcddr_phy_gmm.status == SHDISP_LCDDR_GMM_STATUS_OK) {
        memcpy(tmp_voltage_settings_adrB1_payloads,
               adjusted_voltage_settings_adrB1_payloads,
               sizeof(tmp_voltage_settings_adrB1_payloads));
        memcpy(tmp_voltage_settings_adrB2_payloads,
               adjusted_voltage_settings_adrB2_payloads,
               sizeof(tmp_voltage_settings_adrB2_payloads));
        memcpy(tmp_voltage_settings_adrB3_payloads,
               adjusted_voltage_settings_adrB3_payloads,
               sizeof(tmp_voltage_settings_adrB3_payloads));
        memcpy(tmp_voltage_settings_adrB4_payloads,
               adjusted_voltage_settings_adrB4_payloads,
               sizeof(tmp_voltage_settings_adrB4_payloads));
        memcpy(tmp_voltage_settings_adrC3_payloads,
               adjusted_voltage_settings_adrC3_payloads,
               sizeof(tmp_voltage_settings_adrC3_payloads));
        memcpy(tmp_voltage_settings_adrC3_payloads,
               adjusted_voltage_settings_adrC3_payloads,
               sizeof(tmp_voltage_settings_adrC3_payloads));
        memcpy(tmp_voltage_settings_adrC4_payloads,
               adjusted_voltage_settings_adrC4_payloads,
               sizeof(tmp_voltage_settings_adrC4_payloads));
    } else {
        memcpy(tmp_voltage_settings_adrB1_payloads,
               voltage_settings_adrB1_payloads,
               sizeof(tmp_voltage_settings_adrB1_payloads));
        memcpy(tmp_voltage_settings_adrB2_payloads,
               voltage_settings_adrB2_payloads,
               sizeof(tmp_voltage_settings_adrB2_payloads));
        memcpy(tmp_voltage_settings_adrB3_payloads,
               voltage_settings_adrB3_payloads,
               sizeof(tmp_voltage_settings_adrB3_payloads));
        memcpy(tmp_voltage_settings_adrB4_payloads,
               voltage_settings_adrB4_payloads,
               sizeof(tmp_voltage_settings_adrB4_payloads));
        memcpy(tmp_voltage_settings_adrC3_payloads,
               voltage_settings_adrC3_payloads,
               sizeof(tmp_voltage_settings_adrC3_payloads));
        memcpy(tmp_voltage_settings_adrC4_payloads,
               voltage_settings_adrC4_payloads,
               sizeof(tmp_voltage_settings_adrC4_payloads));
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_get_gmmtable_and_voltage                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

#ifdef SHDISP_SAZABI_VIDEO_STOP
    ret = MIPI_DSI_COMMAND_TX_COMMIT(disp_off_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("disp_off_cmds error.");
        return ret;
    }
    mdss_shdisp_video_transfer_ctrl(false, false);
#endif /* SHDISP_SAZABI_VIDEO_STOP */
    ret = MIPI_DSI_COMMAND_TX_COMMIT(switchtopage1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("switch to page1 error.");
        return ret;
    }
    ret = shdisp_sazabi_get_gmmtable(gmm_info);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> gmm table read error.");
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_sazabi_get_voltage(gmm_info);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> voltage read error.");
        return SHDISP_RESULT_FAILURE;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(switchtopage0_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("switch to page0 error.");
        return ret;
    }
#ifdef SHDISP_SAZABI_VIDEO_STOP
    mdss_shdisp_video_transfer_ctrl(true, false);
    ret = MIPI_DSI_COMMAND_TX_COMMIT(disp_on_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("disp_on_cmds error.");
        return ret;
    }
#endif /* SHDISP_SAZABI_VIDEO_STOP */
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_get_gmmtable                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_get_gmmtable(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    ret = shdisp_sazabi_get_gmm(initial_setting_page1_gmm_D1_payloads[0],
                                &gmm_info->gammaR[0]);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> D1 gmm read.");
        goto shdisp_end;
    }
    ret = shdisp_sazabi_get_gmm(initial_setting_page1_gmm_D4_payloads[0],
                                &gmm_info->gammaR[SHDISP_PANEL_GMM_TBL_SIZE/2]);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> D4 gmm read.");
        goto shdisp_end;
    }
    ret = shdisp_sazabi_get_gmm(initial_setting_page1_gmm_D2_payloads[0],
                                &gmm_info->gammaG[0]);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> D2 gmm read.");
        goto shdisp_end;
    }
    ret = shdisp_sazabi_get_gmm(initial_setting_page1_gmm_D5_payloads[0],
                                &gmm_info->gammaG[SHDISP_PANEL_GMM_TBL_SIZE/2]);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> D5 gmm read.");
        goto shdisp_end;
    }
    ret = shdisp_sazabi_get_gmm(initial_setting_page1_gmm_D3_payloads[0],
                                &gmm_info->gammaB[0]);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> D3 gmm read.");
        goto shdisp_end;
    }
    ret = shdisp_sazabi_get_gmm(initial_setting_page1_gmm_D6_payloads[0],
                                &gmm_info->gammaB[SHDISP_PANEL_GMM_TBL_SIZE/2]);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> D6 gmm read.");
        goto shdisp_end;
    }
    SHDISP_TRACE("out");
    return ret;
shdisp_end:
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_get_gmm                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_get_gmm(unsigned char addr, unsigned short *gmm)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int i = 0;
    unsigned char read_buf[SHDISP_PANEL_GMM_TBL_SIZE];

    SHDISP_TRACE("in");
    memset(read_buf, 0x00, sizeof(read_buf));

    ret = shdisp_sazabi_resister_read(addr, read_buf, SHDISP_PANEL_GMM_TBL_SIZE);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> resister read.");
        return SHDISP_RESULT_FAILURE;
    }
    for(i=0; i<SHDISP_PANEL_GMM_TBL_SIZE/2; i++)
    {
        gmm[i] = ((read_buf[i*2] << 8) & 0x0300);
        gmm[i] |= (read_buf[i*2+1] & 0x00FF);
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_get_voltage                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_get_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char read_buf[SHDISP_PANEL_GMM_TBL_SIZE];

    SHDISP_TRACE("in");

    memset(read_buf, 0x00, sizeof(read_buf));
    ret = shdisp_sazabi_resister_read(voltage_settings_adrB1_payloads[0],
                                      read_buf, sizeof(voltage_settings_adrB1_payloads));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> 0xB1 read.");
        goto shdisp_end;
    }
    gmm_info->vgh = read_buf[VGH_ADR-1];

    memset(read_buf, 0x00, sizeof(read_buf));
    ret = shdisp_sazabi_resister_read(voltage_settings_adrB2_payloads[0],
                                      read_buf, sizeof(voltage_settings_adrB2_payloads));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> 0xB2 read.");
        goto shdisp_end;
    }
    gmm_info->vgl = read_buf[VGL_ADR-1];

    memset(read_buf, 0x00, sizeof(read_buf));
    ret = shdisp_sazabi_resister_read(voltage_settings_adrB3_payloads[0],
                                      read_buf, sizeof(voltage_settings_adrB3_payloads));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> 0xB3 read.");
        goto shdisp_end;
    }
    gmm_info->vgho1 = read_buf[VGHO1_ADR-1];
    gmm_info->vgho2 = read_buf[VGHO2_ADR-1];

    memset(read_buf, 0x00, sizeof(read_buf));
    ret = shdisp_sazabi_resister_read(voltage_settings_adrB4_payloads[0],
                                      read_buf, sizeof(voltage_settings_adrB4_payloads));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> 0xB4 read.");
        goto shdisp_end;
    }
    gmm_info->vglo1 = read_buf[VGLO1_ADR-1];
    gmm_info->vglo2 = read_buf[VGLO2_ADR-1];

    memset(read_buf, 0x00, sizeof(read_buf));
    ret = shdisp_sazabi_resister_read(voltage_settings_adrC3_payloads[0],
                                      read_buf, sizeof(voltage_settings_adrC3_payloads));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> 0xC3 read.");
        goto shdisp_end;
    }
    gmm_info->vgmp2 = read_buf[VGMP2_ADR-1];
    gmm_info->vgmp1 = read_buf[VGMP1_ADR-1];
    gmm_info->vgsp  = read_buf[VGSP_ADR-1];

    memset(read_buf, 0x00, sizeof(read_buf));
    ret = shdisp_sazabi_resister_read(voltage_settings_adrC4_payloads[0],
                                      read_buf, sizeof(voltage_settings_adrC4_payloads));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> 0xC4 read.");
        goto shdisp_end;
    }
    gmm_info->vgmn2 = read_buf[VGMN2_ADR-1];
    gmm_info->vgmn1 = read_buf[VGMN1_ADR-1];
    gmm_info->vgsn  = read_buf[VGSN_ADR-1];
    SHDISP_TRACE("out");
    return ret;
shdisp_end:
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_set_gmm                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_set_gmm(struct shdisp_diag_gamma *gmm)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    if (gmm == NULL) {
        SHDISP_ERR("<NULL_POINTER> gmm.");
        return SHDISP_RESULT_FAILURE;
    }
    if (gmm->level != SHDISP_LCDDR_GMM_SET_POINT_INI) {
        SHDISP_ERR("<INVALID_VALUE> gmm->level(%d).", gmm->level);
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_panel_ctx.lcddr_phy_gmm.status = SHDISP_LCDDR_GMM_STATUS_NOT_SET;
    ret = shdisp_sazabi_set_gmm_and_voltage();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out ret=%d", ret);
        return ret;
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_set_gmm_and_voltage                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_set_gmm_and_voltage(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    ret = MIPI_DSI_COMMAND_TX_COMMIT(switchtopage1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("switch to page1 error.");
        return ret;
    }
    ret = shdisp_sazabi_gmm_setting(true);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("gmm_setting err ret=%d", ret);
        return ret;
    }
    ret = shdisp_sazabi_voltage_setting();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("voltage_setting err ret=%d", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(switchtopage0_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("switch to page0 error.");
        return ret;
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_chk_gmm_chksum                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_chk_gmm_chksum(struct shdisp_lcddr_phy_gmm_reg *phy_gmm)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int i = 0;
    unsigned int checksum = 0;

    SHDISP_TRACE("in");
    if (phy_gmm == NULL) {
        SHDISP_ERR("phy_gmm is NULL.");
        return SHDISP_RESULT_FAILURE;
    }
    if (phy_gmm->status != SHDISP_LCDDR_GMM_STATUS_OK) {
        SHDISP_DEBUG("failed. status=%02x", phy_gmm->status);
        ret = SHDISP_RESULT_FAILURE;
    } else {
        checksum = phy_gmm->status;
        for (i = 0; i < SHDISP_LCDDR_PHY_GMM_BUF_MAX; i++) {
            checksum = checksum + phy_gmm->buf[i];
        }
        for (i = 0; i < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; i++) {
            checksum = checksum + phy_gmm->applied_voltage[i];
        }
        if ((checksum & 0x00FFFFFF) != phy_gmm->chksum) {
            SHDISP_DEBUG("failed. chksum=%06x calc_chksum=%06x",
                         phy_gmm->chksum, (checksum & 0x00FFFFFF));
            ret = SHDISP_RESULT_FAILURE;
        }
    }
    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_init_phy_gmm                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_init_phy_gmm(struct shdisp_lcddr_phy_gmm_reg *phy_gmm)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int index = 0;
    int gmm_size = 0;
    struct shdisp_diag_gamma_info gmm_info;

    SHDISP_TRACE("in");
    if (phy_gmm == NULL) {
        SHDISP_ERR("phy_gmm is NULL.");
        ret = SHDISP_RESULT_FAILURE;
        goto end;
    }
    ret = shdisp_sazabi_chk_gmm_chksum(phy_gmm);
    if (ret == SHDISP_RESULT_FAILURE) {
        phy_gmm->status = SHDISP_LCDDR_GMM_STATUS_NOT_SET;
        SHDISP_DEBUG("phy_gmm not adjusted");
        ret = SHDISP_RESULT_SUCCESS;
        goto end;
    }
    memset(&gmm_info, 0x00, sizeof(gmm_info));
    gmm_size = SHDISP_PANEL_GMM_TBL_SIZE * sizeof(unsigned short);
    memcpy(gmm_info.gammaR, &phy_gmm->buf[SHDISP_PANEL_GMM_TBL_SIZE*0], gmm_size);
    memcpy(gmm_info.gammaG, &phy_gmm->buf[SHDISP_PANEL_GMM_TBL_SIZE*1], gmm_size);
    memcpy(gmm_info.gammaB, &phy_gmm->buf[SHDISP_PANEL_GMM_TBL_SIZE*2], gmm_size);

    index = 0;
    gmm_info.vgh = phy_gmm->applied_voltage[index++];
    gmm_info.vgl = phy_gmm->applied_voltage[index++];
    gmm_info.vgho1 = phy_gmm->applied_voltage[index++];
    gmm_info.vgho2 = phy_gmm->applied_voltage[index++];
    gmm_info.vglo1 = phy_gmm->applied_voltage[index++];
    gmm_info.vglo2 = phy_gmm->applied_voltage[index++];
    gmm_info.vgmp2 = phy_gmm->applied_voltage[index++];
    gmm_info.vgmp1 = phy_gmm->applied_voltage[index++];
    gmm_info.vgsp = phy_gmm->applied_voltage[index++];
    gmm_info.vgmn2 = phy_gmm->applied_voltage[index++];
    gmm_info.vgmn1 = phy_gmm->applied_voltage[index++];
    gmm_info.vgsn = phy_gmm->applied_voltage[index++];

    shdisp_sazabi_set_gmmtable_param(&gmm_info);
    shdisp_sazabi_set_voltage_param(&gmm_info);
end:
    shdisp_sazabi_set_voltage_initseq();
    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_workqueue_handler                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_sazabi_workqueue_handler(struct work_struct *work)
{
    int ret;

    SHDISP_TRACE("in");

#ifdef SHDISP_SAZABI_MIPIERR_DENOISE
    if (shdisp_IO_API_get_Host_gpio(shdisp_sazabi_mipi_err_gpio)) {
#endif /* SHDISP_SAZABI_MIPIERR_DENOISE */
        SHDISP_ERR("MIPI Error");

#ifdef SHDISP_RESET_LOG
        shdisp_sazabi_err_output(SHDISP_DBG_CODE_ERROR_DETECT, SHDISP_DBG_SUBCODE_ESD_MIPI);
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_ESD_MIPI);
#endif /* SHDISP_RESET_LOG */

        shdisp_API_semaphore_start();
        shdisp_sazabi_clear_dsi_error();
        shdisp_API_semaphore_end();
        ret = shdisp_API_do_lcd_det_recovery();
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("recovery request error!! ret=%d", ret);
        }
#ifdef SHDISP_SAZABI_MIPIERR_DENOISE
    } else {
        SHDISP_DEBUG("ED: GPIO37 Low");
        enable_irq(shdisp_sazabi_irq_gpio);
    }
#endif /* SHDISP_SAZABI_MIPIERR_DENOISE */

    wake_unlock(&shdisp_sazabi_wakelock);

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_int_isr_common                                              */
/* ------------------------------------------------------------------------- */
static void shdisp_sazabi_int_isr_common(void)
{
    int ret;

    SHDISP_TRACE("in");

    if (shdisp_API_is_lcd_det_recovering()) {
        SHDISP_WARN("now recovering...");
        goto exit;
    }

    disable_irq_nosync(shdisp_sazabi_irq_gpio);

    if (!shdisp_wq_sazabi) {
        SHDISP_ERR("invalid work queue. wq=%p", shdisp_wq_sazabi);
        goto exit;
    }

    ret = shdisp_api_get_main_disp_status();
    if (ret == SHDISP_MAIN_DISP_OFF) {
        SHDISP_DEBUG("display OFF, will be exited.");
        goto exit;
    }

    wake_lock(&shdisp_sazabi_wakelock);

    ret = queue_work(shdisp_wq_sazabi, &shdisp_wq_sazabi_wk);
    if (ret == 0) {
        wake_unlock(&shdisp_sazabi_wakelock);
        SHDISP_DEBUG("failed to queue_work(). ret=%d", ret);
    }

exit:
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_int_isr                                                     */
/* ------------------------------------------------------------------------- */
static irqreturn_t shdisp_sazabi_int_isr(int irq_num, void *data)
{
    SHDISP_TRACE("in irq=%d", irq_num);

    shdisp_sazabi_int_isr_common();

    SHDISP_TRACE("out");
    return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_check_mipi_err                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_check_mipi_err(void)
{
    SHDISP_TRACE("in");

    if (shdisp_IO_API_get_Host_gpio(shdisp_sazabi_mipi_err_gpio)) {
        SHDISP_ERR("MIPI_ERR is HIGH.");
        shdisp_sazabi_int_isr_common();
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*      shdisp_sazabi_probe                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_probe(struct platform_device *pdev)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifdef CONFIG_OF
    struct resource *res;

    SHDISP_TRACE("in pdev=0x%p.", pdev);

    if (pdev) {
        res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!res) {
            SHDISP_ERR("irq resouce err!!");
        } else {
            shdisp_sazabi_irq_gpio = res->start;
            pshdisp_sazabi_irq_port_dev = pdev;
        }
        if (&(pdev->dev) != NULL) {
            memset(&shdisp_sazabi_vddi_gpio, 0x00, sizeof(shdisp_sazabi_vddi_gpio));
            shdisp_sazabi_vddi_gpio.gpio = of_get_named_gpio(pdev->dev.of_node, SHDISP_SAZABI_VDDIGPIO_ID, 0);
            if (!gpio_is_valid(shdisp_sazabi_vddi_gpio.gpio)) {
                SHDISP_ERR("vddi_gpio gpio not specified");
            } else {
                SHDISP_DEBUG("vddi_gpio succusess!");
            }

            shdisp_sazabi_vdda = devm_regulator_get(&(pdev->dev), SHDISP_SAZABI_VDDA_ID);
            if (IS_ERR(shdisp_sazabi_vdda)) {
                SHDISP_ERR("devm_regulator_get(%s) failure. (%ld)", SHDISP_SAZABI_VDDA_ID
                                                                        , PTR_ERR(shdisp_sazabi_vdda));
                shdisp_sazabi_vdda = NULL;
            } else {
                SHDISP_DEBUG("devm_regulator_get(%s) succusess!", SHDISP_SAZABI_VDDA_ID);
            }

            shdisp_sazabi_rst_gpio = of_get_named_gpio(pdev->dev.of_node, SHDISP_SAZABI_RST_ID, 0);
            if (!gpio_is_valid(shdisp_sazabi_rst_gpio)) {
                SHDISP_ERR("rst gpio not specified");
            } else {
                SHDISP_DEBUG("rst gpio succusess!");
            }

            shdisp_sazabi_mipi_err_gpio = of_get_named_gpio(pdev->dev.of_node, SHDISP_SAZABI_MIPIERR_ID, 0);
            if (!gpio_is_valid(shdisp_sazabi_mipi_err_gpio)) {
                SHDISP_ERR("mipierr gpio not specified");
            } else {
                SHDISP_DEBUG("mipierr gpio succusess!");
            }
        } else {
            SHDISP_ERR("pdev->dev is NULL");
        }
    }

    SHDISP_TRACE("out ret=%d", ret);
#endif /* CONFIG_OF */
    return ret;
}


/* ------------------------------------------------------------------------- */
/*      shdisp_sazabi_remove                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_remove(struct platform_device *pdev)
{
    SHDISP_TRACE("in.");
    /* shdisp_sazabi_vdda         */
    if (shdisp_sazabi_vdda) {
        regulator_put(shdisp_sazabi_vdda);
        shdisp_sazabi_vdda = NULL;
    }
    return SHDISP_RESULT_SUCCESS;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_sazabi_dt_match[] = {
    { .compatible = "sharp,shdisp_sazabi", },
    {}
};
#else
#define shdisp_sazabi_dt_match NULL;
#endif /* CONFIG_OF */

static struct platform_driver shdisp_sazabi_driver = {
    .probe = shdisp_sazabi_probe,
    .remove = shdisp_sazabi_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_sazabi",
        .of_match_table = shdisp_sazabi_dt_match,
    },
};

/* ------------------------------------------------------------------------- */
/*      shdisp_sazabi_register_driver                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_register_driver(void)
{
    SHDISP_TRACE("in.");
    return platform_driver_register(&shdisp_sazabi_driver);
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_is_user_cmd                                                 */
/* ------------------------------------------------------------------------- */
static bool shdisp_sazabi_is_user_cmd(unsigned char addr)
{
    bool ret = false;
    int i;

    for (i = 0; i < sizeof(shdisp_sazabi_user_cmd_addr); i++) {
        if (addr == shdisp_sazabi_user_cmd_addr[i]) {
            ret = true;
            break;
        }
    }
    SHDISP_DEBUG("ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_resister_write                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_resister_write(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret = 0;
    char dtype = 0;

    SHDISP_TRACE("in");

    if (size == 0) {
        dtype = SHDISP_DTYPE_DCS_WRITE;
    } else if (size == 1) {
        if (shdisp_sazabi_is_user_cmd(addr) == true) {
            dtype = SHDISP_DTYPE_DCS_WRITE1;
        } else {
            dtype = SHDISP_DTYPE_GEN_WRITE2;
        }
    } else {
        dtype = SHDISP_DTYPE_GEN_LWRITE;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(dtype, addr, write_data, size);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("DSI write error. ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sazabi_resister_read                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_sazabi_resister_read(unsigned char addr, unsigned char *read_data, unsigned char size)
{

    int ret = 0;
    char dtype = 0;

    SHDISP_TRACE("in");

    if (size == 0) {
        SHDISP_ERR("parameter error. length=0");
        return SHDISP_RESULT_FAILURE;
    } else if (size == 1) {
        if (shdisp_sazabi_is_user_cmd(addr) == true) {
            dtype = SHDISP_DTYPE_DCS_READ;
        } else {
            dtype = SHDISP_DTYPE_GEN_READ1;
        }
    } else {
        dtype = SHDISP_DTYPE_GEN_READ2;
    }

    ret = shdisp_panel_API_mipi_diag_read_reg(dtype, addr, read_data, size);
    if (ret) {
        SHDISP_ERR("DSI read error. ret=%d", ret);
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
