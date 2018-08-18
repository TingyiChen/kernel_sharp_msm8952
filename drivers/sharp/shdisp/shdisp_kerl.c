/* drivers/sharp/shdisp/shdisp_kerl.c  (Display Driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION
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
#include <linux/proc_fs.h>
#include <linux/fb.h>
#ifdef CONFIG_TOUCHSCREEN_SHTPS
#include <sharp/shtps_dev.h>
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#ifdef CONFIG_TOUCHSCREEN_FT8607
#include <linux/input/fts_dev.h>
#endif /* CONFIG_TOUCHSCREEN_FT8607 */
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#ifdef CONFIG_SHTERM
#include <sharp/shterm_k.h>
#endif /* CONFIG_SHTERM */
#include <sharp/sh_smem.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shdisp_to_user_context.h>
#ifdef CONFIG_PROXIMITY_INT_HOST
#include <sharp/proximity.h>
#endif /* CONFIG_PROXIMITY_INT_HOST */
#include "shdisp_kerl_priv.h"
#include "shdisp_kerl_context.h"
#include "shdisp_system.h"
#include "shdisp_io.h"
#include "shdisp_type.h"
#include "shdisp_dbg.h"
#include "shdisp_panel.h"
#ifdef SHDISP_IR2E71Y8
#include "shdisp_pm.h"
#include "shdisp_bdic.h"
#endif /* SHDISP_IR2E71Y8 */
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/delay.h>
#include <asm/param.h>
#ifdef SHDISP_ALS_INT
#include <linux/input.h>
#include <linux/miscdevice.h>
#endif /* SHDISP_ALS_INT */
#include <linux/leds.h>
#ifdef SHDISP_SYSFS_LED
#ifdef SHDISP_IR2E71Y8
#include "shdisp_led.h"
#endif /* SHDISP_IR2E71Y8 */
#endif /* SHDISP_SYSFS_LED */
#include <sharp/sh_boot_manager.h>
#ifndef SHDISP_IR2E71Y8
#include <sharp/shled_api.h>
#endif /* SHDISP_IR2E71Y8 */

#ifdef SHDISP_USE_QUALCOMM_RECOVERY
#ifndef SHDISP_NOT_SUPPORT_DET
int mdss_shdisp_report_panel_dead(void);
#endif /* SHDISP_NOT_SUPPORT_DET */
#else /* SHDISP_USE_QUALCOMM_RECOVERY */
#ifndef SHDISP_NOT_SUPPORT_DET
void mdss_shdisp_video_transfer_ctrl(int onoff, int commit);
void mdss_shdisp_lock_recovery(void);
void mdss_shdisp_unlock_recovery(void);
int mdss_shdisp_cmd_tearcheck_enable(bool enable);
#endif /* SHDISP_NOT_SUPPORT_DET */
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */

static int shdisp_kerl_register_driver(void);

#if defined(CONFIG_SHDISP_PANEL_ARIA) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#ifdef SHDISP_DET_DSI_MIPI_ERROR
extern void mdss_shdisp_dsi_mipi_err_clear(void);
extern int mdss_shdisp_dsi_mipi_err_ctrl(bool enable);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */
extern void mdss_shdisp_mdp_cmd_kickoff(void);
#endif /* CONFIG_SHDISP_PANEL */

extern int mdss_shdisp_mdp_hr_video_suspend(void);
extern int mdss_shdisp_mdp_hr_video_resume(void);
extern int mdss_shdisp_set_lp00_mode(int enable);
extern int mdss_shdisp_fps_led_start(void);
extern void mdss_shdisp_fps_led_stop(void);
extern void mdss_shdisp_bdic_bkl_set(bool fixed, int param);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#ifdef CONFIG_ARCH_PA35
#define SHDISP_NAME "ir2e71y"
#else
#define SHDISP_NAME "shdisp"
#endif /* CONFIG_ARCH_PA35 */

#define SHDISP_ALS_IRQ_REQ_BK_CTL           (0x01)
#define SHDISP_ALS_IRQ_REQ_DBC              (0x02)

#if defined(CONFIG_ANDROID_ENGINEERING)
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#define SHDISP_FPS_LED_PANEL_SUPPORT
#elif defined(CONFIG_SHDISP_PANEL_SAZABI)
#define SHDISP_FPS_LED_HOST_SUPPORT
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static dev_t shdisp_dev;
static dev_t shdisp_major = 0;
static dev_t shdisp_minor = 0;
static struct cdev shdisp_cdev;
static struct class *shdisp_class;
static struct shdisp_kernel_context shdisp_kerl_ctx;
static struct semaphore shdisp_sem;
static struct semaphore shdisp_sem_callback;
static struct semaphore shdisp_sem_irq_fac;
static struct semaphore shdisp_sem_timer;
static struct timer_list shdisp_timer;
static int shdisp_timer_stop = 1;
static sharp_smem_common_type *sh_smem_common = NULL;
static int first_lcd_on = false;
#ifdef SHDISP_USE_QUALCOMM_RECOVERY
static int mdss_recovery_retry_cnt = 0;
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */

static int shdisp_subscribe_type_table[NUM_SHDISP_IRQ_TYPE] = {
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_INT
};


static void (*shdisp_callback_table[NUM_SHDISP_IRQ_TYPE])(void) = {
    NULL,
    NULL,
    NULL,
    NULL
};

static spinlock_t                 shdisp_q_lock;
static struct shdisp_queue_data_t shdisp_queue_data;

static struct workqueue_struct    *shdisp_wq_gpio;
#ifdef SHDISP_IR2E71Y8
static struct work_struct         shdisp_wq_gpio_wk;
#endif /* SHDISP_IR2E71Y8 */

static struct workqueue_struct    *shdisp_wq_gpio_task;
#if defined(SHDISP_IR2E71Y8) || defined(CONFIG_ANDROID_ENGINEERING)
static struct work_struct         shdisp_wq_gpio_task_wk;
#endif /* SHDISP_IR2E71Y8 */

static struct workqueue_struct    *shdisp_wq_timer_task;
static struct work_struct         shdisp_wq_timer_task_wk;

static int shdisp_smem_read_flag = 0;



static struct       wake_lock shdisp_wake_lock_wq;
static int          shdisp_wake_lock_wq_refcnt;

static spinlock_t   shdisp_wake_spinlock;

static struct workqueue_struct    *shdisp_wq_recovery;
static struct semaphore shdisp_sem_req_recovery_lcd;
static unsigned int shdisp_recovery_lcd_queued_flag;
static struct work_struct         shdisp_wq_recovery_lcd_wk;
#ifdef SHDISP_IR2E71Y8
static unsigned int shdisp_pending_det_recovery = false;
#endif /* SHDISP_IR2E71Y8 */
static struct semaphore shdisp_sem_req_recovery_psals;
static unsigned int shdisp_recovery_psals_queued_flag;
static struct work_struct         shdisp_wq_recovery_psals_wk;
#ifdef SHDISP_NOT_SUPPORT_NO_OS
static struct shdisp_gpio_info testmode_info;
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

#ifdef SHDISP_ALS_INT
static struct wake_lock shdisp_timeout_wake_lock;
#ifdef SHDISP_IR2E71Y8
static struct input_dev *shdisp_input_dev;
static int als_int_enable = 0;
#endif /* SHDISP_IR2E71Y8 */
#endif /* SHDISP_ALS_INT */

#if defined(CONFIG_ANDROID_ENGINEERING)
static struct file_operations proc_fops;
#endif /* CONFIG_ANDROID_ENGINEERING */

#ifdef SHDISP_FPS_LED_PANEL_SUPPORT
#define ADDR_PAGE        (0xFF)
#define PAGE_FPS_REG     (0x26)
#define ADDR_FPS_REG     (0x8A)
#ifdef SHDISP_HAYABUSA_HF
#define FPS_LED_INTERVAL (66667)
#else /* SHDISP_HAYABUSA_HF */
#define FPS_LED_INTERVAL (135000)
#endif /* SHDISP_HAYABUSA_HF */

enum {
    FPS_LED_STATE_NONE,
    FPS_LED_STATE_1HZ,
    FPS_LED_STATE_15HZ,
    FPS_LED_STATE_30HZ,
    FPS_LED_STATE_60HZ,
    FPS_LED_STATE_120HZ,
    MAX_FPS_LED_STATE
};

static const char *shdisp_fps_led_state_str[MAX_FPS_LED_STATE] = {
    "NONE",
    "1HZ",
    "15HZ",
    "30HZ",
    "60HZ",
    "120HZ",
};

#ifdef SHDISP_IR2E71Y8
static const char shdisp_fps_led_color[MAX_FPS_LED_STATE][3] = {
    [FPS_LED_STATE_NONE]  = {0, 0, 0},
    [FPS_LED_STATE_1HZ]   = {0, 0, 1},
    [FPS_LED_STATE_15HZ]  = {0, 1, 0},
    [FPS_LED_STATE_30HZ]  = {1, 1, 0},
    [FPS_LED_STATE_60HZ]  = {1, 0, 0},
    [FPS_LED_STATE_120HZ] = {1, 1, 1},
};
#else /* SHDISP_IR2E71Y8 */
static const char shdisp_fps_led_color[MAX_FPS_LED_STATE][3] = {
    [FPS_LED_STATE_NONE]  = {0, 0, 0},
    [FPS_LED_STATE_1HZ]   = {0, 0, 255},
    [FPS_LED_STATE_15HZ]  = {0, 255, 0},
    [FPS_LED_STATE_30HZ]  = {255, 255, 0},
    [FPS_LED_STATE_60HZ]  = {255, 0, 0},
    [FPS_LED_STATE_120HZ] = {255, 255, 255},
};
#endif /* SHDISP_IR2E71Y8 */

static struct {
    bool enable;
    bool suspend;
    bool panel_on;
    int state;
    int interval;
    struct workqueue_struct *workq;
    struct delayed_work work;
} shdisp_fps_led_ctx = {0};

static void shdisp_fps_led_resume(void);
static void shdisp_fps_led_suspend(void);
#endif /* SHDISP_FPS_LED_PANEL_SUPPORT */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static void shdisp_init_context(void);
static void shdisp_get_boot_context(void);
static void shdisp_boot_context_initialize(void);
#ifdef SHDISP_NOT_SUPPORT_NO_OS
static int shdisp_get_upper_unit(struct shdisp_gpio_info *info);
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

#ifdef SHDISP_SYSFS_LED
#ifdef SHDISP_IR2E71Y8
static int  shdisp_SQE_tri_led_on(int no, struct shdisp_tri_led *sysfs_ledx);
#endif /* SHDISP_IR2E71Y8 */
#ifdef SHDISP_IR2E71Y8
static void shdisp_clean_sysfs_led(void);
static void shdisp_clean_normal_led(void);
#endif /* SHDISP_IR2E71Y8 */
#endif /* SHDISP_SYSFS_LED */

static int shdisp_check_initialized(void);
static int shdisp_check_upper_unit(void);
static int shdisp_check_bdic_exist(void);
static int shdisp_get_boot_disp_status(void);
static unsigned short shdisp_get_hw_revision(void);
static int shdisp_get_bdic_is_exist(void);
static struct shdisp_argc_lut *shdisp_get_argc_lut(void);
static struct shdisp_igc_lut *shdisp_get_igc_lut(void);
static int shdisp_bdic_subscribe_check(struct shdisp_subscribe *subscribe);
static int shdisp_bdic_unsubscribe_check(int irq_type);
#ifdef SHDISP_IR2E71Y8
static int shdisp_bdic_bkl_notify(int notify);
#endif /* SHDISP_IR2E71Y8 */

static int shdisp_open(struct inode *inode, struct file *filp);
static long shdisp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int shdisp_release(struct inode *inode, struct file *filp);

static int shdisp_ioctl_get_context(void __user *argp);
#ifdef SHDISP_IR2E71Y8
static int shdisp_ioctl_tri_led_set_color(void __user *argp);
#ifdef SHDISP_KEY_LED
static int shdisp_ioctl_key_bkl_ctl(void __user *argp);
#endif /* SHDISP_KEY_LED */
static int shdisp_ioctl_bdic_write_reg(void __user *argp);
static int shdisp_ioctl_bdic_read_reg(void __user *argp);
static int shdisp_ioctl_bdic_multi_read_reg(void __user *argp);
static int shdisp_ioctl_get_lux(void __user *argp);
static int shdisp_ioctl_photo_sensor_pow_ctl(void __user *argp);
#endif /* SHDISP_IR2E71Y8 */
static int shdisp_ioctl_lcddr_write_reg(void __user *argp);
static int shdisp_ioctl_lcddr_read_reg(void __user *argp);
static int shdisp_ioctl_set_flicker_param(void __user *argp);
static int shdisp_ioctl_get_flicker_param(void __user *argp);
static int shdisp_ioctl_get_flicker_low_param(void __user *argp);
#ifdef SHDISP_IR2E71Y8
static int shdisp_ioctl_bkl_set_auto_mode(void __user *argp);
static int shdisp_ioctl_bkl_set_dtv_mode(void __user *argp);
static int shdisp_ioctl_bkl_set_emg_mode(void __user *argp);
#ifdef SHDISP_LOWBKL
static int shdisp_ioctl_bkl_set_lowbkl_mode(void __user *argp);
#endif /* SHDISP_LOWBKL */
static int shdisp_ioctl_bkl_set_chg_mode(void __user *argp);
#endif /* SHDISP_IR2E71Y8 */
#ifdef SHDISP_TRV_NM2
static int shdisp_ioctl_set_trv_param(void __user *argp);
#endif /* SHDISP_TRV_NM2 */
static int shdisp_ioctl_panel_set_gmmtable_and_voltage(void __user *argp);
static int shdisp_ioctl_panel_get_gmmtable_and_voltage(void __user *argp);
static int shdisp_ioctl_panel_set_gmm(void __user *argp);
#ifdef SHDISP_IR2E71Y8
static int shdisp_ioctl_get_ave_ado(void __user *argp);
static int shdisp_ioctl_get_als(void __user *argp);
#endif /* SHDISP_IR2E71Y8 */
static int shdisp_ioctl_set_irq_mask(void __user *argp);
static int shdisp_ioctl_vcom_tracking(void __user * argp);
#ifdef SHDISP_IR2E71Y8
#ifdef SHDISP_ALS_INT
static int shdisp_ioctl_set_alsint(void __user *argp);
static int shdisp_ioctl_get_alsint(void __user *argp);
static int shdisp_ioctl_get_light_info(void __user *argp);
#endif /* SHDISP_ALS_INT */
#ifdef SHDISP_LED_INT
static int shdisp_led_auto_low_enable(void);
static int shdisp_led_auto_low_disable(void);
static int shdisp_ioctl_set_led_auto_low_mode(void __user *argp);
#endif /* SHDISP_LED_INT */
#endif /* SHDISP_IR2E71Y8 */
static int shdisp_ioctl_set_mfr(void __user *argp);
#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
static int shdisp_ioctl_set_illumi_triple_color(void __user *argp);
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
#ifdef SHDISP_PICADJ_USE_QDCM
static int shdisp_ioctl_get_lut_info(void __user *argp);
#endif /* SHDISP_PICADJ_USE_QDCM */

static int shdisp_SQE_main_lcd_power_on(void);
static int shdisp_SQE_main_lcd_power_off(void);
static int shdisp_SQE_main_lcd_disp_on(void);
static int shdisp_SQE_main_lcd_disp_off(void);
static int shdisp_SQE_main_lcd_start_display(void);
static int shdisp_SQE_main_lcd_post_video_start(void);
static int shdisp_SQE_main_lcd_display_done(void);
#ifdef SHDISP_IR2E71Y8
static int shdisp_SQE_panel_pow_ctl(int mode);
#endif /* SHDISP_IR2E71Y8 */
static int shdisp_SQE_shutdown(int seq);
static int shdisp_SQE_pre_shutdown(void);
static int shdisp_SQE_post_shutdown(void);
#ifdef SHDISP_IR2E71Y8
static int shdisp_SQE_main_bkl_ctl(int type, struct shdisp_main_bkl_ctl *bkl);
static int shdisp_SQE_main_bkl_set_dtv_mode(int dtv_mode);
static int shdisp_SQE_main_bkl_set_emg_mode(int emg_mode);
#ifdef SHDISP_LOWBKL
static int shdisp_SQE_main_bkl_set_lowbkl_mode(int lowbkl_mode);
#endif /* SHDISP_LOWBKL */
static int shdisp_SQE_main_bkl_set_chg_mode(int chg_mode);
#ifdef SHDISP_TRV_NM2
static int shdisp_SQE_set_trv_param(struct shdisp_trv_param param);
#endif /* SHDISP_TRV_NM2 */

static bool shdisp_led_set_color_reject(void);
static int shdisp_tri_led_set_color(struct shdisp_tri_led *tri_led);
static int shdisp_SQE_tri_led_set_color(struct shdisp_tri_led *tri_led);
#ifdef SHDISP_KEY_LED
static int shdisp_SQE_key_bkl_ctl(struct shdisp_key_bkl_ctl *key_bkl_ctl);
#endif /* SHDISP_KEY_LED */
static int shdisp_SQE_bdic_write_reg(unsigned char reg, unsigned char val);
static int shdisp_SQE_bdic_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_SQE_bdic_multi_read_reg(unsigned char reg, unsigned char *val, int size);
static int shdisp_SQE_get_lux(struct shdisp_photo_sensor_val *value);
static int shdisp_SQE_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
static int shdisp_SQE_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
static int shdisp_SQE_photo_sensor_pow_ctl(struct shdisp_photo_sensor_power_ctl *ctl);
#endif /* SHDISP_IR2E71Y8 */
static int shdisp_SQE_panel_write_reg(struct shdisp_lcddr_reg *panel_reg);
static int shdisp_SQE_panel_read_reg(struct shdisp_lcddr_reg *panel_reg);
#ifdef SHDISP_IR2E71Y8
static int shdisp_SQE_prox_sensor_pow_ctl(int power_mode);
#endif /* SHDISP_IR2E71Y8 */
static int shdisp_SQE_set_flicker_param(struct shdisp_diag_flicker_param vcom);
static int shdisp_SQE_get_flicker_param(struct shdisp_diag_flicker_param *vcom);
static int shdisp_SQE_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom);
static int shdisp_SQE_check_recovery(void);
#ifndef SHDISP_USE_QUALCOMM_RECOVERY
#ifndef SHDISP_NOT_SUPPORT_DET
static int shdisp_SQE_do_recovery(void);
#endif /* SHDISP_NOT_SUPPORT_DET */
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */
static int shdisp_SQE_event_subscribe(struct shdisp_subscribe *subscribe);
static int shdisp_SQE_event_unsubscribe(int irq_type);
static int shdisp_SQE_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_SQE_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_SQE_set_gmm(struct shdisp_diag_gamma *gmm);
#ifdef SHDISP_IR2E71Y8
static int shdisp_SQE_get_ave_ado(struct shdisp_ave_ado *ave_ado);
static int shdisp_SQE_get_als(struct shdisp_photo_sensor_raw_val *val);
#ifdef SHDISP_ALS_INT
static int shdisp_SQE_set_alsint(struct shdisp_photo_sensor_int_trigger *val);
static int shdisp_SQE_get_alsint(struct shdisp_photo_sensor_int_trigger *val);
static int shdisp_SQE_get_light_info(struct shdisp_light_info *val);
#endif /* SHDISP_ALS_INT */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
static int shdisp_SQE_set_illumi_triple_color(struct shdisp_illumi_triple_color * illumi_triple_color);
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
#else /* SHDISP_IR2E71Y8 */
static int shdisp_SQE_panel_mfr_ctl(struct shdisp_panel_mode p_mode);
#endif /* SHDISP_IR2E71Y8 */
static int shdisp_SQE_set_irq_mask(int irq_msk_ctl);
static int shdisp_SQE_vcom_tracking(int tracking);
static void shdisp_clear_recovery_lcd_queued(void);
#ifndef SHDISP_USE_QUALCOMM_RECOVERY
#ifndef SHDISP_NOT_SUPPORT_DET
static void shdisp_SQE_lcd_det_recovery(void);
#endif  /* SHDISP_NOT_SUPPORT_DET */
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */
static int shdisp_SQE_psals_recovery(void);

#ifdef SHDISP_IR2E71Y8
static irqreturn_t shdisp_gpio_int_isr(int irq_num, void *data);
static void shdisp_workqueue_handler_gpio(struct work_struct *work);
static void shdisp_workqueue_gpio_task(struct work_struct *work);
static int shdisp_do_detin_recovery(void);
#endif /* SHDISP_IR2E71Y8 */
static void shdisp_wake_lock_init(void);
static void shdisp_wake_lock(void);
static void shdisp_wake_unlock(void);
static void shdisp_timer_int_isr(unsigned long data);
static void shdisp_timer_int_register(void);
static void shdisp_timer_int_delete(void);
static void shdisp_timer_int_mod(void);
static void shdisp_workqueue_timer_task(struct work_struct *work);
static int shdisp_do_lcd_det_recovery(void);
static void shdisp_workqueue_handler_recovery_lcd(struct work_struct *work);
#ifdef SHDISP_USE_QUALCOMM_RECOVERY
#ifndef SHDISP_NOT_SUPPORT_DET
static void mdss_lcd_det_recovery(void);
#endif  /* SHDISP_NOT_SUPPORT_DET */
#else /* SHDISP_USE_QUALCOMM_RECOVERY */
#ifndef SHDISP_NOT_SUPPORT_DET
static void shdisp_lcd_det_recovery(void);
#endif  /* SHDISP_NOT_SUPPORT_DET */
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */
static int shdisp_lcd_det_recovery_subscribe(void);
static int shdisp_lcd_det_recovery_unsubscribe(void);
#ifdef SHDISP_IR2E71Y8
#ifdef SHDISP_ALS_INT
static int shdisp_do_als_int_report(int ginf2_val);
static void shdisp_do_als_int_report_dummy(void);
static int shdisp_set_als_int_subscribe(int trigger);
static int shdisp_set_als_int_unsubscribe(int trigger);
#endif /* SHDISP_ALS_INT */
static int shdisp_do_psals_recovery(void);
#endif /* SHDISP_IR2E71Y8 */
static void shdisp_workqueue_handler_recovery_psals(struct work_struct *work);
static void shdisp_psals_recovery(void);
static int shdisp_event_subscribe(struct shdisp_subscribe *subscribe);
static int shdisp_event_unsubscribe(int irq_type);
static void shdisp_det_mipi_err_ctrl(bool enable);

static void shdisp_semaphore_start(void);
static void shdisp_semaphore_end(const char *func);
#if defined(CONFIG_ANDROID_ENGINEERING)
static int shdisp_proc_write(struct file *file, const char *buffer, unsigned long count, void *data);
static int shdisp_proc_read(char *page, char **start, off_t offset, int count, int *eof, void *data);
static ssize_t shdisp_proc_file_read(struct file *file, char __user *buf, size_t nbytes, loff_t *ppos);
static ssize_t shdisp_proc_file_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos);
static void shdisp_dbg_info_output(int mode);
static void shdisp_dbg_que(int kind);
#ifdef SHDISP_IR2E71Y8
static void shdisp_debug_subscribe(void);
static void callback_ps(void);
#endif /* SHDISP_IR2E71Y8 */
#endif /* CONFIG_ANDROID_ENGINEERING */
static void shdisp_fb_open(void);
static void shdisp_fb_close(void);
static void shdisp_boot_err_output(void);
#ifdef SHDISP_ALS_INT
#ifdef SHDISP_IR2E71Y8
static int shdisp_input_subsystem_init(void);
static int shdisp_input_subsystem_report(int val);
#endif /* SHDISP_IR2E71Y8 */
#endif /* SHDISP_ALS_INT */
#ifdef SHDISP_SYSFS_LED
static ssize_t shdisp_led_blink_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t shdisp_led_blink_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t shdisp_led_on_off_ms_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static void shdisp_led_brightness_set(struct led_classdev *led_cdev, enum led_brightness value);
static enum led_brightness shdisp_led_brightness_get(struct led_classdev *led_cdev);
#endif /* SHDISP_SYSFS_LED */

static struct file_operations shdisp_fops = {
    .owner          = THIS_MODULE,
    .open           = shdisp_open,
    .write          = NULL,
    .read           = NULL,
    .mmap           = NULL,
    .unlocked_ioctl = shdisp_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = shdisp_ioctl,
#endif
    .release        = shdisp_release,
};

#ifdef SHDISP_SYSFS_LED
#define RED1_NAME       "red"
#define GREEN1_NAME     "green"
#define BLUE1_NAME      "blue"
static struct led_classdev red1_led = {
        .name           = RED1_NAME,
        .brightness     = LED_OFF,
        .brightness_set = shdisp_led_brightness_set,
        .brightness_get = shdisp_led_brightness_get,
        .max_brightness = 255,
};
static struct led_classdev green1_led = {
        .name           = GREEN1_NAME,
        .brightness     = LED_OFF,
        .brightness_set = shdisp_led_brightness_set,
        .brightness_get = shdisp_led_brightness_get,
        .max_brightness = 255,
};
static struct led_classdev blue1_led = {
        .name           = BLUE1_NAME,
        .brightness     = LED_OFF,
        .brightness_set = shdisp_led_brightness_set,
        .brightness_get = shdisp_led_brightness_get,
        .max_brightness = 255,
};
#ifdef SHDISP_COLOR_LED_TWIN
#define RED2_NAME       "red2"
#define GREEN2_NAME     "green2"
#define BLUE2_NAME      "blue2"
static struct led_classdev red2_led = {
        .name           = RED2_NAME,
        .brightness     = LED_OFF,
        .brightness_set = shdisp_led_brightness_set,
        .brightness_get = shdisp_led_brightness_get,
        .max_brightness = 255,
};
static struct led_classdev green2_led = {
        .name           = GREEN2_NAME,
        .brightness     = LED_OFF,
        .brightness_set = shdisp_led_brightness_set,
        .brightness_get = shdisp_led_brightness_get,
        .max_brightness = 255,
};
static struct led_classdev blue2_led = {
        .name           = BLUE2_NAME,
        .brightness     = LED_OFF,
        .brightness_set = shdisp_led_brightness_set,
        .brightness_get = shdisp_led_brightness_get,
        .max_brightness = 255,
};
#endif /* SHDISP_COLOR_LED_TWIN */

static DEVICE_ATTR(blink, 0664, shdisp_led_blink_show, shdisp_led_blink_store);
static DEVICE_ATTR(on_off_ms, 0664, NULL, shdisp_led_on_off_ms_store);

static struct attribute *blink_attrs[] = {
    &dev_attr_blink.attr,
    &dev_attr_on_off_ms.attr,
    NULL
};

static const struct attribute_group blink_attr_group = {
    .attrs = blink_attrs,
};
#endif /* SHDISP_SYSFS_LED */

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_power_on                                              */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_power_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL POWER-ON 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_power_on();

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_power_on.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL POWER-ON 0010 END");
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_power_off                                             */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_power_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("SUSPEND PANEL POWER-OFF 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_power_off();

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_power_off.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("SUSPEND PANEL POWER-OFF 0010 END");
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_disp_on                                               */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_disp_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL LCD-ON 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();
    ret = shdisp_SQE_main_lcd_disp_on();
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_disp_on.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL LCD-ON 0010 END");
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_start_display                                         */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_start_display(void)
{
    int ret = 0;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL START-DISP 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_start_display();

    shdisp_semaphore_end(__func__);

    SHDISP_PERFORMANCE("RESUME PANEL START-DISP 0010 END");
    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_post_video_start                                      */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_post_video_start(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL POST-VIDEO-START 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    (void)shdisp_SQE_main_lcd_post_video_start();

    shdisp_semaphore_end(__func__);

    SHDISP_PERFORMANCE("RESUME PANEL POST-VIDEO-START 0010 END");
    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_display_done                                              */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_display_done(void)
{
    int ret;
    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL DISPLAY-DONE 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_display_done();

    shdisp_semaphore_end(__func__);

#ifdef CONFIG_TOUCHSCREEN_SHTPS
    SHDISP_PERFORMANCE("msm_tps_setsleep(on) s");
    msm_tps_setsleep(0);
    SHDISP_PERFORMANCE("msm_tps_setsleep(on) e");
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#ifdef CONFIG_TOUCHSCREEN_FT8607
    fts_setsleep(0);
#endif /* CONFIG_TOUCHSCREEN_FT8607 */

    SHDISP_PERFORMANCE("RESUME PANEL DISPLAY-DONE 0010 END");
    SHDISP_TRACE("out");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_disp_off                                              */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_disp_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("SUSPEND PANEL LCD-OFF 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    SHDISP_DEBUG("");

#ifdef CONFIG_TOUCHSCREEN_SHTPS
    SHDISP_PERFORMANCE("msm_tps_setsleep(off) s");
    msm_tps_setsleep(1);
    SHDISP_PERFORMANCE("msm_tps_setsleep(off) e");
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#ifdef CONFIG_TOUCHSCREEN_FT8607
    fts_setsleep(1);
#endif /* CONFIG_TOUCHSCREEN_FT8607 */

    shdisp_semaphore_start();

    shdisp_lcd_det_recovery_unsubscribe();
    ret = shdisp_SQE_main_lcd_disp_off();

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_disp_off.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("SUSPEND PANEL LCD-OFF 0010 END");
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_bkl_on                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_bkl_on(struct shdisp_main_bkl_ctl *bkl)
{
#ifdef SHDISP_IR2E71Y8
    int ret;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("RESUME PANEL BACKLIGHT-ON 0010 START");

    if (bkl == NULL) {
        SHDISP_ERR("<NULL_POINTER> bkl.");
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: shutdown is in progress\n");
        return SHDISP_RESULT_SUCCESS;
    }

    if ((bkl->mode <= SHDISP_MAIN_BKL_MODE_OFF) ||
        (bkl->mode >= SHDISP_MAIN_BKL_MODE_DTV_OFF)) {
        SHDISP_ERR("<INVALID_VALUE> bkl->mode.");
        return SHDISP_RESULT_FAILURE;
    }

    if (bkl->mode != SHDISP_MAIN_BKL_MODE_FIX) {
        SHDISP_DEBUG("out4");
        return SHDISP_RESULT_SUCCESS;
    }

    if (bkl->param <= SHDISP_MAIN_BKL_PARAM_MIN) {
        SHDISP_ERR("<INVALID_VALUE> bkl->param.");
        return SHDISP_RESULT_FAILURE;
    }

    if (bkl->param > SHDISP_MAIN_BKL_PARAM_MAX) {
        bkl->param = SHDISP_MAIN_BKL_PARAM_MAX;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP, bkl);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_ctl.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL BACKLIGHT-ON 0010 END");
    SHDISP_TRACE("out");
#endif /* SHDISP_IR2E71Y8 */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_main_bkl_off                                                   */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_bkl_off(void)
{
#ifdef SHDISP_IR2E71Y8
    int ret;
    struct shdisp_main_bkl_ctl bkl_ctl;

    SHDISP_TRACE("in");
    SHDISP_PERFORMANCE("SUSPEND PANEL BACKLIGHT-OFF 0010 START");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: bkl_off is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    bkl_ctl.mode  = SHDISP_MAIN_BKL_MODE_OFF;
    bkl_ctl.param = SHDISP_MAIN_BKL_PARAM_OFF;

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP, &(bkl_ctl));

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_ctl.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("SUSPEND PANEL BACKLIGHT-OFF 0010 END");
    SHDISP_TRACE("out");
#endif /* SHDISP_IR2E71Y8 */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_shutdown                                                       */
/* ------------------------------------------------------------------------- */
int shdisp_api_shutdown(int seq)
{
    SHDISP_TRACE("in");

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
    if (seq == SHDISP_PRE_SHUTDOWN) {
#ifdef SHDISP_IR2E71Y8
        if (shdisp_bdic_API_LED_is_running_illumi_triple_color()) {
            shdisp_bdic_API_LED_clear_illumi_triple_color();
        }
#endif /* SHDISP_IR2E71Y8 */
    }
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

    shdisp_semaphore_start();
    shdisp_SQE_shutdown(seq);
    shdisp_semaphore_end(__func__);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_pre_blank_notify                                               */
/* ------------------------------------------------------------------------- */
void shdisp_api_pre_blank_notify(void)
{
#ifdef SHDISP_FPS_LED_PANEL_SUPPORT
    shdisp_fps_led_ctx.panel_on = false;
    shdisp_fps_led_suspend();
#endif /* SHDISP_FPS_LED_PANEL_SUPPORT */
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_write_bdic_i2c                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_api_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
#ifdef SHDISP_IR2E71Y8
    int ret;

    SHDISP_PERFORMANCE("RESUME BDIC WRITE-BDICI2C 0010 START");

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (i2c_msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg->wbuf.");
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_pm_API_is_ps_active() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("<RESULT_FAILURE> ps is not active.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->mode != SHDISP_BDIC_I2C_M_W) {
        i2c_msg->mode = SHDISP_BDIC_I2C_M_W;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_write_bdic_i2c(i2c_msg);

    shdisp_semaphore_end(__func__);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SQE_write_bdic_i2c.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    } else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_write_bdic_i2c.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME BDIC WRITE-BDICI2C 0010 END");
#endif /* SHDISP_IR2E71Y8 */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_read_bdic_i2c                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_api_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
#ifdef SHDISP_IR2E71Y8
    int ret;

    SHDISP_PERFORMANCE("RESUME BDIC READ-BDICI2C 0010 START");

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (i2c_msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg->wbuf.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->rbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg->rbuf.");
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_pm_API_is_ps_active() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("<RESULT_FAILURE> ps is not active.");
        return SHDISP_RESULT_FAILURE;
    }

    if (i2c_msg->mode != SHDISP_BDIC_I2C_M_R) {
        i2c_msg->mode = SHDISP_BDIC_I2C_M_R;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_read_bdic_i2c(i2c_msg);

    shdisp_semaphore_end(__func__);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SQE_read_bdic_i2c.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    } else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_read_bdic_i2c.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME BDIC READ-BDICI2C 0010 END");
#endif /* SHDISP_IR2E71Y8 */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_prox_sensor_pow_ctl                                            */
/* ------------------------------------------------------------------------- */
int shdisp_api_prox_sensor_pow_ctl(int power_mode, struct shdisp_prox_params *prox_params)
{
#ifdef SHDISP_IR2E71Y8
    int ret;

    SHDISP_PERFORMANCE("RESUME PANEL PROXSENSOR-CTL 0010 START");

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: prox_pow_ctl is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    if (power_mode >= NUM_SHDISP_PROX_SENSOR_POWER) {
        SHDISP_ERR("<INVALID_VALUE> power_mode(%d).", power_mode);
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_DEBUG(":power_mode=%d", power_mode);

    if (power_mode == SHDISP_PROX_SENSOR_POWER_ON) {
        if (prox_params == NULL) {
            SHDISP_ERR("<NULL_POINTER> prox_params.");
            return SHDISP_RESULT_FAILURE;
        }
        shdisp_bdic_API_set_prox_sensor_param(prox_params);
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_prox_sensor_pow_ctl(power_mode);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_prox_sensor_pow_ctl.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL PROXSENSOR-CTL 0010 START");
#endif /* SHDISP_IR2E71Y8 */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_boot_context                                               */
/* ------------------------------------------------------------------------- */
void shdisp_api_get_boot_context(void)
{
    shdisp_init_context();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_boot_disp_status                                           */
/* ------------------------------------------------------------------------- */
int shdisp_api_get_boot_disp_status(void)
{
    return shdisp_get_boot_disp_status();
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_argc_lut                                                   */
/* ------------------------------------------------------------------------- */
struct shdisp_argc_lut *shdisp_api_get_argc_lut(void)
{
    return shdisp_get_argc_lut();
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_igc_lut                                                    */
/* ------------------------------------------------------------------------- */
struct shdisp_igc_lut *shdisp_api_get_igc_lut(void)
{
    return shdisp_get_igc_lut();
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_main_disp_status                                           */
/* ------------------------------------------------------------------------- */
int shdisp_api_get_main_disp_status(void)
{
    int ret = SHDISP_MAIN_DISP_OFF;

    ret = shdisp_kerl_ctx.main_disp_status;

    SHDISP_INFO("out status=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_tri_led_set_color                                              */
/* ------------------------------------------------------------------------- */
int shdisp_api_tri_led_set_color(struct shdisp_tri_led *tri_led)
{
#ifdef SHDISP_IR2E71Y8
    int ret;
    unsigned char color;

    color = shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(tri_led);

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
    if (shdisp_bdic_API_LED_is_running_illumi_triple_color()) {
        shdisp_bdic_API_LED_clear_illumi_triple_color();
    }
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

    shdisp_semaphore_start();
    shdisp_kerl_ctx.led_set_color_reject = (color != 0);
    ret = shdisp_tri_led_set_color(tri_led);
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_tri_led_set_color.");
        return SHDISP_RESULT_FAILURE;
    }
#endif /* SHDISP_IR2E71Y8 */
    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_DET_DSI_MIPI_ERROR
/* ------------------------------------------------------------------------- */
/* shdisp_api_do_mipi_dsi_det_recovery                                       */
/* ------------------------------------------------------------------------- */
int shdisp_api_do_mipi_dsi_det_recovery(void)
{
    int ret= SHDISP_RESULT_SUCCESS;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in");
    SHDISP_ERR("MIPI Error");

#ifdef SHDISP_RESET_LOG
    err_code.mode = SHDISP_DBG_MODE_LINUX;
    err_code.type = SHDISP_DBG_TYPE_PANEL;
    err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
    err_code.subcode = SHDISP_DBG_SUBCODE_ESD_MIPI;
    shdisp_dbg_API_err_output(&err_code, 0);
    shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_ESD_MIPI);
#endif /* SHDISP_RESET_LOG */

#ifndef SHDISP_NOT_SUPPORT_DET
    ret = shdisp_do_lcd_det_recovery();
#endif /* SHDISP_NOT_SUPPORT_DET */

    SHDISP_TRACE("out (ret=%d)", ret);
    return ret;
}
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

/* ------------------------------------------------------------------------- */
/* shdisp_api_set_freq_param                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_api_set_freq_param(struct shdisp_freq_params *freq)
{
    int ret= SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

#ifdef SHDISP_HAYABUSA_OSC_SWITCH
    if (freq == NULL) {
        SHDISP_ERR("<NULL_POINTER> freq.");
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();
    ret = shdisp_panel_API_set_freq_param(freq);
    shdisp_semaphore_end(__func__);
#endif  /* SHDISP_HAYABUSA_OSC_SWITCH */

    SHDISP_TRACE("out (ret=%d)", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_panel_pow_ctl                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_api_panel_pow_ctl(int mode)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifdef SHDISP_IR2E71Y8
    SHDISP_TRACE("in");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();
    ret = shdisp_SQE_panel_pow_ctl(mode);
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_panel_pow_ctl.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out ret=%d", ret);
#endif /* SHDISP_IR2E71Y8 */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_mfr_ctl                                                        */
/* ------------------------------------------------------------------------- */
int shdisp_api_mfr_ctl(int brightness)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_IR2E71Y8
    struct shdisp_panel_mode p_mode;
    memset(&p_mode, 0x00, sizeof(p_mode));

    SHDISP_TRACE("in");

    shdisp_semaphore_start();

    p_mode.bkl_param = brightness;
    p_mode.trv_mode = shdisp_kerl_ctx.trv_status;
    SHDISP_DEBUG(" brightness:%d / trv:%d", p_mode.bkl_param, p_mode.trv_mode);

    ret = shdisp_SQE_panel_mfr_ctl(p_mode);

    shdisp_kerl_ctx.main_bkl.param = brightness;

    shdisp_semaphore_end(__func__);

    SHDISP_TRACE("out ret=%d", ret);
#endif /* SHDISP_IR2E71Y8 */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_is_lcd_det_recovering                                          */
/* ------------------------------------------------------------------------- */
int shdisp_API_is_lcd_det_recovering(void)
{
    int ret = false;

    SHDISP_TRACE("in.");

    if (shdisp_recovery_lcd_queued_flag) {
        ret = true;
    }

    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_get_hw_revision                                                */
/* ------------------------------------------------------------------------- */
unsigned short shdisp_API_get_hw_revision(void)
{
    return shdisp_get_hw_revision();
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_get_hw_handset                                                 */
/* ------------------------------------------------------------------------- */
unsigned short shdisp_API_get_hw_handset(void)
{
    return shdisp_kerl_ctx.boot_ctx.hw_handset;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_get_bdic_is_exist                                              */
/* ------------------------------------------------------------------------- */
int shdisp_API_get_bdic_is_exist(void)
{
    return shdisp_get_bdic_is_exist();
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_is_open                                                        */
/* ------------------------------------------------------------------------- */
int shdisp_API_is_open(void)
{
    return shdisp_kerl_ctx.driver_is_open;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_do_lcd_det_recovery                                            */
/* ------------------------------------------------------------------------- */
int shdisp_API_do_lcd_det_recovery(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_do_lcd_det_recovery();

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_psals_recovery_subscribe                                       */
/* ------------------------------------------------------------------------- */
void shdisp_API_psals_recovery_subscribe(void)
{
    struct shdisp_subscribe subscribe;

    SHDISP_TRACE("in");

    subscribe.irq_type = SHDISP_IRQ_TYPE_I2CERR;
    subscribe.callback = shdisp_psals_recovery;

    shdisp_event_subscribe(&subscribe);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_psals_recovery_unsubscribe                                     */
/* ------------------------------------------------------------------------- */
void shdisp_API_psals_recovery_unsubscribe(void)
{
    SHDISP_TRACE("in");

    shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_I2CERR);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_check_upper_unit                                               */
/* ------------------------------------------------------------------------- */
int shdisp_API_check_upper_unit(void)
{
    return shdisp_check_upper_unit();
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_semaphore_start                                                */
/* ------------------------------------------------------------------------- */
void shdisp_API_semaphore_start(void)
{
    SHDISP_INFO("in");
    shdisp_semaphore_start();
    SHDISP_INFO("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_API_semaphore_end                                                  */
/* ------------------------------------------------------------------------- */
void shdisp_API_semaphore_end(void)
{
    SHDISP_INFO("in");
    shdisp_semaphore_end(__func__);
    SHDISP_INFO("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_subscribe_check                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_subscribe_check(struct shdisp_subscribe *subscribe)
{


    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    if (subscribe == NULL) {
        SHDISP_ERR("<NULL POINTER> INT_SUBSCRIBE subscribe");
        return SHDISP_RESULT_FAILURE;
    }

    if ((subscribe->irq_type < SHDISP_IRQ_TYPE_ALS) || (subscribe->irq_type >= NUM_SHDISP_IRQ_TYPE)) {
        SHDISP_ERR("<INVALID_VALUE> subscribe->irq_type(%d)", subscribe->irq_type);
        return SHDISP_RESULT_FAILURE;
    }

    if (subscribe->callback == NULL) {
        SHDISP_ERR("<NULL_POINTER> subscribe->callback");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_unsubscribe_check                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_unsubscribe_check(int irq_type)
{
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    if ((irq_type < SHDISP_IRQ_TYPE_ALS) || (irq_type >= NUM_SHDISP_IRQ_TYPE)) {
        SHDISP_ERR("<INVALID_VALUE> irq_type(%d)", irq_type);
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}
#ifdef SHDISP_IR2E71Y8
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_bkl_notify                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_bkl_notify(int notify)
{
    unsigned long int notify_value = 0, notify_brightness = 0;
    struct shdisp_bdic_bkl_info bkl_info;
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    struct shdisp_panel_mode p_mode;
    memset(&p_mode, 0x00, sizeof(p_mode));
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */

    SHDISP_TRACE("in");
    memset(&bkl_info, 0x00, sizeof(bkl_info));
    shdisp_bdic_API_LCD_BKL_get_param(&bkl_info);
    if (bkl_info.mode != SHDISP_BDIC_BKL_MODE_OFF) {
        notify_value = 1;
    }
    notify_brightness = bkl_info.value;
#ifdef CONFIG_SHTERM
    if (notify | SHDISP_INFO_BACKLIGHT) {
        shterm_k_set_info(SHTERM_INFO_BACKLIGHT, notify_value);
    }
    if (notify | SHDISP_INFO_BACKLIGHT_LEV) {
        shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
    }
#endif  /* CONFIG_SHTERM */
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    if (shdisp_kerl_ctx.driver_is_open) {
        if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
            p_mode.bkl_mode = bkl_info.mode;
            p_mode.bkl_param = bkl_info.param;
#ifdef SHDISP_TRV_NM2
            p_mode.trv_mode = shdisp_kerl_ctx.trv_status;
#endif /* SHDISP_TRV_NM2 */
            shdisp_panel_API_chg_mode(p_mode);
        }
    }
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */
    if (shdisp_kerl_ctx.driver_is_open) {
        if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
            if (bkl_info.mode == SHDISP_BDIC_BKL_MODE_FIX) {
                mdss_shdisp_bdic_bkl_set(true, bkl_info.param);
            } else if (bkl_info.mode == SHDISP_BDIC_BKL_MODE_AUTO) {
                mdss_shdisp_bdic_bkl_set(false, bkl_info.param);
            }
        }
    }
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* INITIALIZE                                                                */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_init_context                                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_init_context(void)
{
#ifndef SHDISP_NOT_SUPPORT_NO_OS
    sharp_smem_common_type *sh_smem_common_adr = NULL;
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */
    if (shdisp_smem_read_flag != 0) {
        return;
    }

    memset(&shdisp_kerl_ctx, 0, sizeof(shdisp_kerl_ctx));

    shdisp_get_boot_context();

#ifdef SHDISP_NOT_SUPPORT_NO_OS
    shdisp_kerl_ctx.boot_ctx.bdic_is_exist      = SHDISP_BDIC_IS_EXIST;
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

    shdisp_kerl_ctx.driver_is_open              = false;
    shdisp_kerl_ctx.driver_open_cnt             = 0;
    shdisp_kerl_ctx.driver_is_initialized       = SHDISP_DRIVER_IS_NOT_INITIALIZED;
    shdisp_kerl_ctx.shutdown_in_progress        = false;
    shdisp_kerl_ctx.thermal_status              = SHDISP_MAIN_BKL_EMG_OFF;
    shdisp_kerl_ctx.usb_chg_status              = SHDISP_MAIN_BKL_CHG_OFF;
    shdisp_kerl_ctx.main_disp_status            = shdisp_kerl_ctx.boot_ctx.main_disp_status;
    shdisp_kerl_ctx.main_bkl.mode               = shdisp_kerl_ctx.boot_ctx.main_bkl.mode;
    shdisp_kerl_ctx.main_bkl.param              = shdisp_kerl_ctx.boot_ctx.main_bkl.param;
    shdisp_kerl_ctx.tri_led.red                 = shdisp_kerl_ctx.boot_ctx.tri_led.red;
    shdisp_kerl_ctx.tri_led.green               = shdisp_kerl_ctx.boot_ctx.tri_led.green;
    shdisp_kerl_ctx.tri_led.blue                = shdisp_kerl_ctx.boot_ctx.tri_led.blue;
    shdisp_kerl_ctx.tri_led.ext_mode            = shdisp_kerl_ctx.boot_ctx.tri_led.ext_mode;
    shdisp_kerl_ctx.tri_led.led_mode            = shdisp_kerl_ctx.boot_ctx.tri_led.led_mode;
    shdisp_kerl_ctx.tri_led.ontime              = shdisp_kerl_ctx.boot_ctx.tri_led.ontime;
    shdisp_kerl_ctx.tri_led.interval            = shdisp_kerl_ctx.boot_ctx.tri_led.interval;
    shdisp_kerl_ctx.tri_led.count               = shdisp_kerl_ctx.boot_ctx.tri_led.count;

#ifdef SHDISP_SYSFS_LED
    shdisp_kerl_ctx.sysfs_led1.red              =0;
    shdisp_kerl_ctx.sysfs_led1.green            =0;
    shdisp_kerl_ctx.sysfs_led1.blue             =0;
    shdisp_kerl_ctx.sysfs_led1.ext_mode         =0;
    shdisp_kerl_ctx.sysfs_led1.led_mode         =SHDISP_TRI_LED_MODE_OFF;
    shdisp_kerl_ctx.sysfs_led1.ontime           =0;
    shdisp_kerl_ctx.sysfs_led1.interval         =0;
    shdisp_kerl_ctx.sysfs_led1.count            =0;
#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_kerl_ctx.sysfs_led2.red              =0;
    shdisp_kerl_ctx.sysfs_led2.green            =0;
    shdisp_kerl_ctx.sysfs_led2.blue             =0;
    shdisp_kerl_ctx.sysfs_led2.ext_mode         =0;
    shdisp_kerl_ctx.sysfs_led2.led_mode         =SHDISP_TRI_LED_MODE_OFF;
    shdisp_kerl_ctx.sysfs_led2.ontime           =0;
    shdisp_kerl_ctx.sysfs_led2.interval         =0;
    shdisp_kerl_ctx.sysfs_led2.count            =0;
#endif /* SHDISP_COLOR_LED_TWIN */
#endif /* SHDISP_SYSFS_LED */

#ifndef SHDISP_NOT_SUPPORT_NO_OS
    sh_smem_common_adr = (sharp_smem_common_type *)sh_smem_get_common_address();
    if (sh_smem_common_adr != NULL) {
        memcpy(&(shdisp_kerl_ctx.boot_ctx), &sh_smem_common_adr->shdisp_data_buf, sizeof(struct shdisp_boot_context));

        shdisp_kerl_ctx.main_bkl.mode = shdisp_kerl_ctx.boot_ctx.main_bkl.mode;
        shdisp_kerl_ctx.main_bkl.param = shdisp_kerl_ctx.boot_ctx.main_bkl.param;

        shdisp_kerl_ctx.tri_led.red = shdisp_kerl_ctx.boot_ctx.tri_led.red;
        shdisp_kerl_ctx.tri_led.green = shdisp_kerl_ctx.boot_ctx.tri_led.green;
        shdisp_kerl_ctx.tri_led.blue = shdisp_kerl_ctx.boot_ctx.tri_led.blue;
        shdisp_kerl_ctx.tri_led.ext_mode = shdisp_kerl_ctx.boot_ctx.tri_led.ext_mode;
        shdisp_kerl_ctx.tri_led.led_mode = shdisp_kerl_ctx.boot_ctx.tri_led.led_mode;
        shdisp_kerl_ctx.tri_led.ontime = shdisp_kerl_ctx.boot_ctx.tri_led.ontime;
        shdisp_kerl_ctx.tri_led.interval = shdisp_kerl_ctx.boot_ctx.tri_led.interval;
        shdisp_kerl_ctx.tri_led.count = shdisp_kerl_ctx.boot_ctx.tri_led.count;
        shdisp_smem_read_flag = 1;
    }
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

#if defined(CONFIG_ANDROID_ENGINEERING)
#ifdef SHDISP_LOG_ENABLE   /* for debug */
    shdisp_dbg_info_output(SHDISP_DEBUG_INFO_TYPE_POWERON);
#endif  /* SHDISP_LOG_ENABLE */
#endif  /* CONFIG_ANDROID_ENGINEERING */
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_common_address                                                 */
/* ------------------------------------------------------------------------- */
static sharp_smem_common_type *shdisp_get_common_address(void)
{
    sharp_smem_common_type *sh_smem_common_adr = NULL;
#ifndef SHDISP_NOT_SUPPORT_NO_OS
    sh_smem_common_adr = (sharp_smem_common_type *)sh_smem_get_common_address();
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */
    return sh_smem_common_adr;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_boot_context                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_get_boot_context(void)
{
    sh_smem_common = shdisp_get_common_address();
    if (sh_smem_common == NULL) {
        shdisp_boot_context_initialize();

#ifdef SHDISP_IR2E71Y8
        shdisp_bdic_API_check_sensor_param(&(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj), &(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj));
#endif /* SHDISP_IR2E71Y8 */
        /* Check upper unit connect status */
#ifdef SHDISP_NOT_SUPPORT_NO_OS
        if (shdisp_get_upper_unit(&testmode_info) == SHDISP_RESULT_SUCCESS) {
            shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected = SHDISP_UPPER_UNIT_IS_CONNECTED;
        } else {
            shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected = SHDISP_UPPER_UNIT_IS_NOT_CONNECTED;
        }
#else   /* SHDISP_NOT_SUPPORT_NO_OS */
        shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected = SHDISP_UPPER_UNIT_IS_NOT_CONNECTED;
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */
    } else {
        memcpy(&(shdisp_kerl_ctx.boot_ctx), &sh_smem_common->shdisp_data_buf, sizeof(struct shdisp_boot_context));
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_context_initialize                                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_boot_context_initialize(void)
{
    int i;

    shdisp_kerl_ctx.boot_ctx.driver_is_initialized       = SHDISP_DRIVER_IS_INITIALIZED;
    shdisp_kerl_ctx.boot_ctx.hw_handset                  = 1;
    shdisp_kerl_ctx.boot_ctx.hw_revision                 = SHDISP_HW_REV_PP2;
    shdisp_kerl_ctx.boot_ctx.device_code                 = 0xff;
    shdisp_kerl_ctx.boot_ctx.disp_on_status              = 0;
    shdisp_kerl_ctx.boot_ctx.handset_color               = 0;
    shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected     = SHDISP_UPPER_UNIT_IS_NOT_CONNECTED;
    shdisp_kerl_ctx.boot_ctx.bdic_is_exist               = SHDISP_BDIC_IS_NOT_EXIST;
    shdisp_kerl_ctx.boot_ctx.main_disp_status            = SHDISP_MAIN_DISP_OFF;
    shdisp_kerl_ctx.boot_ctx.is_trickled                 = false;
    shdisp_kerl_ctx.boot_ctx.main_bkl.mode               = SHDISP_MAIN_BKL_MODE_OFF;
    shdisp_kerl_ctx.boot_ctx.main_bkl.param              = SHDISP_MAIN_BKL_PARAM_OFF;
    shdisp_kerl_ctx.boot_ctx.tri_led.red                 = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.green               = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.blue                = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.ext_mode            = SHDISP_TRI_LED_EXT_MODE_DISABLE;
    shdisp_kerl_ctx.boot_ctx.tri_led.led_mode            = SHDISP_TRI_LED_MODE_NORMAL;
    shdisp_kerl_ctx.boot_ctx.tri_led.ontime              = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.interval            = 0;
    shdisp_kerl_ctx.boot_ctx.tri_led.count               = 0;
    shdisp_kerl_ctx.boot_ctx.vcom                        = 0;
    shdisp_kerl_ctx.boot_ctx.vcom_nvram                  = 0;

    memset(&(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj), 0, sizeof(struct shdisp_photo_sensor_adj));
    memset(&(shdisp_kerl_ctx.boot_ctx.lcddr_phy_gmm), 0, sizeof(struct shdisp_lcddr_phy_gmm_reg));

    for (i = 0; i < SHDISP_IGC_LUT_ENTRIES; i++) {
        if (i == 0) {
            shdisp_kerl_ctx.boot_ctx.igc_lut.r_data[i] = 0x0000;
            shdisp_kerl_ctx.boot_ctx.igc_lut.g_data[i] = 0x0000;
            shdisp_kerl_ctx.boot_ctx.igc_lut.b_data[i] = 0x0000;
        } else {
            shdisp_kerl_ctx.boot_ctx.igc_lut.r_data[i] = 0x0010;
            shdisp_kerl_ctx.boot_ctx.igc_lut.g_data[i] = 0x0010;
            shdisp_kerl_ctx.boot_ctx.igc_lut.b_data[i] = 0x0010;
        }
    }

    memset(shdisp_kerl_ctx.boot_ctx.argc_lut.red, 0, sizeof(shdisp_kerl_ctx.boot_ctx.argc_lut.red));
    memset(shdisp_kerl_ctx.boot_ctx.argc_lut.green, 0, sizeof(shdisp_kerl_ctx.boot_ctx.argc_lut.green));
    memset(shdisp_kerl_ctx.boot_ctx.argc_lut.blue, 0, sizeof(shdisp_kerl_ctx.boot_ctx.argc_lut.blue));
    shdisp_kerl_ctx.boot_ctx.argc_lut.red[SHDISP_ARGC_STAGE_NUM - 1][1] = 0x0100;
    shdisp_kerl_ctx.boot_ctx.argc_lut.green[SHDISP_ARGC_STAGE_NUM - 1][1] = 0x0100;
    shdisp_kerl_ctx.boot_ctx.argc_lut.blue[SHDISP_ARGC_STAGE_NUM - 1][1] = 0x0100;

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_upper_unit                                                     */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_NOT_SUPPORT_NO_OS
static int shdisp_get_upper_unit(struct shdisp_gpio_info *info)
{
    int val;
    SHDISP_TRACE("in");

    if (info == NULL) {
        SHDISP_ERR("info param is NULL.");
        return SHDISP_RESULT_FAILURE;
    }

    if (IS_ERR_OR_NULL(info->state_active)) {
        SHDISP_ERR("gpio_state_active not specified");
        return SHDISP_RESULT_FAILURE;
    }

    if (IS_ERR_OR_NULL(info->state_suspend)) {
        SHDISP_ERR("gpio_state_suspend not specified");
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_IO_API_Host_gpio_request(info->gpio, "upper_unit");
    pinctrl_select_state(info->pinctrl, info->state_active);

    shdisp_IO_API_delay_us(5);
    val = shdisp_IO_API_get_Host_gpio(info->gpio);

    pinctrl_select_state(info->pinctrl, info->state_suspend);
    shdisp_IO_API_Host_gpio_free(info->gpio);
    SHDISP_DEBUG("check_upper_unit val=%d", val);

    if (!val) {
        SHDISP_ERR("<OTHER> Upper unit does not exist.");
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

/* ------------------------------------------------------------------------- */
/* CHECKER                                                                   */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_check_initialized                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_check_initialized(void)
{
    if (shdisp_kerl_ctx.driver_is_initialized == SHDISP_DRIVER_IS_INITIALIZED) {
        return SHDISP_RESULT_SUCCESS;
    } else {
        SHDISP_DEBUG("[SHDISP] shdisp_check_initialized error : driver is not initialized.");
        return SHDISP_RESULT_FAILURE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_check_upper_unit                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_check_upper_unit(void)
{
    if (shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected == SHDISP_UPPER_UNIT_IS_CONNECTED) {
        return SHDISP_RESULT_SUCCESS;
    } else {
        SHDISP_ERR("[SHDISP] shdisp_check_upper_unit error : upper unit is not connected.");
        return SHDISP_RESULT_FAILURE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_check_bdic_exist                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_check_bdic_exist(void)
{
#ifdef SHDISP_IR2E71Y8
    if (shdisp_kerl_ctx.boot_ctx.bdic_is_exist == SHDISP_BDIC_IS_EXIST) {
        return SHDISP_RESULT_SUCCESS;
    } else {
        SHDISP_ERR("[SHDISP] shdisp_check_bdic_exist error : bdic is not exist.");
        return SHDISP_RESULT_FAILURE;
    }
#else /* SHDISP_IR2E71Y8 */
    return SHDISP_RESULT_SUCCESS;
#endif /* SHDISP_IR2E71Y8 */
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_boot_disp_status                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_get_boot_disp_status(void)
{
    return shdisp_kerl_ctx.boot_ctx.main_disp_status;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_hw_revision                                                    */
/* ------------------------------------------------------------------------- */
static unsigned short shdisp_get_hw_revision(void)
{
    return shdisp_kerl_ctx.boot_ctx.hw_revision;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_bdic_is_exist                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_get_bdic_is_exist(void)
{
    return shdisp_kerl_ctx.boot_ctx.bdic_is_exist;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_argc_lut                                                       */
/* ------------------------------------------------------------------------- */
static struct shdisp_argc_lut *shdisp_get_argc_lut(void)
{
    return &shdisp_kerl_ctx.boot_ctx.argc_lut;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_igc_lut                                                        */
/* ------------------------------------------------------------------------- */
static struct shdisp_igc_lut *shdisp_get_igc_lut(void)
{
    return &shdisp_kerl_ctx.boot_ctx.igc_lut;
}

/* ------------------------------------------------------------------------- */
/* FOPS                                                                      */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_open                                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_open(struct inode *inode, struct file *filp)
{

    if (shdisp_kerl_ctx.driver_open_cnt == 0) {
        SHDISP_DEBUG("[SHDISP] new open shdisp device driver.");
    }

    shdisp_kerl_ctx.driver_open_cnt++;

    if (shdisp_kerl_ctx.driver_open_cnt == 1 && !shdisp_kerl_ctx.driver_is_open) {
        shdisp_kerl_ctx.driver_is_open = true;

        shdisp_boot_err_output();
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl                                                              */
/* ------------------------------------------------------------------------- */
static long shdisp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret;
    void __user *argp = (void __user*)arg;

    SHDISP_INFO("shdisp ioctl. cmd=%08X", cmd);

    switch (cmd) {
    case SHDISP_IOCTL_GET_CONTEXT:
        ret = shdisp_ioctl_get_context(argp);
        break;
#ifdef SHDISP_IR2E71Y8
    case SHDISP_IOCTL_TRI_LED_SET_COLOR:
        ret = shdisp_ioctl_tri_led_set_color(argp);
        break;
#ifdef SHDISP_KEY_LED
    case SHDISP_IOCTL_KEY_BKL_CTL:
        ret = shdisp_ioctl_key_bkl_ctl(argp);
        break;
#endif /* SHDISP_KEY_LED */
    case SHDISP_IOCTL_BDIC_WRITE_REG:
        ret = shdisp_ioctl_bdic_write_reg(argp);
        break;
    case SHDISP_IOCTL_BDIC_READ_REG:
        ret = shdisp_ioctl_bdic_read_reg(argp);
        break;
    case SHDISP_IOCTL_BDIC_MULTI_READ_REG:
        ret = shdisp_ioctl_bdic_multi_read_reg(argp);
        break;
    case SHDISP_IOCTL_GET_LUX:
        SHDISP_PERFORMANCE("RESUME PANEL GET-LUX 0010 START");
        ret = shdisp_ioctl_get_lux(argp);
        SHDISP_PERFORMANCE("RESUME PANEL GET-LUX 0010 END");
        break;
    case SHDISP_IOCTL_PHOTO_SENSOR_POW_CTL:
        SHDISP_PERFORMANCE("RESUME PANEL PHOTO-SENCOR 0010 START");
        ret = shdisp_ioctl_photo_sensor_pow_ctl(argp);
        SHDISP_PERFORMANCE("RESUME PANEL PHOTO-SENCOR 0010 END");
        break;
#endif /* SHDISP_IR2E71Y8 */
    case SHDISP_IOCTL_LCDDR_WRITE_REG:
        ret = shdisp_ioctl_lcddr_write_reg(argp);
        break;
    case SHDISP_IOCTL_SET_MFR:
        ret = shdisp_ioctl_set_mfr(argp);
        break;
    case SHDISP_IOCTL_LCDDR_READ_REG:
        ret = shdisp_ioctl_lcddr_read_reg(argp);
        break;
    case SHDISP_IOCTL_SET_FLICKER_PARAM:
        ret = shdisp_ioctl_set_flicker_param(argp);
        break;
    case SHDISP_IOCTL_GET_FLICKER_PARAM:
        ret = shdisp_ioctl_get_flicker_param(argp);
        break;
#ifdef SHDISP_IR2E71Y8
    case SHDISP_IOCTL_BKL_SET_AUTO_MODE:
        ret = shdisp_ioctl_bkl_set_auto_mode(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_DTV_MODE:
        ret = shdisp_ioctl_bkl_set_dtv_mode(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_EMG_MODE:
        ret = shdisp_ioctl_bkl_set_emg_mode(argp);
        break;
#ifdef SHDISP_LOWBKL
    case SHDISP_IOCTL_BKL_SET_LOWBKL_MODE:
        ret = shdisp_ioctl_bkl_set_lowbkl_mode(argp);
        break;
#endif /* SHDISP_LOWBKL */
    case SHDISP_IOCTL_BKL_SET_CHG_MODE:
        ret = shdisp_ioctl_bkl_set_chg_mode(argp);
        break;
#endif /* SHDISP_IR2E71Y8 */
    case SHDISP_IOCTL_GET_FLICKER_LOW_PARAM:
        ret = shdisp_ioctl_get_flicker_low_param(argp);
        break;
    case SHDISP_IOCTL_SET_GMMTABLE_AND_VOLTAGE:
        SHDISP_TRACE("SHDISP_IOCTL_SET_GMMTABLE_AND_VOLTAGE Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_panel_set_gmmtable_and_voltage(argp);
        SHDISP_TRACE("SHDISP_IOCTL_SET_GMMTABLE_AND_VOLTAGE Completed ret:%d", ret);
        break;
    case SHDISP_IOCTL_GET_GMMTABLE_AND_VOLTAGE:
        SHDISP_TRACE("SHDISP_IOCTL_GET_GMMTABLE_AND_VOLTAGE Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_panel_get_gmmtable_and_voltage(argp);
        SHDISP_TRACE("SHDISP_IOCTL_GET_GMMTABLE_AND_VOLTAGE Completed ret:%d", ret);
        break;
    case SHDISP_IOCTL_SET_GMM:
        SHDISP_TRACE("SHDISP_IOCTL_SET_GMM Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_panel_set_gmm(argp);
        SHDISP_TRACE("SHDISP_IOCTL_SET_GMM Completed ret:%d", ret);
        break;
#ifdef SHDISP_IR2E71Y8
    case SHDISP_IOCTL_GET_AVE_ADO:
        SHDISP_TRACE("SHDISP_IOCTL_GET_AVE_ADO Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_get_ave_ado(argp);
        SHDISP_TRACE("SHDISP_IOCTL_GET_AVE_ADO Completed ret:%d", ret);
        break;
    case SHDISP_IOCTL_GET_ALS:
        SHDISP_TRACE("SHDISP_IOCTL_GET_ALS Requested. cmd=%08X", cmd);
        ret = shdisp_ioctl_get_als(argp);
        SHDISP_TRACE("SHDISP_IOCTL_GET_ALS Completed ret:%d", ret);
        break;
#endif /* SHDISP_IR2E71Y8 */
    case SHDISP_IOCTL_SET_IRQ_MASK:
        ret = shdisp_ioctl_set_irq_mask(argp);
        break;
    case SHDISP_IOCTL_VCOM_TRACKING:
        ret = shdisp_ioctl_vcom_tracking(argp);
        break;
#ifdef SHDISP_IR2E71Y8
#ifdef SHDISP_ALS_INT
    case SHDISP_IOCTL_SET_ALSINT:
        ret = shdisp_ioctl_set_alsint(argp);
        break;
    case SHDISP_IOCTL_GET_ALSINT:
        ret = shdisp_ioctl_get_alsint(argp);
        break;
    case SHDISP_IOCTL_GET_LIGHT_INFO:
        ret = shdisp_ioctl_get_light_info(argp);
        break;
#endif /* SHDISP_ALS_INT */
#ifdef SHDISP_LED_INT
    case SHDISP_IOCTL_SET_LED_AUTO_LOW_MODE:
        ret = shdisp_ioctl_set_led_auto_low_mode(argp);
        break;
#endif /* SHDISP_LED_INT */
#endif /* SHDISP_IR2E71Y8 */
#ifdef SHDISP_TRV_NM2
    case SHDISP_IOCTL_SET_TRV_PARAM:
        ret = shdisp_ioctl_set_trv_param(argp);
        break;
#endif /* SHDISP_TRV_NM2 */
#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
    case SHDISP_IOCTL_SET_ILLUMI_TRI_COLOR:
        ret = shdisp_ioctl_set_illumi_triple_color(argp);
        break;
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
#ifdef SHDISP_PICADJ_USE_QDCM
    case SHDISP_IOCTL_GET_LUT_INFO:
        ret = shdisp_ioctl_get_lut_info(argp);
        break;
#endif /* SHDISP_PICADJ_USE_QDCM */
    default:
        SHDISP_ERR("<INVALID_VALUE> cmd(0x%08x).", cmd);
        ret = -EFAULT;
        break;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_release                                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_release(struct inode *inode, struct file *filp)
{
    if (shdisp_kerl_ctx.driver_open_cnt > 0) {
        shdisp_kerl_ctx.driver_open_cnt--;

        if (shdisp_kerl_ctx.driver_open_cnt == 0) {
            SHDISP_DEBUG("[SHDISP] all close shdisp device driver.");
        }
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_context                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_context(void __user *argp)
{
    int ret;
    struct shdisp_to_user_context shdisp_user_ctx;

    shdisp_semaphore_start();

    shdisp_user_ctx.hw_handset                  = shdisp_kerl_ctx.boot_ctx.hw_handset;
    shdisp_user_ctx.hw_revision                 = shdisp_kerl_ctx.boot_ctx.hw_revision;
    shdisp_user_ctx.handset_color               = shdisp_kerl_ctx.boot_ctx.handset_color;
    shdisp_user_ctx.upper_unit_is_connected     = shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected;
    shdisp_user_ctx.bdic_is_exist               = shdisp_kerl_ctx.boot_ctx.bdic_is_exist;
    shdisp_user_ctx.main_disp_status            = shdisp_kerl_ctx.boot_ctx.main_disp_status;
    memcpy(&(shdisp_user_ctx.igc_lut), &(shdisp_kerl_ctx.boot_ctx.igc_lut), sizeof(struct shdisp_igc_lut));
    shdisp_user_ctx.is_vcom_tracking            = 0;
    if (shdisp_SYS_API_get_debugflg() & SHDISP_DEBUGFLG_BIT_USER_LOG) {
        shdisp_user_ctx.log_lv_all              = 1;
    } else {
        shdisp_user_ctx.log_lv_all              = 0;
    }

    ret = copy_to_user(argp, &shdisp_user_ctx, sizeof(struct shdisp_to_user_context));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_tri_led_set_color                                            */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_IR2E71Y8
static int shdisp_ioctl_tri_led_set_color(void __user *argp)
{

    int ret;
    struct shdisp_tri_led tri_led;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: set_color is in progress");
        return SHDISP_RESULT_SUCCESS;
    }


#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
    if (shdisp_bdic_API_LED_is_running_illumi_triple_color()) {
        shdisp_bdic_API_LED_clear_illumi_triple_color();
    }
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
    shdisp_semaphore_start();

    ret = copy_from_user(&tri_led, argp, sizeof(struct shdisp_tri_led));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_tri_led_set_color(&tri_led);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_tri_led_set_color.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}
#ifdef SHDISP_KEY_LED
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_key_bkl_ctl                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_key_bkl_ctl(void __user *argp)
{

    int ret;
    struct shdisp_key_bkl_ctl key_bkl_ctl;

    shdisp_semaphore_start();

    ret = copy_from_user(&key_bkl_ctl, argp, sizeof(struct shdisp_key_bkl_ctl));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_key_bkl_ctl(&key_bkl_ctl);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_key_bkl_ctl.\n");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_KEY_LED */
#endif /* SHDISP_IR2E71Y8 */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_illumi_triple_color                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_illumi_triple_color(void __user *argp)
{

    int ret;
    struct shdisp_illumi_triple_color illumi_triple_color;
    int i = 0;
    int size;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: set_color is in progress");
        return SHDISP_RESULT_SUCCESS;
    }
#ifdef SHDISP_IR2E71Y8
    if (shdisp_bdic_API_LED_is_running_illumi_triple_color()) {
        shdisp_bdic_API_LED_clear_illumi_triple_color();
    }
#endif /* SHDISP_IR2E71Y8 */
    shdisp_semaphore_start();

    ret = copy_from_user(&illumi_triple_color, argp, sizeof(illumi_triple_color));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    size = sizeof(illumi_triple_color.colors) / sizeof(illumi_triple_color.colors[0]);
    for (i = 0; i != size; i++) {
        SHDISP_DEBUG("colors[%d] red = %d, green=%d, blue=%d", i, illumi_triple_color.colors[i].red,
                     illumi_triple_color.colors[i].green, illumi_triple_color.colors[i].blue);
    }
    SHDISP_DEBUG("count = %d", illumi_triple_color.count);
#ifdef SHDISP_IR2E71Y8
    ret = shdisp_SQE_set_illumi_triple_color(&illumi_triple_color);
#endif /* SHDISP_IR2E71Y8 */
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_tri_led_set_color.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

#ifdef SHDISP_PICADJ_USE_QDCM
static int shdisp_ioctl_get_lut_info(void __user *argp)
{
    int ret;
    struct shdisp_lut_info lut_info;

    shdisp_semaphore_start();

    memset(&lut_info, 0x00, sizeof(lut_info));
    lut_info.status = shdisp_kerl_ctx.boot_ctx.lut_status;
    memcpy(&lut_info.lut, &shdisp_kerl_ctx.boot_ctx.argc_lut, sizeof(struct shdisp_argc_lut));

    ret = copy_to_user(argp, &lut_info, sizeof(lut_info));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_PICADJ_USE_QDCM */

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bdic_write_reg                                               */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_IR2E71Y8
static int shdisp_ioctl_bdic_write_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_bdic_reg bdic_reg;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: write_reg is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&bdic_reg, argp, sizeof(struct shdisp_diag_bdic_reg));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_bdic_write_reg(bdic_reg.reg, bdic_reg.val);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_bdic_write_reg.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bdic_read_reg                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bdic_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_bdic_reg bdic_reg;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: read_reg is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&bdic_reg, argp, sizeof(struct shdisp_diag_bdic_reg));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_bdic_read_reg(bdic_reg.reg, &(bdic_reg.val));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_bdic_read_reg.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &bdic_reg, sizeof(struct shdisp_diag_bdic_reg));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bdic_multi_read_reg                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bdic_multi_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_bdic_reg_multi bdic_reg;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: multi_read is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&bdic_reg, argp, sizeof(struct shdisp_diag_bdic_reg_multi));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_bdic_multi_read_reg(bdic_reg.reg, bdic_reg.val, (int)bdic_reg.size);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_bdic_multi_read_reg.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &bdic_reg, sizeof(struct shdisp_diag_bdic_reg_multi));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_lux                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_lux(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_val val;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: get_lux is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&val, argp, sizeof(struct shdisp_photo_sensor_val));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_get_lux(&(val));

    SHDISP_DEBUG(" value=0x%04X, lux=%u, mode=%d", val.value, val.lux, val.mode);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_lux.");
    }

    ret = copy_to_user(argp, &val, sizeof(struct shdisp_photo_sensor_val));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_photo_sensor_pow_ctl                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_photo_sensor_pow_ctl(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_power_ctl power_ctl;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: photo_pow_ctl is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&power_ctl, argp, sizeof(struct shdisp_photo_sensor_power_ctl));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_photo_sensor_pow_ctl(&power_ctl);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_photo_sensor_pow_ctl.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_lcddr_write_reg                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_lcddr_write_reg(void __user *argp)
{
    int ret;
    struct shdisp_lcddr_reg panel_reg;

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&panel_reg, argp, sizeof(struct shdisp_lcddr_reg));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_panel_write_reg(&panel_reg);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_panel_write_reg.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_lcddr_read_reg                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_lcddr_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_lcddr_reg panel_reg;

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&panel_reg, argp, sizeof(struct shdisp_lcddr_reg));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_panel_read_reg(&panel_reg);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_panel_read_reg.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &panel_reg, sizeof(struct shdisp_lcddr_reg));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_flicker_param                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_flicker_param(void __user *argp)
{
    int ret;
    struct shdisp_diag_flicker_param vcom;

    shdisp_semaphore_start();

    ret = copy_from_user(&vcom, argp, sizeof(struct shdisp_diag_flicker_param));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_set_flicker_param(vcom);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_flicker_param.");
        return -EIO;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_flicker_param                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_flicker_param(void __user *argp)
{
    int ret;
    struct shdisp_diag_flicker_param vcom;

    vcom.master_alpha = 0;
    vcom.slave_alpha = 0;

    shdisp_semaphore_start();

    ret = shdisp_SQE_get_flicker_param(&vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_flicker_param.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &vcom, sizeof(struct shdisp_diag_flicker_param));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_flicker_low_param                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_flicker_low_param(void __user *argp)
{
    int ret;
    struct shdisp_diag_flicker_param vcom;

    vcom.master_alpha = 0;
    vcom.slave_alpha = 0;

    shdisp_semaphore_start();

    ret = shdisp_SQE_get_flicker_low_param(&vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_flicker_low_param.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &vcom, sizeof(struct shdisp_diag_flicker_param));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_auto_mode                                            */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_IR2E71Y8
static int shdisp_ioctl_bkl_set_auto_mode(void __user *argp)
{
    int ret;
    struct shdisp_main_bkl_auto auto_bkl;
    struct shdisp_main_bkl_ctl bkl_ctl;

    SHDISP_PERFORMANCE("RESUME PANEL BACKLIGHT-SET-AUTO-MODE 0010 START");

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: auto_mode is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&auto_bkl, argp, sizeof(struct shdisp_main_bkl_auto));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    if (auto_bkl.mode == SHDISP_MAIN_BKL_AUTO_OFF) {
        SHDISP_DEBUG("BKL_AUTO_MODE : AUTO_OFF");
        bkl_ctl.mode  = SHDISP_MAIN_BKL_MODE_AUTO;
        bkl_ctl.param = SHDISP_MAIN_BKL_PARAM_OFF;
    } else if (auto_bkl.mode == SHDISP_MAIN_BKL_AUTO_ON) {
        if ((auto_bkl.param >= SHDISP_MAIN_BKL_PARAM_MIN_AUTO) &&
            (auto_bkl.param <= SHDISP_MAIN_BKL_PARAM_MAX_AUTO)) {
            SHDISP_DEBUG("BKL_AUTO_MODE : AUTO_ON");
            bkl_ctl.mode  = SHDISP_MAIN_BKL_MODE_AUTO;
            bkl_ctl.param = auto_bkl.param;
        } else {
            SHDISP_ERR("<INVALID_VALUE> mode(%d) param(%d)", auto_bkl.mode, auto_bkl.param);
            shdisp_semaphore_end(__func__);
            return -EIO;
        }
    } else {
        SHDISP_ERR("<INVALID_VALUE> mode(%d).", auto_bkl.mode);
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO, &(bkl_ctl));

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_ctl.");
        return -EIO;
    }

    SHDISP_PERFORMANCE("RESUME PANEL BACKLIGHT-SET-AUTO-MODE 0010 END");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_dtv_mode                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bkl_set_dtv_mode(void __user *argp)
{
    int ret;
    int dtv_mode;

    ret = copy_from_user(&dtv_mode, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_dtv_mode(dtv_mode);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_dtv_mode.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_emg_mode                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bkl_set_emg_mode(void __user *argp)
{
    int ret;
    int emg_mode;

    ret = copy_from_user(&emg_mode, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_emg_mode(emg_mode);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_emg_mode.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_LOWBKL
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_lowbkl_mode                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bkl_set_lowbkl_mode(void __user *argp)
{
    int ret;
    int lowbkl_mode;

    ret = copy_from_user(&lowbkl_mode, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_lowbkl_mode(lowbkl_mode);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_lowbkl_mode.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_LOWBKL */

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_chg_mode                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_bkl_set_chg_mode(void __user *argp)
{
    int ret;
    int chg_mode;

    ret = copy_from_user(&chg_mode, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_chg_mode(chg_mode);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_chg_mode.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_IR2E71Y8 */

#ifdef SHDISP_TRV_NM2
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_trv_param                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_trv_param(void __user *argp)
{
    int ret = -EIO;
    struct shdisp_trv_param param;
#ifndef SHDISP_IR2E71Y8
    struct shdisp_panel_mode p_mode;
    memset(&p_mode, 0x00, sizeof(p_mode));
#endif /* SHDISP_IR2E71Y8 */

    ret = copy_from_user(&param, argp, sizeof(param));
    if (ret) {
        SHDISP_ERR("copy_from_user() failure. ret=%d", ret);
        return ret;
    }

    shdisp_semaphore_start();
#ifdef SHDISP_IR2E71Y8
    ret = shdisp_SQE_set_trv_param(param);
#else /* SHDISP_IR2E71Y8 */
    p_mode.bkl_param = shdisp_kerl_ctx.main_bkl.param;
    p_mode.trv_mode = param.request;

    ret = shdisp_SQE_panel_mfr_ctl(p_mode);

    shdisp_kerl_ctx.trv_status = param.request;
#endif /* SHDISP_IR2E71Y8 */
    shdisp_semaphore_end(__func__);

    return ret;
}
#endif /* SHDISP_TRV_NM2 */

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_panel_set_gmmtable_and_voltage                               */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_panel_set_gmmtable_and_voltage(void __user *argp)
{
    int ret;
    struct shdisp_diag_gamma_info gmm_info;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&gmm_info,
                         argp,
                         sizeof(struct shdisp_diag_gamma_info));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_set_gmmtable_and_voltage(&gmm_info);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_gmmtable_and_voltage.");
        return -EIO;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_panel_get_gmmtable_and_voltage                               */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_panel_get_gmmtable_and_voltage(void __user *argp)
{
    int ret;
    struct shdisp_diag_gamma_info gmm_info;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    shdisp_semaphore_start();

    memset(&gmm_info, 0, sizeof(gmm_info));
    ret = shdisp_SQE_get_gmmtable_and_voltage(&gmm_info);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_gmmtable_and_voltage.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp,
                       &gmm_info,
                       sizeof(struct shdisp_diag_gamma_info));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_panel_set_gmm                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_panel_set_gmm(void __user *argp)
{
    int ret;
    struct shdisp_diag_gamma gmm;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&gmm,
                         argp,
                         sizeof(struct shdisp_diag_gamma));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }


    ret = shdisp_SQE_set_gmm(&gmm);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_gmm.");
        return -EIO;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_ave_ado                                                  */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_IR2E71Y8
static int shdisp_ioctl_get_ave_ado(void __user *argp)
{
    int ret;
    struct shdisp_ave_ado ave_ado;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: get_ave_ado is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&ave_ado, argp, sizeof(struct shdisp_ave_ado));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_get_ave_ado(&ave_ado);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_ioctl_get_ave_ado.");
        shdisp_semaphore_end(__func__);
        return -EIO;
    }

    ret = copy_to_user(argp, &ave_ado, sizeof(struct shdisp_ave_ado));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_als                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_als(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_raw_val val;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return -EIO;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: get_als is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&val, argp, sizeof(struct shdisp_photo_sensor_raw_val));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_get_als(&val);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_als.");
    }

    ret = copy_to_user(argp, &val, sizeof(struct shdisp_photo_sensor_raw_val));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_irq_mask                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_irq_mask(void __user *argp)
{
    int ret;
    int irq_msk_ctl;

    ret = copy_from_user(&irq_msk_ctl, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_set_irq_mask(irq_msk_ctl);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_ioctl_set_irq_mask.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_vcom_tracking                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_vcom_tracking(void __user *argp)
{
    int ret;
    int tracking;

    ret = copy_from_user(&tracking, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        return ret;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_vcom_tracking(tracking);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_ioctl_vcom_tracking.\n");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}
#ifdef SHDISP_IR2E71Y8
#ifdef SHDISP_ALS_INT
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_alsint                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_alsint(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_int_trigger val;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: set_alsint is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();
    ret = copy_from_user(&val, argp, sizeof(struct shdisp_photo_sensor_int_trigger));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_set_alsint(&val);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_alsint.");
    }

    ret = copy_to_user(argp, &val, sizeof(struct shdisp_photo_sensor_int_trigger));
    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_alsint                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_alsint(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_int_trigger val;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: get_alsint is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();
    ret = copy_from_user(&val, argp, sizeof(struct shdisp_photo_sensor_int_trigger));
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_SQE_get_alsint(&val);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_alsint.");
    }

    ret = copy_to_user(argp, &val, sizeof(struct shdisp_photo_sensor_int_trigger));
    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_light_info                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_get_light_info(void __user *argp)
{
    int ret;
    struct shdisp_light_info val;

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("bdic is not exist");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("canceled: get_light_info is in progress");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();
    ret = shdisp_SQE_get_light_info(&val);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_light_info.");
    }

    ret = copy_to_user(argp, &val, sizeof(struct shdisp_light_info));
    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.");
        return ret;
    }
    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_ALS_INT */

#ifdef SHDISP_LED_INT
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_led_auto_low_mode                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_led_auto_low_mode(void __user *argp)
{
    int ret;
    struct shdisp_led_auto_low_mode_param param;

    ret = copy_from_user(&param, argp, sizeof(param));
    if (ret) {
        SHDISP_ERR("copy_from_user() failure. ret=%d", ret);
        return ret;
    }

    shdisp_semaphore_start();

    if (param.enable) {
        ret = shdisp_led_auto_low_enable();
        if (ret == SHDISP_RESULT_SUCCESS) {
            ret = 0;
        } else {
            ret = -EIO;
        }
    } else {
        ret = shdisp_led_auto_low_disable();
        if (ret == SHDISP_RESULT_SUCCESS) {
            ret = 0;
        } else {
            ret = -EIO;
        }
    }

    shdisp_semaphore_end(__func__);

    return ret;
}
#endif /* SHDISP_LED_INT */
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_mfr                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_mfr(void __user *argp)
{
    int ret;
    int val = 0;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("upper_unit nothing");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = copy_from_user(&val, argp, sizeof(int));

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.");
        shdisp_semaphore_end(__func__);
        return ret;
    }

    ret = shdisp_panel_API_mfr(val);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_ioctl_set_mfr.");
        return -EIO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* SEQUENCE                                                                  */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_power_on                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_power_on(void)
{

    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

#ifdef SHDISP_IR2E71Y8
    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_ON);

    if (first_lcd_on) {
        shdisp_pm_API_panel_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_ON, SHDISP_PANEL_POWER_NORMAL_ON);
    } else {
        shdisp_pm_API_panel_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_ON, SHDISP_PANEL_POWER_FIRST_ON);
        first_lcd_on = true;
    }
#else /* SHDISP_IR2E71Y8 */
    if (first_lcd_on) {
#ifdef SHDISP_USE_QUALCOMM_RECOVERY
        shdisp_panel_API_power_on(SHDISP_PANEL_POWER_RECOVERY_ON);
#else /* SHDISP_USE_QUALCOMM_RECOVERY */
        shdisp_panel_API_power_on(SHDISP_PANEL_POWER_NORMAL_ON);
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */
    } else {
        shdisp_panel_API_power_on(SHDISP_PANEL_POWER_FIRST_ON);
        first_lcd_on = true;
    }
#endif /* SHDISP_IR2E71Y8 */
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_power_off                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_power_off(void)
{
    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

#ifdef SHDISP_IR2E71Y8
    if (shdisp_kerl_ctx.shutdown_in_progress) {
        shdisp_pm_API_panel_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_OFF, SHDISP_PANEL_POWER_SHUTDOWN_OFF);
    } else {
        shdisp_pm_API_panel_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_OFF, SHDISP_PANEL_POWER_NORMAL_OFF);
    }

    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_OFF);
#else /* SHDISP_IR2E71Y8 */
    if (shdisp_kerl_ctx.shutdown_in_progress) {
        shdisp_panel_API_power_off(SHDISP_PANEL_POWER_SHUTDOWN_OFF);
    } else {
#ifdef SHDISP_USE_QUALCOMM_RECOVERY
        shdisp_panel_API_power_off(SHDISP_PANEL_POWER_RECOVERY_OFF);
#else /* SHDISP_USE_QUALCOMM_RECOVERY */
        shdisp_panel_API_power_off(SHDISP_PANEL_POWER_NORMAL_OFF);
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */
    }
#endif /* SHDISP_IR2E71Y8 */
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_disp_on                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_disp_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_disp_on();

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_start_display                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_start_display(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_start_display();

    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_ON;

#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_LCDPOW, 1);
#endif  /* CONFIG_SHTERM */

#ifdef SHDISP_FPS_LED_PANEL_SUPPORT
    shdisp_fps_led_ctx.panel_on = true;
    shdisp_fps_led_resume();
#endif /* SHDISP_FPS_LED_PANEL_SUPPORT */

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_post_video_start                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_post_video_start(void)
{
    int ret;
    SHDISP_TRACE("in");

    ret = shdisp_panel_API_post_video_start();

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_display_done                                          */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_USE_QUALCOMM_RECOVERY
static int shdisp_SQE_main_lcd_display_done(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifdef SHDISP_RESET_LOG
    unsigned char subcode;
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in");

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_lcd_det_recovery_subscribe();

    ret = shdisp_SQE_check_recovery();
    shdisp_clear_recovery_lcd_queued();
    if (ret != SHDISP_RESULT_SUCCESS) {
        if (mdss_recovery_retry_cnt < 3) {
            shdisp_do_lcd_det_recovery();
        } else {
            SHDISP_ERR("recovery retry over");
#ifdef SHDISP_RESET_LOG
            subcode = shdisp_dbg_API_get_subcode();
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_PANEL;
            err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
            err_code.subcode = subcode;
            shdisp_dbg_API_err_output(&err_code, 0);
            shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_NONE);
#endif /* SHDISP_RESET_LOG */
            goto end;
        }
    } else {
        mdss_recovery_retry_cnt = 0;
#ifdef SHDISP_RESET_LOG
       shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_NONE);
#endif /* SHDISP_RESET_LOG */
    }

    shdisp_det_mipi_err_ctrl(true);

end:
    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}
#else /* SHDISP_USE_QUALCOMM_RECOVERY */
static int shdisp_SQE_main_lcd_display_done(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_clear_recovery_lcd_queued();

    shdisp_lcd_det_recovery_subscribe();

    ret = shdisp_SQE_check_recovery();
    if (ret != SHDISP_RESULT_SUCCESS) {
        shdisp_do_lcd_det_recovery();
    }

    shdisp_det_mipi_err_ctrl(true);

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_disp_off                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_lcd_disp_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_OFF) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_det_mipi_err_ctrl(false);

    ret = shdisp_panel_API_disp_off();

    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_OFF;

#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_LCDPOW, 0);
#endif  /* CONFIG_SHTERM */

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_panel_pow_ctl                                                  */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_IR2E71Y8
static int shdisp_SQE_panel_pow_ctl(int mode)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in mode=%d", mode);

    switch (mode) {
    case SHDISP_PANEL_POWER_ON:
        (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_TP, SHDISP_DEV_REQ_ON);
        if (shdisp_pm_API_is_active_panel_user(SHDISP_DEV_TYPE_TP) == SHDISP_DEV_STATE_OFF) {
            shdisp_pending_det_recovery = false;
        }
        ret = shdisp_pm_API_panel_power_manager(SHDISP_DEV_TYPE_TP, SHDISP_DEV_REQ_ON, SHDISP_PANEL_POWER_TP_ON);
        break;
    case SHDISP_PANEL_POWER_OFF:
        ret = shdisp_pm_API_panel_power_manager(SHDISP_DEV_TYPE_TP, SHDISP_DEV_REQ_OFF, SHDISP_PANEL_POWER_TP_OFF);
        (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_TP, SHDISP_DEV_REQ_OFF);
        SHDISP_DEBUG("recovery pending status. pending_det_recovery=%d main_disp_status=%d",
                     shdisp_pending_det_recovery, shdisp_kerl_ctx.main_disp_status);
        if (shdisp_pending_det_recovery) {
            shdisp_pending_det_recovery = false;
            if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
                ret = shdisp_do_lcd_det_recovery();
            }
        }
        break;
    default:
        ret = SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return ret;
}
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_shutdown                                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_shutdown(int seq)
{
    SHDISP_TRACE("in");

    if ((seq != SHDISP_PRE_SHUTDOWN) && (seq != SHDISP_POST_SHUTDOWN)) {
        return SHDISP_RESULT_FAILURE;
    }

    if (seq == SHDISP_PRE_SHUTDOWN) {
        shdisp_SQE_pre_shutdown();
    } else {
        shdisp_SQE_post_shutdown();
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_pre_shutdown                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_pre_shutdown(void)
{
    SHDISP_TRACE("in main_disp_status=%d", shdisp_kerl_ctx.main_disp_status);

    shdisp_kerl_ctx.shutdown_in_progress = true;

#ifdef SHDISP_IR2E71Y8
    shdisp_pm_API_psals_power_off();

    if (shdisp_kerl_ctx.main_bkl.mode != SHDISP_MAIN_BKL_MODE_OFF) {
        shdisp_bdic_API_LCD_BKL_off();
    }
#endif /* SHDISP_IR2E71Y8 */
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_OFF) {
        SHDISP_DEBUG("shutdown progress main_disp_status=%d", shdisp_kerl_ctx.main_disp_status);
        (void)shdisp_panel_API_shutdown();
    }
    shdisp_SYS_API_free_irq();

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_post_shutdown                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_post_shutdown(void)
{
    SHDISP_TRACE("in");
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_shutdown();
#endif /* SHDISP_IR2E71Y8 */
    shdisp_kerl_ctx.boot_ctx.bdic_is_exist = SHDISP_BDIC_IS_NOT_EXIST;

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}
#ifdef SHDISP_IR2E71Y8
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_ctl                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_bkl_ctl(int type, struct shdisp_main_bkl_ctl *bkl)
{
    struct shdisp_main_bkl_ctl temp, request;

    SHDISP_TRACE("in");
    if (type >= NUM_SHDISP_MAIN_BKL_DEV_TYPE) {
        SHDISP_ERR("<INVALID_VALUE> type(%d).", type);
        return SHDISP_RESULT_FAILURE;
    }

    temp.mode  = bkl->mode;
    temp.param = bkl->param;
    shdisp_bdic_API_LCD_BKL_get_request(type, &temp, &request);

    if ((request.mode == shdisp_kerl_ctx.main_bkl.mode) && (request.param == shdisp_kerl_ctx.main_bkl.param)) {
        return SHDISP_RESULT_SUCCESS;
    }

    switch (request.mode) {
    case SHDISP_MAIN_BKL_MODE_OFF:
        shdisp_bdic_API_LCD_BKL_off();
        break;
    case SHDISP_MAIN_BKL_MODE_FIX:
        shdisp_bdic_API_LCD_BKL_fix_on(request.param);
        break;
    case SHDISP_MAIN_BKL_MODE_AUTO:
        shdisp_bdic_API_LCD_BKL_auto_on(request.param);
        break;
    default:
        break;
    }

    shdisp_kerl_ctx.main_bkl.mode  = request.mode;
    shdisp_kerl_ctx.main_bkl.param = request.param;
    shdisp_bdic_bkl_notify(SHDISP_INFO_BACKLIGHT | SHDISP_INFO_BACKLIGHT_LEV);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_dtv_mode                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_bkl_set_dtv_mode(int dtv_mode)
{
    if (dtv_mode == SHDISP_MAIN_BKL_DTV_OFF) {
        SHDISP_DEBUG("BKL_DTV_MODE : DTV_OFF");
        shdisp_bdic_API_LCD_BKL_dtv_off();
    } else if (dtv_mode == SHDISP_MAIN_BKL_DTV_ON) {
        SHDISP_DEBUG("BKL_DTV_MODE : DTV_ON");
        shdisp_bdic_API_LCD_BKL_dtv_on();
    } else {
        SHDISP_ERR("<INVALID_VALUE> dtv_mode(%d).", dtv_mode);
        return SHDISP_RESULT_FAILURE;
    }
    shdisp_bdic_bkl_notify(SHDISP_INFO_BACKLIGHT_LEV);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_emg_mode                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_bkl_set_emg_mode(int emg_mode)
{
    SHDISP_TRACE("in emg_mode=%d", emg_mode);

    if ((emg_mode < 0) || (emg_mode >= NUM_SHDISP_MAIN_BKL_EMG)) {
        SHDISP_ERR("invalid emg_mode. emg_mode=%d", emg_mode);
        return SHDISP_RESULT_FAILURE;
    }

    switch (emg_mode) {
    case SHDISP_MAIN_BKL_EMG_OFF:
        SHDISP_DEBUG("EMG_OFF");
        shdisp_bdic_API_LCD_BKL_set_emg_mode(SHDISP_BDIC_BKL_EMG_OFF);
        break;
    case SHDISP_MAIN_BKL_EMG_ON_LEVEL0:
        SHDISP_DEBUG("EMG_ON_LEVEL0");
        shdisp_bdic_API_LCD_BKL_set_emg_mode(SHDISP_BDIC_BKL_EMG_ON_LEVEL0);
        break;
    case SHDISP_MAIN_BKL_EMG_ON_LEVEL1:
        SHDISP_DEBUG("EMG_ON_LEVEL1");
        shdisp_bdic_API_LCD_BKL_set_emg_mode(SHDISP_BDIC_BKL_EMG_ON_LEVEL1);
        break;
    }

    shdisp_kerl_ctx.thermal_status = emg_mode;
    shdisp_bdic_bkl_notify(SHDISP_INFO_BACKLIGHT_LEV);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_LOWBKL
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_lowbkl_mode                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_bkl_set_lowbkl_mode(int lowbkl_mode)
{
    if (lowbkl_mode == SHDISP_MAIN_BKL_LOWBKL_MODE_OFF) {
        SHDISP_DEBUG("BKL_LOWBKL_MODE : OFF");
        shdisp_bdic_API_LCD_BKL_lowbkl_off();
    } else if (lowbkl_mode == SHDISP_MAIN_BKL_LOWBKL_MODE_ON) {
        SHDISP_DEBUG("BKL_LOWBKL_MODE : ON");
        shdisp_bdic_API_LCD_BKL_lowbkl_on();
    } else {
        SHDISP_ERR("<INVALID_VALUE> lowbkl_mode(%d).", lowbkl_mode);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_bkl_notify(SHDISP_INFO_BACKLIGHT_LEV);

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_LOWBKL */

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_chg_mode                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_main_bkl_set_chg_mode(int chg_mode)
{
    if (chg_mode == SHDISP_MAIN_BKL_CHG_OFF) {
        SHDISP_DEBUG("BKL_CHG_MODE : OFF");
        shdisp_bdic_API_LCD_BKL_chg_off();
    } else if (chg_mode == SHDISP_MAIN_BKL_CHG_ON) {
        SHDISP_DEBUG("BKL_CHG_MODE : ON");
        shdisp_bdic_API_LCD_BKL_chg_on();
    } else {
        SHDISP_ERR("<INVALID_VALUE> chg_mode(%d).", chg_mode);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_kerl_ctx.usb_chg_status = chg_mode;
    shdisp_bdic_bkl_notify(SHDISP_INFO_BACKLIGHT_LEV);
    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_TRV_NM2
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_trv_param                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_trv_param(struct shdisp_trv_param param)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_LCD_BKL_trv_param(param);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_LCD_BKL_trv_param.");
        return -EIO;
    }
    shdisp_kerl_ctx.trv_status = param.request;
    shdisp_bdic_bkl_notify(SHDISP_INFO_BACKLIGHT_LEV);

    SHDISP_TRACE("out");
    return ret;
}
#endif /* SHDISP_TRV_NM2 */

#ifdef SHDISP_SYSFS_LED
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_tri_led_on                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_tri_led_on(int no, struct shdisp_tri_led *led)
{
    struct shdisp_tri_led sys_led;
    int ret;

    if (shdisp_led_set_color_reject()) {
        SHDISP_DEBUG("reject request.");
        return SHDISP_RESULT_SUCCESS;
    }

    memset(&sys_led, 0x00, sizeof(sys_led));
    switch (no) {
    case SYSFS_LED_SH_LED_1:
        sys_led = shdisp_kerl_ctx.sysfs_led1;
        break;
#ifdef SHDISP_COLOR_LED_TWIN
    case SYSFS_LED_SH_LED_2:
        sys_led = shdisp_kerl_ctx.sysfs_led2;
        break;
#endif /* SHDISP_COLOR_LED_TWIN */
    default:
        break;
    }

     if ((sys_led.red   == led->red) &&
        (sys_led.green == led->green) &&
        (sys_led.blue  == led->blue)) {
        SHDISP_DEBUG("same leds request.");
        return SHDISP_RESULT_SUCCESS;
    }

    if ((led->red   == 0) &&
        (led->green == 0) &&
        (led->blue  == 0)) {
        SHDISP_DEBUG("leds off. no=%d", no);
        led->led_mode = SHDISP_TRI_LED_MODE_OFF;
        ret = shdisp_bdic_API_LED_off(no);
    } else {
        led->led_mode = SHDISP_TRI_LED_MODE_NORMAL;

        SHDISP_DEBUG("leds on. no=%d, red=%d, green:%d, blue:%d",
                      no, led->red, led->green, led->blue);
        ret = shdisp_bdic_API_LED_on(no, *led);
        shdisp_clean_normal_led();
    }

    switch (no) {
    case SYSFS_LED_SH_LED_1:
        shdisp_kerl_ctx.sysfs_led1 = *led;
        break;
#ifdef SHDISP_COLOR_LED_TWIN
    case SYSFS_LED_SH_LED_2:
        shdisp_kerl_ctx.sysfs_led2 = *led;
        break;
#endif /* SHDISP_COLOR_LED_TWIN */
    default:
        break;
    }

    return ret;
}
#endif /* SHDISP_SYSFS_LED */

/* ------------------------------------------------------------------------- */
/* shdisp_tri_led_set_color                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_tri_led_set_color(struct shdisp_tri_led *tri_led)
{

    unsigned char color, xstb_ch012;
    struct shdisp_tri_led  led;

    led.red   = tri_led->red;
    led.green = tri_led->green;
    led.blue  = tri_led->blue;
    color = shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(&led);

    if (tri_led->led_mode == SHDISP_TRI_LED_MODE_NORMAL) {
        if ((shdisp_kerl_ctx.tri_led.red      == led.red) &&
            (shdisp_kerl_ctx.tri_led.green    == led.green) &&
            (shdisp_kerl_ctx.tri_led.blue     == led.blue) &&
            (shdisp_kerl_ctx.tri_led.led_mode == tri_led->led_mode)) {
            return SHDISP_RESULT_SUCCESS;
        }
    }

    xstb_ch012 = (color == 0) ? 0 : 1;

    if (xstb_ch012 != 0) {
        SHDISP_DEBUG("led_mode=%d color:%d, ontime:%d, interval:%d, count:%d",
                      tri_led->led_mode, color, tri_led->ontime, tri_led->interval, tri_led->count);

        switch (tri_led->led_mode) {
        case SHDISP_TRI_LED_MODE_NORMAL:
            shdisp_bdic_API_TRI_LED_normal_on(color);
            break;
        case SHDISP_TRI_LED_MODE_BLINK:
            shdisp_bdic_API_TRI_LED_blink_on(color, tri_led->ontime, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_FIREFLY:
            shdisp_bdic_API_TRI_LED_firefly_on(color, tri_led->ontime, tri_led->interval, tri_led->count);
            break;
#ifdef SHDISP_ANIME_COLOR_LED
#ifdef SHDISP_ILLUMI_COLOR_LED
        case SHDISP_TRI_LED_MODE_HISPEED:
            shdisp_bdic_API_TRI_LED_high_speed_on(color, tri_led->interval, tri_led->count);
            break;
#ifndef CONFIG_ARCH_PE46
        case SHDISP_TRI_LED_MODE_STANDARD:
            shdisp_bdic_API_TRI_LED_standard_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_BREATH:
            shdisp_bdic_API_TRI_LED_breath_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_LONG_BREATH:
            shdisp_bdic_API_TRI_LED_long_breath_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_WAVE:
            shdisp_bdic_API_TRI_LED_wave_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_FLASH:
            shdisp_bdic_API_TRI_LED_flash_on(color, tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_AURORA:
            shdisp_bdic_API_TRI_LED_aurora_on(tri_led->interval, tri_led->count);
            break;
        case SHDISP_TRI_LED_MODE_RAINBOW:
            shdisp_bdic_API_TRI_LED_rainbow_on(tri_led->interval, tri_led->count);
            break;
#endif  /* CONFIG_ARCH_PE46 */
#endif  /* SHDISP_ILLUMI_COLOR_LED */
        case SHDISP_TRI_LED_MODE_EMOPATTERN:
            shdisp_bdic_API_TRI_LED_emopattern_on(tri_led->interval, tri_led->count);
            break;
#endif  /* SHDISP_ANIME_COLOR_LED */
        default:
            SHDISP_ERR("led_mode=%d not supported.", tri_led->led_mode);
            break;
        }
#ifdef SHDISP_SYSFS_LED
        shdisp_clean_sysfs_led();
#endif /* SHDISP_SYSFS_LED */
    } else {
        shdisp_bdic_API_TRI_LED_off();
    }

    shdisp_kerl_ctx.tri_led.red      = led.red;
    shdisp_kerl_ctx.tri_led.green    = led.green;
    shdisp_kerl_ctx.tri_led.blue     = led.blue;
    shdisp_kerl_ctx.tri_led.led_mode = tri_led->led_mode;
    switch (tri_led->led_mode) {
    case SHDISP_TRI_LED_MODE_NORMAL:
        break;
    case SHDISP_TRI_LED_MODE_BLINK:
    case SHDISP_TRI_LED_MODE_FIREFLY:
        shdisp_kerl_ctx.tri_led.ontime   = tri_led->ontime;
        shdisp_kerl_ctx.tri_led.interval = tri_led->interval;
        shdisp_kerl_ctx.tri_led.count    = tri_led->count;
        break;
#ifdef SHDISP_ANIME_COLOR_LED
    case SHDISP_TRI_LED_MODE_EMOPATTERN:
        shdisp_kerl_ctx.tri_led.red = 1;
        shdisp_kerl_ctx.tri_led.green = 0;
        shdisp_kerl_ctx.tri_led.blue = 1;
        shdisp_kerl_ctx.tri_led.interval = 0;
        shdisp_kerl_ctx.tri_led.count = 0;
        break;
#endif  /* SHDISP_ANIME_COLOR_LED */
    default:
        shdisp_kerl_ctx.tri_led.interval = tri_led->interval;
        shdisp_kerl_ctx.tri_led.count    = tri_led->count;
        break;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_led_set_color_reject                                               */
/* ------------------------------------------------------------------------- */
static bool shdisp_led_set_color_reject(void)
{
    bool ret = false;
    if (shdisp_kerl_ctx.led_set_color_reject) {
        ret = true;
    }
#ifdef SHDISP_FPS_LED_PANEL_SUPPORT
    if (shdisp_fps_led_ctx.enable) {
        ret = true;
    }
#endif /* SHDISP_FPS_LED_PANEL_SUPPORT */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_tri_led_set_color                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_tri_led_set_color(struct shdisp_tri_led *tri_led)
{
    if (shdisp_led_set_color_reject()) {
        SHDISP_DEBUG("reject request.");
        return SHDISP_RESULT_SUCCESS;
    }

    return shdisp_tri_led_set_color(tri_led);
}
#ifdef SHDISP_KEY_LED
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_key_bkl_ctl                                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_key_bkl_ctl(struct shdisp_key_bkl_ctl *key_bkl_ctl)
{
    unsigned char index;
    unsigned char dim = 0;
    struct shdisp_key_bkl_ctl key_bkl;

    key_bkl = *key_bkl_ctl;

    if (key_bkl.key_left == SHDISP_KEY_BKL_DIM) {
        dim |= (1<<SHDISP_KEY_BKL_LEFT);
    }
    if (key_bkl.key_center == SHDISP_KEY_BKL_DIM) {
        dim |= (1<<SHDISP_KEY_BKL_CENTER);
    }
    if (key_bkl.key_right == SHDISP_KEY_BKL_DIM) {
        dim |= (1<<SHDISP_KEY_BKL_RIGHT);
    }
    dim &= 0x07;
#ifdef SHDISP_IR2E71Y8
    index = shdisp_bdic_API_KEY_LED_get_color_index_and_reedit( &key_bkl );
#endif /* SHDISP_IR2E71Y8 */
    if ((shdisp_kerl_ctx.key_bkl_ctl.key_left== key_bkl.key_left) &&
        (shdisp_kerl_ctx.key_bkl_ctl.key_center == key_bkl.key_center) &&
        (shdisp_kerl_ctx.key_bkl_ctl.key_right== key_bkl.key_right) &&
        (shdisp_kerl_ctx.key_bkl_ctl.ontime == key_bkl.ontime) &&
        (shdisp_kerl_ctx.key_bkl_ctl.interval == key_bkl.interval)) {
        return SHDISP_RESULT_SUCCESS;
    }

    SHDISP_DEBUG("{key_left,key_center,key_right}={%d,%d,%d}, ontime:%d, interval:%d\n",
            key_bkl_ctl->key_left, key_bkl_ctl->key_center, key_bkl_ctl->key_right, key_bkl_ctl->ontime, key_bkl_ctl->interval);
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_KEY_LED_ctl(dim, index, key_bkl.ontime, key_bkl.interval);
#endif /* SHDISP_IR2E71Y8 */
    shdisp_kerl_ctx.key_bkl_ctl = key_bkl;

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_KEY_LED */
#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
/* ------------------------------------------------------------------------- */
/* shdisp_set_illumi_triple_color                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_set_illumi_triple_color(struct shdisp_illumi_triple_color * illumi_triple_color)
{
    shdisp_bdic_API_LED_set_illumi_triple_color(*illumi_triple_color);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_illumi_triple_color                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_illumi_triple_color(struct shdisp_illumi_triple_color * illumi_triple_color)
{
    int ret;

    shdisp_clean_sysfs_led();
    ret =  shdisp_set_illumi_triple_color(illumi_triple_color);

    shdisp_kerl_ctx.tri_led.red = illumi_triple_color->colors[ILLUMI_FRAME_FIRST].red;
    shdisp_kerl_ctx.tri_led.green = illumi_triple_color->colors[ILLUMI_FRAME_FIRST].green;
    shdisp_kerl_ctx.tri_led.blue = illumi_triple_color->colors[ILLUMI_FRAME_FIRST].blue;
    shdisp_kerl_ctx.tri_led.led_mode = SHDISP_TRI_LED_MODE_ILLUMI_TRIPLE_COLOR;

    return ret;
}
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_write_reg                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_bdic_write_reg(unsigned char reg, unsigned char val)
{
    shdisp_bdic_API_DIAG_write_reg(reg, val);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_read_reg                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_bdic_read_reg(unsigned char reg, unsigned char *val)
{
    shdisp_bdic_API_DIAG_read_reg(reg, val);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_multi_read_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_bdic_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;

    ret = shdisp_bdic_API_DIAG_multi_read_reg(reg, val, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_lux                                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_lux(struct shdisp_photo_sensor_val *value)
{
    int ret;

    ret = shdisp_bdic_API_PHOTO_SENSOR_get_lux(&(value->value), &(value->lux));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_get_lux.");
        value->result = SHDISP_RESULT_FAILURE;
    } else {
        value->result = SHDISP_RESULT_SUCCESS;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_als                                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_als(struct shdisp_photo_sensor_raw_val *raw_val)
{
    int ret;

    ret = shdisp_bdic_API_PHOTO_SENSOR_get_raw_als(&(raw_val->clear), &(raw_val->ir));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_get_raw_als.");
        raw_val->result = SHDISP_RESULT_FAILURE;
    } else {
        raw_val->result = SHDISP_RESULT_SUCCESS;
    }

    return ret;
}

#ifdef SHDISP_ALS_INT
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_alsint                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_alsint(struct shdisp_photo_sensor_int_trigger *val)
{
    int ret;
    int trigger = 0;

    ret = shdisp_bdic_API_PHOTO_SENSOR_set_alsint(val);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_set_alsint.");
    }

    if (val->result == SHDISP_RESULT_SUCCESS) {
        if (val->trigger1.enable | val->trigger2.enable) {
            if (val->trigger1.enable) {
                trigger = trigger | SHDISP_OPT_CHANGE_INT_1;
            }
            if (val->trigger2.enable) {
                trigger = trigger | SHDISP_OPT_CHANGE_INT_2;
            }
            SHDISP_DEBUG("<OTHER>als int is enable");
            shdisp_set_als_int_subscribe(trigger);
        } else {
            SHDISP_DEBUG("<OTHER>als int is disable");
            shdisp_set_als_int_unsubscribe(SHDISP_OPT_CHANGE_INT_1 | SHDISP_OPT_CHANGE_INT_2);
        }
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_alsint                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_alsint(struct shdisp_photo_sensor_int_trigger *val)
{
    int ret;

    ret = shdisp_bdic_API_PHOTO_SENSOR_get_alsint(val);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_get_alsint.");
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_light_info                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_light_info(struct shdisp_light_info *val)
{
    int ret;

    ret = shdisp_bdic_API_PHOTO_SENSOR_get_light_info(val);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_get_light_info.");
    }
    return ret;
}
#endif /* SHDISP_ALS_INT */

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_write_bdic_i2c                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;

    ret = shdisp_bdic_API_i2c_transfer(i2c_msg);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_bdic_API_i2c_transfer.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    } else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_i2c_transfer.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_read_bdic_i2c                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;

    ret = shdisp_bdic_API_i2c_transfer(i2c_msg);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_bdic_API_i2c_transfer.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    } else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_i2c_transfer.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_photo_sensor_pow_ctl                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_photo_sensor_pow_ctl(struct shdisp_photo_sensor_power_ctl *ctl)
{
    int ret;

    ret = shdisp_bdic_API_als_sensor_pow_ctl(ctl->type, ctl->power);
    if (ret != SHDISP_RESULT_SUCCESS) {
#ifdef SHDISP_ALS_INT
        if (ret == SHDISP_RESULT_ALS_INT_OFF) {
            SHDISP_DEBUG("<OTHER>als int is disable");
            shdisp_set_als_int_unsubscribe(SHDISP_OPT_CHANGE_INT_1 | SHDISP_OPT_CHANGE_INT_2);
            return SHDISP_RESULT_SUCCESS;
        }
#endif /* SHDISP_ALS_INT */
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_als_sensor_pow_ctl.");
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_panel_write_reg                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_panel_write_reg(struct shdisp_lcddr_reg *panel_reg)
{
    int ret;

    ret = shdisp_panel_API_diag_write_reg(panel_reg->address, panel_reg->buf, panel_reg->size);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_write_reg.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_panel_read_reg                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_panel_read_reg(struct shdisp_lcddr_reg *panel_reg)
{
    int ret;

    ret = shdisp_panel_API_diag_read_reg(panel_reg->address, panel_reg->buf, panel_reg->size);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_read_reg.");
        return SHDISP_RESULT_FAILURE;
    }


    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_prox_sensor_pow_ctl                                            */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_IR2E71Y8
static int shdisp_SQE_prox_sensor_pow_ctl(int power_mode)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in power_mode=%d", power_mode);

    switch (power_mode) {
    case SHDISP_PROX_SENSOR_POWER_OFF:
        shdisp_pm_API_ps_user_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_OFF);
        break;

    case SHDISP_PROX_SENSOR_POWER_ON:
        shdisp_pm_API_ps_user_manager(SHDISP_DEV_TYPE_PS, SHDISP_DEV_REQ_ON);
        break;

    default:
        ret = SHDISP_RESULT_FAILURE;
        SHDISP_ERR("POWER_MODE ERROR(mode=%d)", power_mode);
        break;
    }

    SHDISP_TRACE("out ret=%d", ret);
    return ret;

}
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_flicker_param                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_flicker_param(struct shdisp_diag_flicker_param vcom)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_set_flicker_param(vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_set_flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_flicker_param                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_flicker_param(struct shdisp_diag_flicker_param *vcom)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_get_flicker_param(vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_get_flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_flicker_low_param                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_get_flicker_low_param(vcom);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_get_flicker_low_param.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_check_recovery                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_check_recovery(void)
{
    int ret;

    ret = shdisp_panel_API_check_recovery();

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_check_recovery.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

#ifndef SHDISP_USE_QUALCOMM_RECOVERY
#ifndef SHDISP_NOT_SUPPORT_DET
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_do_recovery                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_do_recovery(void)
{
    SHDISP_DEBUG("recovery : start");

#ifdef SHDISP_IR2E71Y8
    shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_RECOVERY, SHDISP_DEV_REQ_ON);
#endif /* SHDISP_IR2E71Y8 */
    shdisp_SQE_main_lcd_disp_off();

#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_SAZABI)
    mdss_shdisp_video_transfer_ctrl(false, false);
    mdss_shdisp_set_lp00_mode(false);
#endif /* CONFIG_SHDISP_PANEL */

    shdisp_panel_API_power_off(SHDISP_PANEL_POWER_RECOVERY_OFF);

    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_OFF;

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    shdisp_IO_API_msleep(500);
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
    shdisp_IO_API_msleep(40);
#elif defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    shdisp_IO_API_msleep(100);
#elif defined(CONFIG_SHDISP_PANEL_SAZABI)
    shdisp_IO_API_msleep(100);
#else  /* CONFIG_SHDISP_PANEL_XXXX */
#warning "unknown panel!!!"
    shdisp_IO_API_msleep(500);
#endif  /* CONFIG_SHDISP_PANEL */

#if defined(CONFIG_SHDISP_PANEL_ARIA) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    mdss_shdisp_cmd_tearcheck_enable(false);
#endif  /* CONFIG_SHDISP_PANEL */

    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_ON;
    shdisp_panel_API_power_on(SHDISP_PANEL_POWER_RECOVERY_ON);

#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_SAZABI)
    mdss_shdisp_set_lp00_mode(true);
#endif /* CONFIG_SHDISP_PANEL */

    shdisp_panel_API_disp_on();

#if defined(CONFIG_SHDISP_PANEL_ARIA) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    mdss_shdisp_mdp_cmd_kickoff();
#endif  /* CONFIG_SHDISP_PANEL */

    shdisp_panel_API_start_display();

#if defined(CONFIG_SHDISP_PANEL_ARIA) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    mdss_shdisp_cmd_tearcheck_enable(true);
    shdisp_IO_API_msleep(17);
#endif  /* CONFIG_SHDISP_PANEL */

#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_SAZABI)
    mdss_shdisp_video_transfer_ctrl(true, false);
#endif /* CONFIG_SHDISP_PANEL */

    shdisp_panel_API_post_video_start();
#ifdef SHDISP_IR2E71Y8
    shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_RECOVERY, SHDISP_DEV_REQ_OFF);
#endif /* SHDISP_IR2E71Y8 */
    SHDISP_DEBUG("recovery : end");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_NOT_SUPPORT_DET */
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_event_subscribe                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_event_subscribe(struct shdisp_subscribe *subscribe)
{
    int ret;
    int i;
    int bAllNull = 0;

    if (shdisp_subscribe_type_table[subscribe->irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
#ifdef SHDISP_IR2E71Y8
        ret = shdisp_bdic_API_IRQ_check_type(subscribe->irq_type);
#endif /* SHDISP_IR2E71Y8 */
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_IRQ_check_type.");
            return SHDISP_RESULT_FAILURE;
        }
    }

    down(&shdisp_sem_callback);

    bAllNull = 1;
    for (i = 0; i < NUM_SHDISP_IRQ_TYPE; i++) {
        if ((shdisp_subscribe_type_table[i] == SHDISP_SUBSCRIBE_TYPE_INT) &&
            (shdisp_callback_table[i] != NULL)) {
            bAllNull = 0;
        }
    }

    if (shdisp_callback_table[subscribe->irq_type] != NULL) {
        SHDISP_DEBUG("INT_SUBSCRIBE CHANGE(irq_type=%d)", subscribe->irq_type);
    } else {
        SHDISP_DEBUG("INT_SUBSCRIBE NEW ENTRY(irq_type=%d)", subscribe->irq_type);
    }

    shdisp_callback_table[subscribe->irq_type] = subscribe->callback;

    if (shdisp_subscribe_type_table[subscribe->irq_type] != SHDISP_SUBSCRIBE_TYPE_INT) {
        shdisp_timer_int_register();
    }
#ifdef SHDISP_IR2E71Y8
    if (subscribe->irq_type == SHDISP_IRQ_TYPE_DET) {
        shdisp_bdic_API_IRQ_det_irq_ctrl(true);
    }
#endif /* SHDISP_IR2E71Y8 */
    up(&shdisp_sem_callback);

    if (shdisp_subscribe_type_table[subscribe->irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        if (bAllNull) {
            SHDISP_DEBUG("INT_SUBSCRIBE enable_irq");
            shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);
        }
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_event_unsubscribe                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_event_unsubscribe(int irq_type)
{
    int ret;
    int i;
    int bAllNull = 0;

    if (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
#ifdef SHDISP_IR2E71Y8
        ret = shdisp_bdic_API_IRQ_check_type(irq_type);
#endif /* SHDISP_IR2E71Y8 */
        if (ret !=  SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_IRQ_check_type.");
            return SHDISP_RESULT_FAILURE;
        }
    }

    down(&shdisp_sem_callback);

    if (shdisp_callback_table[irq_type] == NULL) {
        SHDISP_DEBUG("INT_UNSUBSCRIBE DONE(irq_type=%d)", irq_type);
    } else {
        if (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        } else {
            shdisp_timer_int_delete();
        }
#ifdef SHDISP_IR2E71Y8
        if (irq_type == SHDISP_IRQ_TYPE_DET) {
            shdisp_bdic_API_IRQ_det_irq_ctrl(false);
        }
#endif /* SHDISP_IR2E71Y8 */
        shdisp_callback_table[irq_type] = NULL;

        if (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
            bAllNull = 1;
            for (i = 0; i < NUM_SHDISP_IRQ_TYPE; i++) {
                if ((shdisp_subscribe_type_table[i] == SHDISP_SUBSCRIBE_TYPE_INT) &&
                    (shdisp_callback_table[i] != NULL)) {
                    bAllNull = 0;
                }
            }
            if (bAllNull) {
                shdisp_SYS_API_set_irq(SHDISP_IRQ_DISABLE);
                SHDISP_DEBUG("INT_UNSUBSCRIBE disable_irq");
            }
        }

        SHDISP_DEBUG("INT_UNSUBSCRIBE SUCCESS(irq_type=%d)", irq_type);
    }

    up(&shdisp_sem_callback);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_gmmtable_and_voltage                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_set_gmmtable_and_voltage(gmm_info);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_set_gmmtable_and_voltage.");
        return SHDISP_RESULT_FAILURE;
    }


    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_gmmtable_and_voltage                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_get_gmmtable_and_voltage(gmm_info);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_get_gmmtable_and_voltage.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_gmm                                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_gmm(struct shdisp_diag_gamma *gmm)
{
    int ret;

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_panel_API_diag_set_gmm(gmm);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_set_gmm.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}
#ifdef SHDISP_IR2E71Y8
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_ave_ado                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_get_ave_ado(struct shdisp_ave_ado *ave_ado)
{
    int ret;

    if (ave_ado->als_range >= NUM_SHDISP_MAIN_DISP_ALS_RANGE) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_bdic_API_get_ave_ado(ave_ado);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_get_ave_ado.");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}
#else /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_panel_mfr_ctl                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_panel_mfr_ctl(struct shdisp_panel_mode p_mode)
{
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.shutdown_in_progress) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        SHDISP_DEBUG("out4");
        return SHDISP_RESULT_SUCCESS;
    }

    if ((p_mode.bkl_param == shdisp_kerl_ctx.main_bkl.param) &&
        (p_mode.trv_mode == shdisp_kerl_ctx.trv_status)) {
        SHDISP_DEBUG("out5");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_panel_API_chg_mode(p_mode);
    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_IR2E71Y8 */

#ifndef SHDISP_USE_QUALCOMM_RECOVERY
#ifndef SHDISP_NOT_SUPPORT_DET
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_lcd_det_recovery                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_SQE_lcd_det_recovery(void)
{
    int i;
    int retry_max = 3;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
    int err_code_reset;
    unsigned char subcode;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in");
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_IRQ_det_irq_ctrl(false);
#endif /* SHDISP_IR2E71Y8 */
#ifdef SHDISP_RESET_LOG
    subcode = shdisp_dbg_API_get_subcode();
#endif /* SHDISP_RESET_LOG */
    SHDISP_DEBUG("retry_max=%d", retry_max);
    for (i = 0; i < retry_max; i++) {

        shdisp_SQE_do_recovery();

        if (shdisp_SQE_check_recovery() == SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("recovery completed");
#ifdef CONFIG_SHTERM
            shterm_k_set_info(SHTERM_INFO_LCDPOW, 1);
#endif  /* CONFIG_SHTERM */
            shdisp_clear_recovery_lcd_queued();
#ifdef SHDISP_IR2E71Y8
            shdisp_bdic_API_IRQ_det_irq_ctrl(true);
#endif /* SHDISP_IR2E71Y8 */
            /* enable REQOUT error interrupt for andy,hayabusa panel */
#if defined(CONFIG_SHDISP_PANEL_ANDY)
            shdisp_IO_API_delay_us(20 * 1000);
#endif  /* CONFIG_SHDISP_PANEL_ANDY */

#ifdef SHDISP_DET_DSI_MIPI_ERROR
            /* reject the mipi error detecting contention */
            mdss_shdisp_dsi_mipi_err_clear();
#endif  /* SHDISP_DET_DSI_MIPI_ERROR */

            /* enable the mipi error detection */
            shdisp_det_mipi_err_ctrl(true);

#ifdef SHDISP_RESET_LOG
            shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_NONE);
#endif /* SHDISP_RESET_LOG */
            return;
        }
        SHDISP_WARN("recovery retry(%d)", i);
    }

    shdisp_clear_recovery_lcd_queued();

    SHDISP_ERR("recovery retry over");
#ifdef SHDISP_RESET_LOG
    err_code.mode = SHDISP_DBG_MODE_LINUX;
    err_code.type = SHDISP_DBG_TYPE_PANEL;
    err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
    err_code.subcode = subcode;
    err_code_reset = 0;
 #if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_API_get_reset_flg() & SHDISP_DBG_RESET_PANEL_RETRY_OVER) {
        err_code_reset = 1;
    }
 #endif /* defined (CONFIG_ANDROID_ENGINEERING) */
    shdisp_dbg_API_err_output(&err_code, err_code_reset);
    shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_NONE);
#endif /* SHDISP_RESET_LOG */
    return;
}
#endif /* SHDISP_NOT_SUPPORT_DET */
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_psals_recovery                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_psals_recovery(void)
{
    int result = SHDISP_RESULT_SUCCESS;
#ifdef SHDISP_IR2E71Y8
    int ps_flg = 0;
#endif /* SHDISP_IR2E71Y8 */
    SHDISP_TRACE("in");
#ifdef SHDISP_IR2E71Y8
    ps_flg = shdisp_pm_API_is_ps_active();

    if (ps_flg == SHDISP_DEV_STATE_ON) {
        /* notify to proximity module that recovery is staring */
#ifdef CONFIG_PROXIMITY_INT_HOST
        PROX_recovery_start_func();
#endif /* CONFIG_PROXIMITY_INT_HOST */
    }
#endif /* SHDISP_IR2E71Y8 */

    shdisp_semaphore_start();
#ifdef SHDISP_IR2E71Y8
    shdisp_pm_API_psals_power_off();
#endif /* SHDISP_IR2E71Y8 */
    shdisp_IO_API_delay_us(10 * 1000);
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_IRQ_i2c_error_Clear();

    shdisp_pm_API_psals_error_power_recovery();

#endif /* SHDISP_IR2E71Y8 */
    down(&shdisp_sem_req_recovery_psals);
    shdisp_recovery_psals_queued_flag = 0;
    up(&shdisp_sem_req_recovery_psals);

#ifdef SHDISP_IR2E71Y8
    result = shdisp_bdic_API_psals_is_recovery_successful();
#endif /* SHDISP_IR2E71Y8 */
    if (result != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("PALS Recovery Error!!");
    }

    shdisp_semaphore_end(__func__);

#ifdef SHDISP_IR2E71Y8
    if (ps_flg == SHDISP_DEV_STATE_ON) {

        /* notify to proximity module that recovery is ending */
       shdisp_IO_API_msleep(20);
#ifdef CONFIG_PROXIMITY_INT_HOST
        PROX_recovery_end_func();
#endif /* CONFIG_PROXIMITY_INT_HOST */
    }
#endif /* SHDISP_IR2E71Y8 */

    SHDISP_DEBUG("main_disp_status=%d", shdisp_kerl_ctx.main_disp_status)
    if ((result == SHDISP_RESULT_SUCCESS) ||
            (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON)) {
        SHDISP_DEBUG("enable_irq for psals_recovery");
        shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_irq_mask                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_set_irq_mask(int irq_msk_ctl)
{
    SHDISP_TRACE("in");

    if (irq_msk_ctl == SHDISP_IRQ_NO_MASK) {
        shdisp_det_mipi_err_ctrl(true);
        shdisp_lcd_det_recovery_subscribe();
    } else {
        shdisp_det_mipi_err_ctrl(false);
        shdisp_lcd_det_recovery_unsubscribe();
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_vcom_tracking                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_vcom_tracking(int tracking)
{
    int ret;

    SHDISP_TRACE("in");

#if defined(CONFIG_SHDISP_PANEL_ANDY)
    ret = shdisp_andy_API_vcom_tracking(tracking);
#else /* CONFIG_SHDISP_PANEL_ANDY */
    ret = SHDISP_RESULT_SUCCESS;
#endif /* CONFIG_SHDISP_PANEL_ANDY */

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_semaphore_start                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_semaphore_start(void)
{
    SHDISP_INFO("in");
    down(&shdisp_sem);
    SHDISP_INFO("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_semaphore_end                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_semaphore_end(const char *func)
{
    SHDISP_INFO("in");
    up(&shdisp_sem);
    SHDISP_INFO("out");
}

/* ------------------------------------------------------------------------- */
/* INTERRUPT                                                                 */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_IR2E71Y8
/* ------------------------------------------------------------------------- */
/* shdisp_gpio_int_isr                                                       */
/* ------------------------------------------------------------------------- */
static irqreturn_t shdisp_gpio_int_isr(int irq_num, void *data)
{
    irqreturn_t rc = IRQ_HANDLED;
    int ret;
    unsigned long flags = 0;

    shdisp_SYS_API_set_irq(SHDISP_IRQ_DISABLE);

    spin_lock_irqsave(&shdisp_q_lock, flags);

    SHDISP_TRACE(":Start");

    if (shdisp_wq_gpio) {
        shdisp_wake_lock();
        ret = queue_work(shdisp_wq_gpio, &shdisp_wq_gpio_wk);
        if (ret == 0) {
            shdisp_wake_unlock();
            SHDISP_ERR("<QUEUE_WORK_FAILURE>");
        }
    }
    spin_unlock_irqrestore(&shdisp_q_lock, flags);

    return rc;
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_gpio                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_gpio(struct work_struct *work)
{
    struct shdisp_queue_data_t *qdata = NULL;
    int    i;
    int    bFirstQue = 0;
    int    ret;
    int    nBDIC_QueFac = 0;

    SHDISP_TRACE("Start");

    shdisp_semaphore_start();
    shdisp_bdic_API_IRQ_save_fac();
    do {
        ret = shdisp_bdic_API_IRQ_check_fac();
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("no factor");
            break;
        }
        down(&shdisp_sem_irq_fac);
        for (i = 0; i < SHDISP_IRQ_MAX_KIND; i++) {
            nBDIC_QueFac = shdisp_bdic_API_IRQ_get_fac(i);
            if (nBDIC_QueFac == SHDISP_BDIC_IRQ_TYPE_NONE) {
                break;
            }
            if (shdisp_wq_gpio_task) {
                qdata = kmalloc(sizeof(shdisp_queue_data), GFP_KERNEL);
                if (qdata != NULL) {
                    qdata->irq_GFAC = nBDIC_QueFac;
                    list_add_tail(&qdata->list, &shdisp_queue_data.list);
                    if (bFirstQue == 0) {
                        bFirstQue = 1;
                        shdisp_wake_lock();
                        ret = queue_work(shdisp_wq_gpio_task, &shdisp_wq_gpio_task_wk);
                        if (ret == 0) {
                            shdisp_wake_unlock();
                            SHDISP_DEBUG("<QUEUE_WORK_FAILURE> queue_work failed");
                        }
                    }
                } else {
                   SHDISP_ERR("<QUEUE_WORK_FAILURE> kmalloc failed (BDIC_QueFac=%d)", nBDIC_QueFac);
                }
            }
        }
        up(&shdisp_sem_irq_fac);

    } while (0);

    shdisp_bdic_API_IRQ_Clear();
    shdisp_semaphore_end(__func__);

    if (shdisp_bdic_API_IRQ_check_DET() != SHDISP_BDIC_IRQ_TYPE_DET &&
        shdisp_bdic_API_IRQ_check_I2C_ERR() != SHDISP_BDIC_IRQ_TYPE_I2C_ERR) {
        SHDISP_DEBUG("enable_irq for \"No DET\" or \"No I2C_ERR\"");
        shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);
    }
    SHDISP_TRACE("Finish");
    shdisp_wake_unlock();
    return;
}
/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_gpio_task                                                */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_gpio_task(struct work_struct *work)
{
    struct list_head *listptr;
    struct shdisp_queue_data_t  *entry;
    struct shdisp_queue_data_t  *entryFirst = NULL;
    int     nFirstBDIC_GFAC = 0;
    int     nFirst_GFAC = -1;
    int     bFirst = 0;
    int     bThrough = 0;
    void (*temp_callback)(void);

    SHDISP_TRACE("Start");

    do {
        down(&shdisp_sem_irq_fac);
        bThrough = 0;
        entryFirst = NULL;
        bFirst = 0;
        nFirstBDIC_GFAC = 0;
        list_for_each(listptr, &shdisp_queue_data.list) {
            entry = list_entry( listptr, struct shdisp_queue_data_t, list);
            if (bFirst == 0) {
                entryFirst = entry;
                nFirstBDIC_GFAC = entry->irq_GFAC;
                bFirst = 1;
            } else {
                if (entry->irq_GFAC == nFirstBDIC_GFAC) {
                    bThrough = 1;
                }
            }
        }

        if (entryFirst != NULL) {
            list_del(&entryFirst->list);
            kfree(entryFirst);
        } else {
            SHDISP_DEBUG("no entry");
            up(&shdisp_sem_irq_fac);
            break;
        }
        up(&shdisp_sem_irq_fac);


        if (bThrough == 0) {
            if (nFirstBDIC_GFAC == SHDISP_BDIC_IRQ_TYPE_NONE) {
                SHDISP_DEBUG("failed (no BDIC_GFAC=%d)", nFirstBDIC_GFAC);
            } else {
                nFirst_GFAC = -1;
                switch (nFirstBDIC_GFAC) {
                case SHDISP_BDIC_IRQ_TYPE_ALS:
                        nFirst_GFAC = SHDISP_IRQ_TYPE_ALS_REQ;
                        break;
                case SHDISP_BDIC_IRQ_TYPE_PS:
                        nFirst_GFAC = SHDISP_IRQ_TYPE_PS;
                        break;
                default:
                        break;
                }

                SHDISP_DEBUG("Callback[%d] Start", nFirstBDIC_GFAC);
                if (nFirst_GFAC >= 0) {
                    down(&shdisp_sem_callback);
                    temp_callback = shdisp_callback_table[nFirst_GFAC];
                    up(&shdisp_sem_callback);

                    if (temp_callback != NULL) {
                        (*temp_callback)();
                    } else {
                        SHDISP_DEBUG("Callback is Nulle pointer(irq_type=%d)", nFirst_GFAC);
                    }
                } else {
                    switch (nFirstBDIC_GFAC) {
                    case SHDISP_BDIC_IRQ_TYPE_DET:
                        shdisp_do_detin_recovery();
                        break;
                    case SHDISP_BDIC_IRQ_TYPE_I2C_ERR:
                        shdisp_do_psals_recovery();
                        break;
#ifdef SHDISP_ALS_INT
                    case SHDISP_BDIC_IRQ_TYPE_ALS_TRIGGER:
                        shdisp_do_als_int_report(SHDISP_OPT_CHANGE_INT_1 | SHDISP_OPT_CHANGE_INT_2);
                        break;
                    case SHDISP_BDIC_IRQ_TYPE_ALS_TRIGGER1:
                        shdisp_do_als_int_report(SHDISP_OPT_CHANGE_INT_1);
                        break;
                    case SHDISP_BDIC_IRQ_TYPE_ALS_TRIGGER2:
                        shdisp_do_als_int_report(SHDISP_OPT_CHANGE_INT_2);
                        break;
#endif /* SHDISP_ALS_INT */
                    default:
                        break;
                    }
                }
            }
        } else {
            SHDISP_DEBUG("Skip (BDIC_GFAC=%d)", nFirstBDIC_GFAC);
        }
    } while (1);


    SHDISP_TRACE("Finish");
    shdisp_wake_unlock();

    return;
}
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_wake_lock_init                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_wake_lock_init(void)
{
    unsigned long flags = 0;

    spin_lock_irqsave(&shdisp_wake_spinlock, flags);
    shdisp_wake_lock_wq_refcnt = 0;
    wake_lock_init(&shdisp_wake_lock_wq, WAKE_LOCK_SUSPEND, "shdisp_wake_lock_wq");
    spin_unlock_irqrestore(&shdisp_wake_spinlock, flags);
}

/* ------------------------------------------------------------------------- */
/* shdisp_wake_lock                                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_wake_lock(void)
{
    unsigned long flags = 0;

    spin_lock_irqsave(&shdisp_wake_spinlock, flags);
    if (shdisp_wake_lock_wq_refcnt++ == 0) {
        wake_lock(&shdisp_wake_lock_wq);
    }
    spin_unlock_irqrestore(&shdisp_wake_spinlock, flags);
}

/* ------------------------------------------------------------------------- */
/* shdisp_wake_unlock                                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_wake_unlock(void)
{
    unsigned long flags = 0;

    spin_lock_irqsave(&shdisp_wake_spinlock, flags);
    if (--shdisp_wake_lock_wq_refcnt <= 0) {
        wake_unlock(&shdisp_wake_lock_wq);
        shdisp_wake_lock_wq_refcnt = 0;
    }
    spin_unlock_irqrestore(&shdisp_wake_spinlock, flags);
}

/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_isr                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_timer_int_isr(unsigned long data)
{
    int ret;

    SHDISP_DEBUG("Timeout ( registered %ld, now %ld ).", data, jiffies);
    SHDISP_TRACE(":Start");

    if (shdisp_timer_stop) {
        SHDISP_DEBUG("Timer is not to be restarted.");
        return;
    }
    if (shdisp_wq_timer_task) {
        ret = queue_work(shdisp_wq_timer_task, &shdisp_wq_timer_task_wk);
        if (ret == 0) {
            SHDISP_ERR("<QUEUE_WORK_FAILURE> ");
        }
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_register                                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_timer_int_register(void)
{
    down(&shdisp_sem_timer);

    shdisp_timer.expires  = jiffies + (10 * HZ);
    shdisp_timer.data     = (unsigned long)jiffies;
    shdisp_timer.function = shdisp_timer_int_isr;

    if (!shdisp_timer_stop) {
        up(&shdisp_sem_timer);
        return;
    }

    add_timer(&shdisp_timer);
    shdisp_timer_stop = 0;

    up(&shdisp_sem_timer);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_delete                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_timer_int_delete(void)
{
    down(&shdisp_sem_timer);

    del_timer_sync(&shdisp_timer);
    shdisp_timer_stop = 1;

    up(&shdisp_sem_timer);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_mod                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_timer_int_mod(void)
{
    down(&shdisp_sem_timer);

    if (shdisp_timer_stop) {
        up(&shdisp_sem_timer);
        return;
    }

    mod_timer(&shdisp_timer, (unsigned long)(jiffies + (10 * HZ)));
    shdisp_timer_stop = 0;

    up(&shdisp_sem_timer);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_timer_task                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_timer_task(struct work_struct *work)
{
    int     ret = 0;
    int     nFirst_GFAC = -1;
    void    (*temp_callback)(void);

    nFirst_GFAC = SHDISP_IRQ_TYPE_DET;

    shdisp_semaphore_start();
    ret = shdisp_panel_API_check_recovery();
    shdisp_semaphore_end(__func__);
    if (ret == SHDISP_RESULT_SUCCESS) {
        shdisp_timer_int_mod();
        return;
    }
    SHDISP_DEBUG("A recovery processing is required.");

    down(&shdisp_sem_callback);
    temp_callback = shdisp_callback_table[nFirst_GFAC];
    up(&shdisp_sem_callback);

    if (temp_callback != NULL) {
        (*temp_callback)();
    } else {
        SHDISP_DEBUG(" Callback is Nulle pointer(irq_type=%d)", nFirst_GFAC);
    }

    shdisp_timer_int_mod();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_det_mipi_err_ctrl                                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_det_mipi_err_ctrl(bool enable)
{
    SHDISP_TRACE("in enable=%d", enable);

    if (enable) {
#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_SAZABI)
        (void)shdisp_panel_API_set_irq(SHDISP_IRQ_ENABLE);
#elif defined(CONFIG_SHDISP_PANEL_ARIA) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
        (void)shdisp_panel_API_set_irq(SHDISP_IRQ_ENABLE);
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */
#ifdef SHDISP_DET_DSI_MIPI_ERROR
        (void)mdss_shdisp_dsi_mipi_err_ctrl(true);
#endif  /* SHDISP_DET_DSI_MIPI_ERROR */
#endif  /* CONFIG_SHDISP_PANEL */
    }else{
#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_SAZABI)
        (void)shdisp_panel_API_set_irq(SHDISP_IRQ_DISABLE);
#elif defined(CONFIG_SHDISP_PANEL_ARIA) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#ifdef SHDISP_DET_DSI_MIPI_ERROR
        (void)mdss_shdisp_dsi_mipi_err_ctrl(false);
#endif  /* SHDISP_DET_DSI_MIPI_ERROR */
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
        (void)shdisp_panel_API_set_irq(SHDISP_IRQ_DISABLE);
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */
#endif  /* CONFIG_SHDISP_PANEL */
    }

    SHDISP_TRACE("out");
    return;
}
#ifdef SHDISP_IR2E71Y8
/* ------------------------------------------------------------------------- */
/* shdisp_do_detin_recovery                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_do_detin_recovery(void)
{
    int ret;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_DEBUG("enable_irq for DET before");
    shdisp_semaphore_start();
    ret = shdisp_bdic_API_RECOVERY_check_restoration();
    shdisp_semaphore_end(__func__);
    if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("lcd det bdic");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_ESD_DETIN;
        shdisp_dbg_API_err_output(&err_code, 0);
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_ESD_DETIN);
#endif /* SHDISP_RESET_LOG */
        shdisp_do_lcd_det_recovery();
    } else {
        SHDISP_DEBUG("lcd det bdic detects the false");
        shdisp_bdic_API_IRQ_det_irq_ctrl(true);
    }
    SHDISP_DEBUG("enable_irq for DET after");
    shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_do_lcd_det_recovery                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_do_lcd_det_recovery(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int queued = 0;

    SHDISP_TRACE("in");

    if (!shdisp_wq_recovery) {
        SHDISP_ERR(" workqueue nothing.");
        return SHDISP_RESULT_FAILURE;
    }
    if (sh_boot_get_bootmode() == SH_BOOT_D || sh_boot_get_bootmode() == SH_BOOT_F_F) {
        SHDISP_DEBUG("Diag mode skip recovery");
        return ret;
    }
    shdisp_wake_lock();

    down(&shdisp_sem_req_recovery_lcd);

    if (!shdisp_recovery_lcd_queued_flag) {
        shdisp_recovery_lcd_queued_flag = 1;

        ret = queue_work(shdisp_wq_recovery, &shdisp_wq_recovery_lcd_wk);

        if (ret == 0) {
            shdisp_recovery_lcd_queued_flag = 0;
            SHDISP_ERR("<QUEUE_WORK_FAILURE> .");
            ret = SHDISP_RESULT_FAILURE;
        } else {
            queued = 1;
            ret = SHDISP_RESULT_SUCCESS;
        }
    } else {
        SHDISP_DEBUG("queued. now recovering... ");
        ret = SHDISP_RESULT_SUCCESS;
    }

    up(&shdisp_sem_req_recovery_lcd);

    if (!queued) {
        shdisp_wake_unlock();
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_recovery_lcd                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_recovery_lcd(struct work_struct *work)
{
    void (*temp_callback)(void);

    SHDISP_TRACE("in");

    down(&shdisp_sem_callback);
    temp_callback = shdisp_callback_table[SHDISP_IRQ_TYPE_DET];
    up(&shdisp_sem_callback);

    if (temp_callback != NULL) {
        (*temp_callback)();
    } else {
        shdisp_clear_recovery_lcd_queued();
    }

    shdisp_wake_unlock();

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_clear_recovery_lcd_queued                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_clear_recovery_lcd_queued(void)
{
        down(&shdisp_sem_req_recovery_lcd);
        shdisp_recovery_lcd_queued_flag = 0;
        up(&shdisp_sem_req_recovery_lcd);
}

#ifdef SHDISP_USE_QUALCOMM_RECOVERY
#ifndef SHDISP_NOT_SUPPORT_DET
/* ------------------------------------------------------------------------- */
/* mdss_lcd_det_recovery                                                     */
/* ------------------------------------------------------------------------- */
static void mdss_lcd_det_recovery(void)
{
    SHDISP_TRACE("in");
    mdss_shdisp_report_panel_dead();
    SHDISP_WARN("mdss recovery request. cnt=%d", mdss_recovery_retry_cnt);
    mdss_recovery_retry_cnt++;
    SHDISP_TRACE("out.");
}
#endif /* SHDISP_NOT_SUPPORT_DET */
#else /* SHDISP_USE_QUALCOMM_RECOVERY */
#ifndef SHDISP_NOT_SUPPORT_DET
/* ------------------------------------------------------------------------- */
/* shdisp_lcd_det_recovery                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_lcd_det_recovery(void)
{
    SHDISP_TRACE("in");

    shdisp_semaphore_start();
#ifdef SHDISP_IR2E71Y8
    if (shdisp_pm_API_is_active_panel_user(SHDISP_DEV_TYPE_TP) == SHDISP_DEV_STATE_ON) {
        shdisp_pending_det_recovery = true;
        shdisp_clear_recovery_lcd_queued();
        shdisp_semaphore_end(__func__);
        SHDISP_WARN("TP FW Loading... No recovery.");
        return;
    }
#endif /* SHDISP_IR2E71Y8 */
    shdisp_semaphore_end(__func__);

#ifdef CONFIG_TOUCHSCREEN_SHTPS
    SHDISP_PERFORMANCE("msm_tps_setsleep(off) s");
    msm_tps_setsleep(1);
    SHDISP_PERFORMANCE("msm_tps_setsleep(off) e");
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#ifdef CONFIG_TOUCHSCREEN_FT8607
    fts_setsleep(1);
#endif /* CONFIG_TOUCHSCREEN_FT8607 */

    mdss_shdisp_lock_recovery();

    shdisp_semaphore_start();

    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_OFF) {
        shdisp_clear_recovery_lcd_queued();
        shdisp_semaphore_end(__func__);
        mdss_shdisp_unlock_recovery();
        SHDISP_WARN("out1");
        return;
    }

#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    mdss_shdisp_mdp_hr_video_suspend();
#endif /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_HAYABUSA */
    shdisp_SQE_lcd_det_recovery();
#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    mdss_shdisp_mdp_hr_video_resume();
#endif /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_HAYABUSA */

    shdisp_semaphore_end(__func__);

    mdss_shdisp_unlock_recovery();

#ifdef CONFIG_TOUCHSCREEN_SHTPS
    SHDISP_PERFORMANCE("msm_tps_setsleep(on) s");
    msm_tps_setsleep(0);
    SHDISP_PERFORMANCE("msm_tps_setsleep(on) e");
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#ifdef CONFIG_TOUCHSCREEN_FT8607
    fts_setsleep(0);
#endif /* CONFIG_TOUCHSCREEN_FT8607 */

    SHDISP_TRACE("out");
    return;
}
#endif /* SHDISP_NOT_SUPPORT_DET */
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */

/* ------------------------------------------------------------------------- */
/* shdisp_lcd_det_recovery_subscribe                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_lcd_det_recovery_subscribe(void)
{
    int ret = 0;
#ifndef SHDISP_NOT_SUPPORT_DET
    struct shdisp_subscribe lcd_subs;
#endif /* SHDISP_NOT_SUPPORT_DET */

    SHDISP_TRACE("in");

#ifdef SHDISP_USE_QUALCOMM_RECOVERY
#ifndef SHDISP_NOT_SUPPORT_DET
    lcd_subs.irq_type = SHDISP_IRQ_TYPE_DET;
    lcd_subs.callback = mdss_lcd_det_recovery;
    shdisp_event_subscribe(&lcd_subs);
#endif /* SHDISP_NOT_SUPPORT_DET */
#else /* SHDISP_USE_QUALCOMM_RECOVERY */
#ifndef SHDISP_NOT_SUPPORT_DET
    if (sh_boot_get_bootmode() == SH_BOOT_D || sh_boot_get_bootmode() == SH_BOOT_F_F) {
#ifdef SHDISP_IR2E71Y8
        shdisp_bdic_API_IRQ_det_irq_ctrl(true);
#endif /* SHDISP_IR2E71Y8 */
    } else {
        lcd_subs.irq_type = SHDISP_IRQ_TYPE_DET;
        lcd_subs.callback = shdisp_lcd_det_recovery;
        shdisp_event_subscribe(&lcd_subs);
    }
#else  /* SHDISP_NOT_SUPPORT_DET */
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_IRQ_det_irq_ctrl(true);
#endif /* SHDISP_IR2E71Y8 */
#endif /* SHDISP_NOT_SUPPORT_DET */
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_lcd_det_recovery_unsubscribe                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_lcd_det_recovery_unsubscribe(void)
{
    int ret = 0;

    SHDISP_TRACE("in");

#ifndef SHDISP_NOT_SUPPORT_DET
    if (sh_boot_get_bootmode() == SH_BOOT_D || sh_boot_get_bootmode() == SH_BOOT_F_F) {
#ifdef SHDISP_IR2E71Y8
        shdisp_bdic_API_IRQ_det_irq_ctrl(false);
#endif /* SHDISP_IR2E71Y8 */
    } else {
        shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_DET);
    }
#else  /* SHDISP_NOT_SUPPORT_DET */
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_IRQ_det_irq_ctrl(false);
#endif /* SHDISP_IR2E71Y8 */
#endif /* SHDISP_NOT_SUPPORT_DET */

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}
#ifdef SHDISP_IR2E71Y8
#ifdef SHDISP_ALS_INT
/* ------------------------------------------------------------------------- */
/* shdisp_do_als_int_report                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_do_als_int_report(int ginf2_val)
{
    SHDISP_TRACE("in");

    if (ginf2_val != 0) {
        shdisp_input_subsystem_report(ginf2_val);
        shdisp_set_als_int_unsubscribe(ginf2_val);
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_do_als_int_report_dummy                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_do_als_int_report_dummy(void)
{
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_set_als_int_subscribe                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_set_als_int_subscribe(int trigger)
{
    int ret = 0;
    struct shdisp_subscribe alsint_subs;
    int before_flag = als_int_enable;

    SHDISP_TRACE("in");

    if (trigger == 0) {
        SHDISP_ERR("trigger zero");
        return SHDISP_RESULT_FAILURE;
    }

    als_int_enable = (als_int_enable | trigger) & (SHDISP_OPT_CHANGE_INT_1 | SHDISP_OPT_CHANGE_INT_2);
    if (before_flag != 0) {
        SHDISP_DEBUG("<caution>double subscribe");
        return SHDISP_RESULT_SUCCESS;
    }

    alsint_subs.irq_type = SHDISP_IRQ_TYPE_ALS;
    alsint_subs.callback = shdisp_do_als_int_report_dummy;
    ret = shdisp_event_subscribe(&alsint_subs);

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_set_als_int_unsubscribe                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_set_als_int_unsubscribe(int trigger)
{
    int ret = 0;

    SHDISP_TRACE("in");

    if (trigger == 0) {
        SHDISP_ERR("trigger zero");
        return SHDISP_RESULT_FAILURE;
    }

    if (als_int_enable == 0) {
        SHDISP_DEBUG("<caution>double unsubscribe");
        return SHDISP_RESULT_SUCCESS;
    }

    als_int_enable = (als_int_enable & (~trigger)) & (SHDISP_OPT_CHANGE_INT_1 | SHDISP_OPT_CHANGE_INT_2);
    if (als_int_enable == 0) {
        ret = shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_ALS);
    } else {
        SHDISP_DEBUG("<caution>double unsubscribe");
        return SHDISP_RESULT_SUCCESS;
    }

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}
#endif /* SHDISP_ALS_INT */

#ifdef SHDISP_LED_INT
/* ------------------------------------------------------------------------- */
/* shdisp_led_auto_low_isr                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_led_auto_low_isr(void)
{
    SHDISP_TRACE("in");

    shdisp_semaphore_start();

    if (!shdisp_kerl_ctx.led_auto_low_enable) {
        SHDISP_WARN("disabled.");
        shdisp_semaphore_end(__func__);
    }

    shdisp_bdic_API_led_auto_low_process();

    shdisp_semaphore_end(__func__);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_led_auto_low_subscribe                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_led_auto_low_subscribe(void)
{
    int ret;
    struct shdisp_subscribe subs;

    SHDISP_TRACE("in");

    subs.irq_type = SHDISP_IRQ_TYPE_ALS_REQ;
    subs.callback = shdisp_led_auto_low_isr;
    ret = shdisp_event_subscribe(&subs);

    SHDISP_TRACE("out ret=%d", ret);

   return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_led_auto_low_unsubscribe                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_led_auto_low_unsubscribe(void)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_ALS_REQ);

    SHDISP_TRACE("out ret=%d", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_led_auto_low_enable                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_led_auto_low_enable(void)
{
    int ret;

    SHDISP_TRACE("in");

    if (shdisp_kerl_ctx.led_auto_low_enable) {
        SHDISP_DEBUG("already enabled.");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_bdic_API_led_auto_low_enable(true);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("enable failure.");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_led_auto_low_subscribe();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("subscribe failure.");
        shdisp_bdic_API_led_auto_low_enable(false);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_kerl_ctx.led_auto_low_enable = true;

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_led_auto_low_disable                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_led_auto_low_disable(void)
{
    int ret;

    SHDISP_TRACE("in");

    if (!shdisp_kerl_ctx.led_auto_low_enable) {
        SHDISP_DEBUG("already disabled.");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_led_auto_low_unsubscribe();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("unsubscribe failure.");
    }

    ret = shdisp_bdic_API_led_auto_low_enable(false);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("disable failure.");
    }

    shdisp_kerl_ctx.led_auto_low_enable = false;

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_LED_INT */

/* ------------------------------------------------------------------------- */
/* shdisp_do_psals_recovery                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_do_psals_recovery(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    int queued = 0;
    SHDISP_TRACE("in");

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out3");
        return SHDISP_RESULT_SUCCESS;
    }

    if (!shdisp_wq_recovery) {
        SHDISP_ERR(" workqueue nothing.");
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_wake_lock();

    down(&shdisp_sem_req_recovery_psals);

    if (!shdisp_recovery_psals_queued_flag) {
        shdisp_recovery_psals_queued_flag = 1;
        ret = queue_work(shdisp_wq_recovery, &shdisp_wq_recovery_psals_wk);

        if (ret == 0) {
            shdisp_recovery_psals_queued_flag = 0;
            SHDISP_ERR("<QUEUE_WORK_FAILURE> .");
            ret = SHDISP_RESULT_FAILURE;
        } else {
            queued = 1;
            ret = SHDISP_RESULT_SUCCESS;
        }
    } else {
        SHDISP_DEBUG("queued. now recovering... ");
        ret = SHDISP_RESULT_SUCCESS;
    }
    up(&shdisp_sem_req_recovery_psals);

    if (!queued) {
        shdisp_wake_unlock();
    }

    SHDISP_TRACE("out");

    return ret;
}
#endif /* SHDISP_IR2E71Y8 */
/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_recovery_psals                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_recovery_psals(struct work_struct *work)
{
    void (*temp_callback)(void);

    SHDISP_TRACE("in");

    down(&shdisp_sem_callback);
    temp_callback = shdisp_callback_table[SHDISP_IRQ_TYPE_I2CERR];
    up(&shdisp_sem_callback);

    if (temp_callback != NULL) {
        (*temp_callback)();
    }

    shdisp_wake_unlock();

    SHDISP_TRACE("End");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_psals_recovery                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_psals_recovery(void)
{
    SHDISP_TRACE("in");

    shdisp_SQE_psals_recovery();

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_event_unsubscribe                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_event_unsubscribe(int irq_type)
{
    int ret;

    SHDISP_PERFORMANCE("RESUME PANEL EVENT-UNSUBSCRIBE 0010 START");

    if (shdisp_bdic_unsubscribe_check(irq_type) != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_SQE_event_unsubscribe(irq_type);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_event_unsubscribe.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL EVENT-UNSUBSCRIBE 0010 END");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_event_subscribe                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_event_subscribe(struct shdisp_subscribe *subscribe)
{
    int ret;

    SHDISP_PERFORMANCE("RESUME PANEL EVENT-SUBSCRIBE 0010 START");


    SHDISP_TRACE(":Start(irq_type=%d)", subscribe->irq_type);

    if (shdisp_bdic_subscribe_check(subscribe) != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    if (subscribe->irq_type == SHDISP_IRQ_TYPE_DET) {
        if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_SUCCESS;
        }
    }

    ret = shdisp_SQE_event_subscribe(subscribe);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_event_subscribe.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_PERFORMANCE("RESUME PANEL EVENT-SUBSCRIBE 0010 END");

    return SHDISP_RESULT_SUCCESS;

}

#ifdef SHDISP_SYSFS_LED
#ifdef SHDISP_IR2E71Y8
/* ------------------------------------------------------------------------- */
/* shdisp_clear_sysfs_led                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_clean_sysfs_led(void) {

       shdisp_kerl_ctx.sysfs_led1.red = 0;
       shdisp_kerl_ctx.sysfs_led1.green = 0;
       shdisp_kerl_ctx.sysfs_led1.blue = 0;
       shdisp_kerl_ctx.sysfs_led1.led_mode = SHDISP_TRI_LED_MODE_OFF;
#ifdef SHDISP_COLOR_LED_TWIN
       shdisp_kerl_ctx.sysfs_led2.red = 0;
       shdisp_kerl_ctx.sysfs_led2.green = 0;
       shdisp_kerl_ctx.sysfs_led2.blue = 0;
       shdisp_kerl_ctx.sysfs_led2.led_mode = SHDISP_TRI_LED_MODE_OFF;
#endif /* SHDISP_COLOR_LED_TWIN */

}

/* ------------------------------------------------------------------------- */
/* shdisp_clear_normal_led                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_clean_normal_led(void) {

      shdisp_kerl_ctx.tri_led.red = 0;
      shdisp_kerl_ctx.tri_led.green = 0;
      shdisp_kerl_ctx.tri_led.blue = 0;
      shdisp_kerl_ctx.tri_led.led_mode = SHDISP_TRI_LED_MODE_NORMAL;
}
#endif /* SHDISP_IR2E71Y8 */
#endif /* SHDISP_SYSFS_LED */

#ifdef SHDISP_FPS_LED_PANEL_SUPPORT
/* ------------------------------------------------------------------------- */
/* shdisp_fps_led_read_reg                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_fps_led_read_reg(unsigned char *fps_reg)
{
    int ret;
    char page;

    page = PAGE_FPS_REG;
    ret = shdisp_panel_API_mipi_diag_write_reg(
            SHDISP_DTYPE_DCS_WRITE1,
            ADDR_PAGE,
            &page,
            sizeof(page));
    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_panel_API_mipi_diag_read_reg(
            SHDISP_DTYPE_DCS_READ,
            ADDR_FPS_REG,
            fps_reg,
            sizeof(*fps_reg));
    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_fps_led_set_color                                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_fps_led_set_color(int state)
{
#ifdef SHDISP_IR2E71Y8
    struct shdisp_tri_led param;

    param.red   = shdisp_fps_led_color[state][0];
    param.green = shdisp_fps_led_color[state][1];
    param.blue  = shdisp_fps_led_color[state][2];
    param.ext_mode = SHDISP_TRI_LED_EXT_MODE_DISABLE;
    param.led_mode = SHDISP_TRI_LED_MODE_NORMAL;

    shdisp_tri_led_set_color(&param);
#else /* SHDISP_IR2E71Y8 */
    struct shled_tri_led led;

    led.red = shdisp_fps_led_color[state][0];
    led.green = shdisp_fps_led_color[state][1];
    led.blue = shdisp_fps_led_color[state][2];

    shled_api_set_brightness(&led);
#endif /* SHDISP_IR2E71Y8 */
}

/* ------------------------------------------------------------------------- */
/* shdisp_fps_led_work                                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_fps_led_work(struct work_struct *work)
{
    int ret;
    int bit_cnt;
    int state, bef_state;
    unsigned char fps_reg = 0;

    SHDISP_TRACE("in");

    if (!shdisp_fps_led_ctx.enable || shdisp_fps_led_ctx.suspend) {
        SHDISP_DEBUG("out2 enable=%d suspend=%d",
                shdisp_fps_led_ctx.enable,
                shdisp_fps_led_ctx.suspend);
        return;
    }

    queue_delayed_work(
            shdisp_fps_led_ctx.workq,
            &shdisp_fps_led_ctx.work,
            usecs_to_jiffies(shdisp_fps_led_ctx.interval));

    state = shdisp_fps_led_ctx.state;
    bef_state = state;

    shdisp_semaphore_start();
    if (shdisp_fps_led_ctx.panel_on) {
        ret = shdisp_fps_led_read_reg(&fps_reg);
        if (ret == SHDISP_RESULT_SUCCESS) {
            bit_cnt = hweight8(fps_reg);
#ifdef SHDISP_HAYABUSA_HF
            if (bit_cnt >= 8) {
                state = FPS_LED_STATE_120HZ;
            } else if (bit_cnt >= 4) {
                state = FPS_LED_STATE_60HZ;
            } else if (bit_cnt >= 2) {
                state = FPS_LED_STATE_30HZ;
            } else if (bit_cnt >= 1) {
                state = FPS_LED_STATE_15HZ;
            } else {
                state = FPS_LED_STATE_1HZ;
            }
#else /* SHDISP_HAYABUSA_HF */
            if (bit_cnt >= 8) {
                state = FPS_LED_STATE_60HZ;
            } else if (bit_cnt >= 4) {
                state = FPS_LED_STATE_30HZ;
            } else if (bit_cnt >= 2) {
                state = FPS_LED_STATE_15HZ;
            } else if (bit_cnt >= 1) {
                state = FPS_LED_STATE_1HZ;
            } else {
                state = FPS_LED_STATE_1HZ;
            }
#endif /* SHDISP_HAYABUSA_HF */
            SHDISP_DEBUG("fps_reg=%02x state=%s", fps_reg, shdisp_fps_led_state_str[state]);
        }
    } else {
        state = FPS_LED_STATE_NONE;
    }
    shdisp_semaphore_end(__func__);

    if (state != bef_state) {
        shdisp_semaphore_start();
        shdisp_fps_led_set_color(state);
        shdisp_semaphore_end(__func__);
    }

    shdisp_fps_led_ctx.state = state;

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_fps_led_start                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_fps_led_start(void)
{
    SHDISP_TRACE("in");

    if (shdisp_fps_led_ctx.enable) {
        SHDISP_DEBUG("out2 enable=%d",
                shdisp_fps_led_ctx.enable);
        return;
    }

    if (!shdisp_fps_led_ctx.workq) {
        SHDISP_ERR("workq is NULL.");
        return;
    }

    shdisp_fps_led_ctx.enable = true;
    shdisp_fps_led_ctx.state = FPS_LED_STATE_NONE;
    shdisp_fps_led_ctx.interval = FPS_LED_INTERVAL;
#ifdef SHDISP_IR2E71Y8
#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
    if (shdisp_bdic_API_LED_is_running_illumi_triple_color()) {
        shdisp_bdic_API_LED_clear_illumi_triple_color();
    }
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
#endif /* SHDISP_IR2E71Y8 */
    shdisp_semaphore_start();
    shdisp_fps_led_set_color(FPS_LED_STATE_NONE);
    shdisp_semaphore_end(__func__);

    if (shdisp_fps_led_ctx.panel_on) {
        shdisp_fps_led_ctx.suspend = false;
        queue_delayed_work(
                shdisp_fps_led_ctx.workq,
                &shdisp_fps_led_ctx.work,
                usecs_to_jiffies(shdisp_fps_led_ctx.interval));
    } else {
        shdisp_fps_led_ctx.suspend = true;
    }

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_fps_led_stop                                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_fps_led_stop(void)
{
    SHDISP_TRACE("in");

    if (!shdisp_fps_led_ctx.enable) {
        SHDISP_DEBUG("out2 enable=%d",
                shdisp_fps_led_ctx.enable);
        return;
    }

    if (!shdisp_fps_led_ctx.workq) {
        SHDISP_ERR("workq is NULL.");
        return;
    }

    shdisp_fps_led_ctx.enable = false;
    cancel_delayed_work_sync(&shdisp_fps_led_ctx.work);

    shdisp_semaphore_start();
    shdisp_fps_led_set_color(FPS_LED_STATE_NONE);
    shdisp_semaphore_end(__func__);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_fps_led_resume                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_fps_led_resume(void)
{
    SHDISP_TRACE("in");

    if (!shdisp_fps_led_ctx.enable || !shdisp_fps_led_ctx.suspend) {
        SHDISP_DEBUG("out2 enable=%d suspend=%d",
                shdisp_fps_led_ctx.enable,
                shdisp_fps_led_ctx.suspend);
        return;
    }

    if (!shdisp_fps_led_ctx.workq) {
        SHDISP_ERR("workq is NULL.");
        return;
    }

    shdisp_fps_led_ctx.suspend = false;
    queue_delayed_work(
            shdisp_fps_led_ctx.workq,
            &shdisp_fps_led_ctx.work,
            usecs_to_jiffies(shdisp_fps_led_ctx.interval * 3));

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_fps_led_suspend                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_fps_led_suspend(void)
{
    SHDISP_TRACE("in");

    if (!shdisp_fps_led_ctx.enable || shdisp_fps_led_ctx.suspend) {
        SHDISP_DEBUG("out2 enable=%d suspend=%d",
                shdisp_fps_led_ctx.enable,
                shdisp_fps_led_ctx.suspend);
        return;
    }

    if (!shdisp_fps_led_ctx.workq) {
        SHDISP_ERR("workq is NULL.");
        return;
    }

    shdisp_fps_led_ctx.suspend = true;
    cancel_delayed_work_sync(&shdisp_fps_led_ctx.work);
    shdisp_fps_led_ctx.state = FPS_LED_STATE_NONE;

    shdisp_semaphore_start();
    shdisp_fps_led_set_color(FPS_LED_STATE_NONE);
    shdisp_semaphore_end(__func__);

    SHDISP_TRACE("out");
}
#endif /* SHDISP_FPS_LED_PANEL_SUPPORT */

/* ------------------------------------------------------------------------- */
/* OTHER                                                                     */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_proc_write                                                         */
/* ------------------------------------------------------------------------- */
#define PROC_BUF_LENGTH                 (4096)
#define PROC_BUF_REWIND_LENGTH          (4000)
#define SHDISP_DEBUG_CONSOLE(fmt, args...) \
        do { \
            int buflen = 0; \
            int remain = PROC_BUF_LENGTH - proc_buf_pos - 1; \
            if (remain > 0) { \
                buflen = snprintf(&proc_buf[proc_buf_pos], remain, fmt, ## args); \
                proc_buf_pos += (buflen > 0) ? buflen : 0; \
            } \
        } while (0)
static unsigned char proc_buf[PROC_BUF_LENGTH] = {0};
static unsigned int  proc_buf_pos = 0;

static int shdisp_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
#define SHDISP_LEN_ID                   (2)
#define SHDISP_LEN_PARAM                (4)
#define SHDISP_PARAM_MAX                (4)

    unsigned long len = count;
    struct shdisp_procfs shdisp_pfs;
    char buf[SHDISP_LEN_PARAM + 1];
    char kbuf[SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM];
    int i;
    int ret = 0;
#ifdef SHDISP_IR2E71Y8
    struct shdisp_bdic_i2c_msg i2c_msg;
    unsigned char i2c_wbuf[6];
    unsigned char i2c_rbuf[6];
    struct shdisp_prox_params prox_params;
    struct shdisp_main_bkl_ctl bkl;
    unsigned char   val;
#endif /* SHDISP_IR2E71Y8 */
    char *kbufex;
    unsigned char *param = NULL;
    int paramlen = 0;
    int needalloc = 0;
#ifdef SHDISP_IR2E71Y8
#ifdef SHDISP_KEY_LED
    struct shdisp_key_bkl_ctl key_bkl;
#endif /* SHDISP_KEY_LED */
    struct shdisp_tri_led tri_led;
#endif /* SHDISP_IR2E71Y8 */
    int recovery_error_flag;
    int recovery_error_count;

    len--;
    /* Check length */
    if (len < SHDISP_LEN_ID) {
        return count;
    }
    if (len > (SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM)) {
        len = SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM;
        needalloc = 1;
    }

    if (copy_from_user(kbuf, buffer, len)) {
        return -EFAULT;
    }
    /* Get FunctionID */
    memcpy(buf, kbuf, SHDISP_LEN_ID);
    buf[SHDISP_LEN_ID] = '\0';
    shdisp_pfs.id = simple_strtol(buf, NULL, 10);
    shdisp_pfs.par[0] = 0;
    shdisp_pfs.par[1] = 0;
    shdisp_pfs.par[2] = 0;
    shdisp_pfs.par[3] = 0;

    /* Get Parameters */
    for (i = 0; (i + 1) * SHDISP_LEN_PARAM <= (len - SHDISP_LEN_ID); i++) {
        memcpy(buf, &(kbuf[SHDISP_LEN_ID + i * SHDISP_LEN_PARAM]), SHDISP_LEN_PARAM);
        buf[SHDISP_LEN_PARAM] = '\0';
        shdisp_pfs.par[i] = simple_strtol(buf, NULL, 16);
    }

    printk("[SHDISP] shdisp_proc_write(%d, 0x%04x, 0x%04x, 0x%04x, 0x%04x)\n", shdisp_pfs.id,
                                                                               shdisp_pfs.par[0], shdisp_pfs.par[1],
                                                                               shdisp_pfs.par[2], shdisp_pfs.par[3]);

    switch (shdisp_pfs.id) {
    case SHDISP_DEBUG_DSI_DCS_WRITE:
    case SHDISP_DEBUG_DSI_GEN_WRITE:
        if (len < 8) {
            SHDISP_ERR("(%d): DSI_WRITE param error", shdisp_pfs.id);
            goto out;
        }
        needalloc = 1;
        break;
    }

    if (needalloc) {
        len = count - (SHDISP_LEN_ID + 1);
        if (len > (1024 * SHDISP_PARAM_MAX) - (SHDISP_LEN_ID + 1)) {
           len = (1024 * SHDISP_PARAM_MAX) - (SHDISP_LEN_ID + 1);
        }
        kbufex = kmalloc(len, GFP_KERNEL);
        if (!kbufex) {
            return -EFAULT;
        }
        buffer += SHDISP_LEN_ID;
        if (copy_from_user(kbufex, buffer, len)) {
            kfree(kbufex);
            return -EFAULT;
        }
        paramlen = len / (SHDISP_LEN_PARAM / 2);
        param = kmalloc(paramlen, GFP_KERNEL);
        if (!param) {
            kfree(kbufex);
            return -EFAULT;
        }
        /* Get Parameters */
        for (i = 0; i < paramlen; i++) {
            memcpy(buf, &(kbufex[i * (SHDISP_LEN_PARAM / 2)]), (SHDISP_LEN_PARAM / 2));
            buf[(SHDISP_LEN_PARAM / 2)] = '\0';
            param[i] = simple_strtol(buf, NULL, 16);
        }
        kfree(kbufex);
    }

    switch (shdisp_pfs.id) {
    case SHDISP_DEBUG_PROCESS_STATE_OUTPUT:
        shdisp_semaphore_start();
        shdisp_dbg_info_output((int)shdisp_pfs.par[0]);
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_TRACE_LOG_SWITCH:
        if (shdisp_pfs.par[0] == 0) {
            printk("[SHDISP] Trace log OFF\n");
        } else {
            printk("[SHDISP] Trace log ON(%d)\n", shdisp_pfs.par[0]);
        }
        SHDISP_SET_LOG_LV((unsigned short)shdisp_pfs.par[0]);
        SHDISP_DEBUG("TraceLog enable check!!");
        break;
#ifdef SHDISP_IR2E71Y8
    case SHDISP_DEBUG_BDIC_I2C_WRITE:
        printk("[SHDISP] BDIC-I2C WRITE (addr : 0x%02x, data : 0x%02x)\n", shdisp_pfs.par[0], shdisp_pfs.par[1]);
        i2c_wbuf[0] = shdisp_pfs.par[0];
        i2c_wbuf[1] = shdisp_pfs.par[1];

        i2c_msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
        i2c_msg.mode = SHDISP_BDIC_I2C_M_W;
        i2c_msg.wlen = 2;
        i2c_msg.rlen = 0;
        i2c_msg.wbuf = &i2c_wbuf[0];
        i2c_msg.rbuf = NULL;
        shdisp_semaphore_start();
        ret = shdisp_SQE_write_bdic_i2c(&i2c_msg);
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_BDIC_I2C_READ:
        printk("[SHDISP] BDIC-I2C READ (addr : 0x%02x)\n", shdisp_pfs.par[0]);
        i2c_wbuf[0] = shdisp_pfs.par[0];
        i2c_rbuf[0] = 0x00;

        i2c_msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
        i2c_msg.mode = SHDISP_BDIC_I2C_M_R;
        i2c_msg.wlen = 1;
        i2c_msg.rlen = 1;
        i2c_msg.wbuf = &i2c_wbuf[0];
        i2c_msg.rbuf = &i2c_rbuf[0];
        shdisp_semaphore_start();
        ret = shdisp_SQE_read_bdic_i2c(&i2c_msg);
        shdisp_semaphore_end(__func__);
        printk("[SHDISP] Read data(0x%02x)\n", i2c_rbuf[0]);
        SHDISP_DEBUG_CONSOLE("<COMMAND = I2C_READ>\n");
        SHDISP_DEBUG_CONSOLE("  IN     = 0x%02x\n", i2c_wbuf[0]);
        SHDISP_DEBUG_CONSOLE("  OUT    = 0x%02x\n", i2c_rbuf[0]);
        if (ret == SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG_CONSOLE("  RESULT = OK\n");
        } else {
            SHDISP_DEBUG_CONSOLE("  RESULT = NG\n");
        }
        break;

    case SHDISP_DEBUG_PROX_SENSOR_CTL:
        shdisp_semaphore_start();
        switch (shdisp_pfs.par[0]) {
        case 0:
            ret = shdisp_SQE_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_OFF);
            break;
        case 1:
            printk("[SHDISP] POWER_ON_PARAM (LOW : %d, HIGH : %d)\n", shdisp_pfs.par[1], shdisp_pfs.par[2]);
            prox_params.threshold_low  = (unsigned int)shdisp_pfs.par[1];
            prox_params.threshold_high = (unsigned int)shdisp_pfs.par[2];

            ret = shdisp_SQE_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON);
            break;
        default:
            break;
        }
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_BKL_CTL:
        shdisp_semaphore_start();
        if (shdisp_pfs.par[0] == 0) {
            printk("[SHDISP] BKL_OFF\n");
            bkl.mode  = SHDISP_MAIN_BKL_MODE_OFF;
            bkl.param = SHDISP_MAIN_BKL_PARAM_OFF;
            ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP, &bkl);
        } else {
            printk("[SHDISP] BKL_ON (mode : %d, param : %d)\n", shdisp_pfs.par[1], shdisp_pfs.par[2]);
            bkl.mode  = shdisp_pfs.par[1];
            bkl.param = shdisp_pfs.par[2];
            if (bkl.mode == SHDISP_MAIN_BKL_MODE_AUTO) {
                ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO, &bkl);
            } else {
                ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP, &bkl);
            }
        }
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_LIGHT_SENSOR_CTL:
        shdisp_semaphore_start();
        switch (shdisp_pfs.par[0]) {
        case 0:
            shdisp_pm_API_als_user_manager(
                    SHDISP_DEV_TYPE_ALS_MASK & (unsigned long)shdisp_pfs.par[1],
                    SHDISP_DEV_REQ_OFF);
            break;
        case 1:
            shdisp_pm_API_als_user_manager(
                    SHDISP_DEV_TYPE_ALS_MASK & (unsigned long)shdisp_pfs.par[1],
                    SHDISP_DEV_REQ_ON);
            break;
        default:
            break;
        }
        shdisp_semaphore_end(__func__);
        break;
#endif /* SHDISP_IR2E71Y8 */
    case SHDISP_DEBUG_IRQ_LOGIC_CHK:
        i = shdisp_pfs.par[0];
        printk("[SHDISP] shdisp_proc_write(%d):Test Pattern=%d\n", shdisp_pfs.id, i);
        shdisp_dbg_que(i);
        break;
#ifdef SHDISP_IR2E71Y8
    case SHDISP_DEBUG_BDIC_IRQ_ALL_CLEAR:
        printk("[SHDISP] shdisp_proc_write(%d):Interrupt Clear All\n", shdisp_pfs.id);
        shdisp_semaphore_start();
        shdisp_bdic_API_IRQ_dbg_Clear_All();
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_BDIC_IRQ_CLEAR:
        printk("[SHDISP] shdisp_proc_write(%d):Interrupt Clear\n", shdisp_pfs.id);
        shdisp_semaphore_start();
        shdisp_bdic_API_IRQ_Clear();
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_DUMMY_SUBSCRIBE:
        printk("[SHDISP] shdisp_proc_write(%d):dummy subscribe\n", shdisp_pfs.id);
        shdisp_semaphore_start();
        shdisp_debug_subscribe();
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_DUMMY_UNSUBSCRIBE_PS:
        printk("[SHDISP] shdisp_proc_write(%d):dummy unsubscribe\n", shdisp_pfs.id);
        shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_PS);
        break;

    case SHDISP_DEBUG_BDIC_WRITE:
        printk("[SHDISP] shdisp_proc_write(%d): BDIC register write\n", shdisp_pfs.id);
        val = shdisp_pfs.par[0] & 0x00FF;
        printk("[SHDISP] shdisp_SQE_bdic_write_reg() reg=0x%02x val=0x%02x\n", ((shdisp_pfs.par[0] >> 8) & 0x00FF),
                                                                               val);
        shdisp_semaphore_start();
        shdisp_SQE_bdic_write_reg(((shdisp_pfs.par[0] >> 8) & 0x00FF), val);
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_BDIC_READ:
        printk("[SHDISP] shdisp_proc_write(%d): BDIC register read\n", shdisp_pfs.id);
        val = 0;
        printk("[SHDISP] shdisp_SQE_bdic_read_reg() reg=0x%02x\n", ((shdisp_pfs.par[0] >> 8) & 0x00FF));
        shdisp_semaphore_start();
        ret = shdisp_SQE_bdic_read_reg(((shdisp_pfs.par[0] >> 8) & 0x00FF), &val);
        shdisp_semaphore_end(__func__);
        printk("[SHDISP] value=0x%02x\n", val);
        SHDISP_DEBUG_CONSOLE("<COMMAND = BDIC_register_READ>\n");
        SHDISP_DEBUG_CONSOLE("  IN     = 0x%02x\n", ((shdisp_pfs.par[0] >> 8) & 0x00FF));
        SHDISP_DEBUG_CONSOLE("  OUT    = 0x%02x\n", val);
        if (ret == SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG_CONSOLE("  RESULT = OK\n");
        } else {
            SHDISP_DEBUG_CONSOLE("  RESULT = NG\n");
        }
        break;

#ifdef SHDISP_IR2E71Y8
#ifdef SHDISP_KEY_LED
    case SHDISP_DEBUG_KEY_BKL_LED:
        key_bkl.key_left = ((shdisp_pfs.par[0] >> 8) & 0x00FF);
        key_bkl.key_center = ( shdisp_pfs.par[0]       & 0x00FF);
        key_bkl.key_right = ((shdisp_pfs.par[1] >> 8) & 0x00FF);
        key_bkl.ontime   = ( shdisp_pfs.par[1]       & 0x00FF);
        key_bkl.interval = ((shdisp_pfs.par[2] >> 8) & 0x00FF);
        shdisp_semaphore_start();
        ret = shdisp_SQE_key_bkl_ctl(&key_bkl);
        shdisp_semaphore_end(__func__);
        break;
#endif /* SHDISP_KEY_LED */
#endif /* SHDISP_IR2E71Y8 */

    case SHDISP_DEBUG_RGB_LED:
#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
        if (shdisp_bdic_API_LED_is_running_illumi_triple_color()) {
            shdisp_bdic_API_LED_clear_illumi_triple_color();
        }
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
        tri_led.red      = ((shdisp_pfs.par[0] >> 8) & 0x00FF);
        tri_led.green    = ( shdisp_pfs.par[0]       & 0x00FF);
        tri_led.blue     = ((shdisp_pfs.par[1] >> 8) & 0x00FF);
        tri_led.ext_mode = ( shdisp_pfs.par[1]       & 0x00FF);
        tri_led.led_mode = ((shdisp_pfs.par[2] >> 8) & 0x00FF);
        tri_led.ontime   = ( shdisp_pfs.par[2]       & 0x00FF);
        tri_led.interval = ((shdisp_pfs.par[3] >> 8) & 0x00FF);
        tri_led.count    = ( shdisp_pfs.par[3]       & 0x00FF);
        shdisp_semaphore_start();
        ret = shdisp_SQE_tri_led_set_color(&tri_led);
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_LED_REG_DUMP:
        shdisp_semaphore_start();
        if (shdisp_pfs.par[0] == 1) {
            shdisp_bdic_API_TRI_LED2_INFO_output();
        } else {
            shdisp_bdic_API_TRI_LED_INFO_output();
        }
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_BDIC_RESTART:
        shdisp_semaphore_start();
        shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_I2CERR);
        shdisp_event_unsubscribe(SHDISP_IRQ_TYPE_DET);
        shdisp_pm_API_bdic_shutdown();
        shdisp_bdic_API_boot_init(NULL);
        shdisp_bdic_API_boot_init_2nd();
        shdisp_pm_API_bdic_resume();

        if (shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.status == SHDISP_ALS_SENSOR_ADJUST_STATUS_COMPLETED) {
            shdisp_bdic_API_als_sensor_adjust(&(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj));
        }
        shdisp_semaphore_end(__func__);
        break;

#if defined(PMI_PIERCE_LED)
    case SHDISP_DEBUG_PIERCE_INOUT:
        if (shdisp_pfs.par[0] == 0) {
            printk("[SHDISP] Smart phone pierce Removed.\n");
            ret = leds_api_remove_sp_pierce();
        } else if (shdisp_pfs.par[0] == 1) {
            printk("[SHDISP] Smart phone pierce Inserted.\n");
            ret = leds_api_insert_sp_pierce();
        }
        break;
#endif /* PMI_PIERCE_LED */

    case SHDISP_DEBUG_LCD_POWER_CHG:
        if (shdisp_pfs.par[0] >= 1 && shdisp_pfs.par[0] <= 4){
            shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_OFF;
        }
        shdisp_semaphore_start();
        switch (shdisp_pfs.par[0]) {
        case 1:
            printk("[SHDISP_DEBUG][%s] lcd_power_on(NORMAL) \n", __func__);
            first_lcd_on = true;
            shdisp_SQE_main_lcd_power_on();
            break;
        case 2:
            printk("[SHDISP_DEBUG][%s] lcd_power_on(FIRST) \n", __func__);
            first_lcd_on = false;
            shdisp_SQE_main_lcd_power_on();
            break;
        case 3:
            printk("[SHDISP_DEBUG][%s] lcd_power_off(NORMAL) \n", __func__);
            shdisp_SQE_main_lcd_power_off();
            break;
        case 4:
            printk("[SHDISP_DEBUG][%s] lcd_power_off(SHUTDOWN) \n", __func__);
            shdisp_kerl_ctx.shutdown_in_progress = true;
            shdisp_SQE_main_lcd_power_off();
            shdisp_kerl_ctx.shutdown_in_progress = false;
            break;
        case 5:
            printk("[SHDISP_DEBUG][%s] lcd_power_on(TP) \n", __func__);
            shdisp_SQE_panel_pow_ctl(SHDISP_PANEL_POWER_ON);
            break;

        case 6:
            printk("[SHDISP_DEBUG][%s] lcd_power_off(TP) \n", __func__);
            shdisp_SQE_panel_pow_ctl(SHDISP_PANEL_POWER_OFF);
            break;
        default:
            printk("[SHDISP_DEBUG][%s] SHDISP_DEBUG_LCD_POWER_CHG unknown param \n", __func__);
            break;
        }
        shdisp_semaphore_end(__func__);
        break;
#endif /* SHDISP_IR2E71Y8 */
    case SHDISP_DEBUG_MIPI_TX_FREQ_CHG:

        break;

    case SHDISP_DEBUG_FPS_LED:
#ifdef SHDISP_FPS_LED_PANEL_SUPPORT
        if (shdisp_pfs.par[0]) {
            shdisp_fps_led_start();
        } else {
            shdisp_fps_led_stop();
        }
#elif defined(SHDISP_FPS_LED_HOST_SUPPORT)
        if (shdisp_pfs.par[0]) {
            mdss_shdisp_fps_led_start();
        } else {
            mdss_shdisp_fps_led_stop();
        }
#endif /* SHDISP_FPS_LED_PANEL_SUPPORT */
        break;

    case SHDISP_DEBUG_DISPLAYLOG_ERROR_LOG_TEST:
    {
        struct shdisp_dbg_error_code    code;

        code.mode      = (unsigned char)shdisp_pfs.par[0];
        code.type      = (unsigned char)shdisp_pfs.par[1];
        code.code      = (unsigned char)shdisp_pfs.par[2];
        code.subcode   = (unsigned char)shdisp_pfs.par[3];
        shdisp_dbg_API_add_err_log(&code);
        break;
    }
    case SHDISP_DEBUG_DISPLAYLOG_SUMMARY_TEST:
    {
        struct shdisp_dbg_error_code    code;

        code.mode      = (unsigned char)shdisp_pfs.par[0];
        code.type      = (unsigned char)shdisp_pfs.par[1];
        code.code      = (unsigned char)shdisp_pfs.par[2];
        code.subcode   = (unsigned char)shdisp_pfs.par[3];
        shdisp_dbg_API_err_countup(&code);
        break;
    }
#ifdef SHDISP_IR2E71Y8
#ifndef SHDISP_NOT_SUPPORT_BKL_CHG_MODE
    case SHDISP_DEBUG_CHARGE_BLK_MODE:
        shdisp_semaphore_start();
        shdisp_SQE_main_bkl_set_chg_mode(shdisp_pfs.par[0]);
        shdisp_semaphore_end(__func__);
        break;
#endif  /* SHDISP_NOT_SUPPORT_BKL_CHG_MODE */

#ifndef SHDISP_NOT_SUPPORT_BKL_EMG_MODE
    case SHDISP_DEBUG_EMG_BLK_MODE:
        shdisp_semaphore_start();
        shdisp_SQE_main_bkl_set_emg_mode(shdisp_pfs.par[0]);
        shdisp_semaphore_end(__func__);
        break;
#endif /* SHDISP_NOT_SUPPORT_BKL_EMG_MODE */

#ifdef SHDISP_LOWBKL
    case SHDISP_DEBUG_LOW_BLK_MODE:
        shdisp_semaphore_start();
        shdisp_SQE_main_bkl_set_lowbkl_mode(shdisp_pfs.par[0]);
        shdisp_semaphore_end(__func__);
        break;
#endif /* SHDISP_LOWBKL */
#endif /* SHDISP_IR2E71Y8 */
    case SHDISP_DEBUG_RECOVERY_NG:
        if (shdisp_pfs.par[0] == 1) {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_DISPON;
            printk("[SHDISP] set recovery check error disp on (reg)\n");
        } else if (shdisp_pfs.par[0] == 2) {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_DETLOW;
            printk("[SHDISP] set recovery check error det low\n");
#ifdef SHDISP_IR2E71Y8
        } else if (shdisp_pfs.par[0] == 3) {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_PSALS;
            printk("[SHDISP] set recovery check error psals\n");
#endif /* SHDISP_IR2E71Y8 */
        } else if (shdisp_pfs.par[0] == 4) {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_DISPON_READ;
            printk("[SHDISP] set recovery check error disp on (read)\n");
#ifdef SHDISP_IR2E71Y8
        } else if (shdisp_pfs.par[0] == 5) {
            recovery_error_flag = SHDISP_DBG_BDIC_ERROR_DCDC1;
            printk("[SHDISP] set bdic dcdc1 error\n");
#endif /* SHDISP_IR2E71Y8 */
        } else {
            recovery_error_flag = SHDISP_DBG_RECOVERY_ERROR_NONE;
            printk("[SHDISP] set recovery check error none\n");
        }

        recovery_error_count = shdisp_pfs.par[1];
        printk("[SHDISP] set recovery check error retry count=%d\n", recovery_error_count);

        shdisp_dbg_API_set_recovery_check_error(recovery_error_flag, recovery_error_count);
        break;
#ifdef SHDISP_IR2E71Y8
#ifdef SHDISP_ALS_INT
    case SHDISP_DEBUG_ALS_REPORT:
        val = shdisp_pfs.par[0];
        SHDISP_DEBUG("value    : %X", val);
        ret = shdisp_input_subsystem_report(val);
        SHDISP_DEBUG("ret    : %d\n", ret);
        break;
#endif /* SHDISP_ALS_INT */
#endif /* SHDISP_IR2E71Y8 */
    case SHDISP_DEBUG_DSI_GEN_WRITE:
    case SHDISP_DEBUG_DSI_DCS_WRITE:
    {
        struct shdisp_dsi_cmd_req dsi_req;
        unsigned char buf[SHDISP_LCDDR_BUF_MAX];

        memset(&dsi_req, 0x00, sizeof(dsi_req));
        memset(buf, 0x00, sizeof(buf));
        dsi_req.data = buf;

        if (shdisp_pfs.id == SHDISP_DEBUG_DSI_GEN_WRITE) {
            dsi_req.dtype = SHDISP_DTYPE_GEN_WRITE;
        } else {
            dsi_req.dtype = SHDISP_DTYPE_DCS_WRITE;
        }
        dsi_req.size = param[0];
        dsi_req.addr = param[1];
        dsi_req.mode = param[2];

        SHDISP_DEBUG(" Size    : %2d", dsi_req.size);
        SHDISP_DEBUG(" Address : %02Xh", dsi_req.addr);
        SHDISP_DEBUG(" Mode    : %02Xh", dsi_req.mode);
        for (i = 0; i < dsi_req.size; i++) {
            dsi_req.data[i] = param[i + 3];
            if ((i % 8) == 0) {
                printk("[SHDISP_DEBUG][%s]  WData    : ", __func__);
            }
            printk("%02X ", dsi_req.data[i]);
            if ((i % 8) == 7) {
                printk("\n");
            }
        }
        printk("\n");

        shdisp_semaphore_start();
        ret = shdisp_panel_API_dsi_write_reg(&dsi_req);
        shdisp_semaphore_end(__func__);
        break;
    }

    case SHDISP_DEBUG_DSI_GEN_READ:
    case SHDISP_DEBUG_DSI_DCS_READ:
    {
        struct shdisp_dsi_cmd_req dsi_req;
        unsigned char buf[SHDISP_LCDDR_BUF_MAX];

        memset(&dsi_req, 0x00, sizeof(dsi_req));
        memset(buf, 0x00, sizeof(buf));
        dsi_req.data = buf;

        if (shdisp_pfs.id == SHDISP_DEBUG_DSI_GEN_READ) {
            dsi_req.dtype = SHDISP_DTYPE_GEN_READ;
        } else {
            dsi_req.dtype = SHDISP_DTYPE_DCS_READ;
        }
        dsi_req.size    = ((shdisp_pfs.par[0] >> 8) & 0x00FF);
        dsi_req.addr    = ( shdisp_pfs.par[0]       & 0x00FF);
        dsi_req.mode    = ((shdisp_pfs.par[1] >> 8) & 0x00FF);

        SHDISP_DEBUG("PANEL INFO ->>");
        SHDISP_DEBUG(" Size    : %2d", dsi_req.size);
        SHDISP_DEBUG(" Address : %02Xh", dsi_req.addr);
        SHDISP_DEBUG(" Mode    : %02Xh", dsi_req.mode);
        SHDISP_DEBUG_CONSOLE("<COMMAND = DSI_READ>\n");
        SHDISP_DEBUG_CONSOLE("  IN     : Address=0x%02X Size=%d\n", dsi_req.addr, dsi_req.size);

        shdisp_semaphore_start();
        ret = shdisp_panel_API_dsi_read_reg(&dsi_req);
        shdisp_semaphore_end(__func__);

        for (i = 0; i < dsi_req.size; i++) {
            if ((i % 16) == 0) {
                printk("[SHDISP_DEBUG][%s]  RData    :", __func__);
                SHDISP_DEBUG_CONSOLE("  OUT    : RData[%2d]=", i);
            }
            printk(" %02X", dsi_req.data[i]);
            SHDISP_DEBUG_CONSOLE(" %02X", dsi_req.data[i]);
            if ((i % 16) == 15 || i == (dsi_req.size - 1)) {
                printk("\n");
                SHDISP_DEBUG_CONSOLE("\n");
            }
        }

        if (ret == SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG_CONSOLE("  RESULT = OK\n");
        } else {
            SHDISP_DEBUG_CONSOLE("  RESULT = NG\n");
        }

        break;
    }

    case SHDISP_DEBUG_SET_IRQ:
        shdisp_semaphore_start();
        if (shdisp_pfs.par[0] == 0) {
            shdisp_det_mipi_err_ctrl(false);
#ifdef SHDISP_IR2E71Y8
            shdisp_bdic_API_IRQ_det_irq_ctrl(false);
#endif /* SHDISP_IR2E71Y8 */
        } else {
            shdisp_det_mipi_err_ctrl(true);
#ifdef SHDISP_IR2E71Y8
            shdisp_bdic_API_IRQ_det_irq_ctrl(true);
#endif /* SHDISP_IR2E71Y8 */
        }
        shdisp_semaphore_end(__func__);
        break;

    case SHDISP_DEBUG_DO_RECOVERY:
        shdisp_do_lcd_det_recovery();
        break;

    default:
        break;
    }

    printk("[SHDISP] result : %d.\n", ret);

    if (needalloc) {
        kfree(param);
    }

out:

    return count;
}

/* ------------------------------------------------------------------------- */
/* shdisp_proc_read                                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_proc_read(char *page, char **start, off_t offset, int count, int *eof, void *data)
{
    int len = 0;

    len += snprintf(page, count, "%s", proc_buf);
    proc_buf[0] = 0;
    proc_buf_pos = 0;

    return len;
}

#define PROC_BLOCK_SIZE	(PAGE_SIZE - 1024)
/* ------------------------------------------------------------------------- */
/* shdisp_proc_file_read                                                     */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_proc_file_read(struct file *file, char __user *buf, size_t nbytes, loff_t *ppos)
{
    char    *page;
    ssize_t retval=0;
    int eof=0;
    ssize_t n, count;
    char    *start;
    unsigned long long pos;

    /*
     * Gaah, please just use "seq_file" instead. The legacy /proc
     * interfaces cut loff_t down to off_t for reads, and ignore
     * the offset entirely for writes..
     */
    pos = *ppos;
    if (pos > MAX_NON_LFS) {
        return 0;
    }
    if (nbytes > (MAX_NON_LFS - pos)) {
        nbytes = MAX_NON_LFS - pos;
    }

    if (!(page = (char*) __get_free_page(GFP_TEMPORARY))) {
        return -ENOMEM;
    }

    while ((nbytes > 0) && !eof) {
        count = min_t(size_t, PROC_BLOCK_SIZE, nbytes);

        start = NULL;
        n = shdisp_proc_read(page, &start, *ppos,
                  count, &eof, NULL);

        if (n == 0) {    /* end of file */
            break;
        }
        if (n < 0) {  /* error */
            if (retval == 0)
                retval = n;
            break;
        }

        if (start == NULL) {
            if (n > PAGE_SIZE) {
                printk(KERN_ERR
                       "proc_file_read: Apparent buffer overflow!\n");
                n = PAGE_SIZE;
            }
            n -= *ppos;
            if (n <= 0)
                break;
            if (n > count)
                n = count;
            start = page + *ppos;
        } else if (start < page) {
            if (n > PAGE_SIZE) {
                printk(KERN_ERR
                       "proc_file_read: Apparent buffer overflow!\n");
                n = PAGE_SIZE;
            }
            if (n > count) {
                /*
                 * Don't reduce n because doing so might
                 * cut off part of a data block.
                 */
                printk(KERN_WARNING
                       "proc_file_read: Read count exceeded\n");
            }
        } else /* start >= page */ {
            unsigned long startoff = (unsigned long)(start - page);
            if (n > (PAGE_SIZE - startoff)) {
                printk(KERN_ERR
                       "proc_file_read: Apparent buffer overflow!\n");
                n = PAGE_SIZE - startoff;
            }
            if (n > count) {
                n = count;
            }
        }

        n -= copy_to_user(buf, start < page ? page : start, n);
        if (n == 0) {
            if (retval == 0) {
                retval = -EFAULT;
            }
            break;
        }

        *ppos += start < page ? (unsigned long)start : n;
        nbytes -= n;
        buf += n;
        retval += n;
    }
    free_page((unsigned long) page);

    return retval;
}

/* ------------------------------------------------------------------------- */
/* shdisp_proc_file_write                                                    */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_proc_file_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    ssize_t rv = -EIO;

    rv = shdisp_proc_write(file, buffer, count, NULL);

    return rv;
}
#endif /* CONFIG_ANDROID_ENGINEERING */


#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_info_output                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_info_output(int mode)
{
    int i;

    switch (mode) {
    case SHDISP_DEBUG_INFO_TYPE_BOOT:
        printk("[SHDISP] BOOT INFO ->>\n");
        printk("[SHDISP] sizeof(shdisp_kerl_ctx.boot_ctx)                = %ld.\n", (long)sizeof(shdisp_kerl_ctx.boot_ctx));
        printk("[SHDISP] boot_ctx.driver_is_initialized         = %d.\n", shdisp_kerl_ctx.boot_ctx.driver_is_initialized);
        printk("[SHDISP] boot_ctx.hw_handset                    = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.hw_handset);
        printk("[SHDISP] boot_ctx.hw_revision                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.hw_revision);
        printk("[SHDISP] boot_ctx.device_code                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.device_code);
        printk("[SHDISP] boot_ctx.disp_on_status                = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.disp_on_status);
        printk("[SHDISP] boot_ctx.handset_color                 = %d.\n", shdisp_kerl_ctx.boot_ctx.handset_color);
        printk("[SHDISP] boot_ctx.upper_unit_is_connected       = %d.\n", shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected);
        printk("[SHDISP] boot_ctx.main_disp_status              = %d.\n", shdisp_kerl_ctx.boot_ctx.main_disp_status);
        printk("[SHDISP] boot_ctx.is_trickled                   = %d.\n", shdisp_kerl_ctx.boot_ctx.is_trickled);
        printk("[SHDISP] boot_ctx.main_bkl.mode                 = %d.\n", shdisp_kerl_ctx.boot_ctx.main_bkl.mode);
        printk("[SHDISP] boot_ctx.main_bkl.param                = %d.\n", shdisp_kerl_ctx.boot_ctx.main_bkl.param);
        printk("[SHDISP] boot_ctx.tri_led.red                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.red);
        printk("[SHDISP] boot_ctx.tri_led.green                 = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.green);
        printk("[SHDISP] boot_ctx.tri_led.blue                  = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.blue);
        printk("[SHDISP] boot_ctx.tri_led.ext_mode              = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.ext_mode);
        printk("[SHDISP] boot_ctx.tri_led.led_mode              = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.led_mode);
        printk("[SHDISP] boot_ctx.tri_led.ontime                = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.ontime);
        printk("[SHDISP] boot_ctx.tri_led.interval              = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.interval);
        printk("[SHDISP] boot_ctx.tri_led.count                 = %d.\n", shdisp_kerl_ctx.boot_ctx.tri_led.count);
        printk("[SHDISP] boot_ctx.vcom                         = 0x%04X.\n", (unsigned int)shdisp_kerl_ctx.boot_ctx.vcom);
        printk("[SHDISP] boot_ctx.vcom_low                     = 0x%04X.\n", (unsigned int)shdisp_kerl_ctx.boot_ctx.vcom_low);
        printk("[SHDISP] boot_ctx.vcom_nvram                   = 0x%04X.\n",
                 (unsigned int)shdisp_kerl_ctx.boot_ctx.vcom_nvram);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.status       = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.status);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_adj0);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_adj1);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_shift);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].clear_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].ir_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_adj0);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_adj1);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_shift);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].clear_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].ir_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.key_backlight    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.key_backlight);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.chksum       = 0x%06X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.chksum);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_is_exist     = %d.\n", shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_is_exist);
        printk("[SHDISP] boot_ctx.ledc_status.power_status      = %d.\n", shdisp_kerl_ctx.boot_ctx.ledc_status.power_status);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.red      = %p.\n",
                shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.red);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.green    = %p.\n",
                shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.green);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.blue     = %p.\n",
                shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.blue);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.led_mode = %d.\n",
                shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.led_mode);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.on_count = %d.\n",
                shdisp_kerl_ctx.boot_ctx.ledc_status.ledc_req.on_count);
        printk("[SHDISP] boot_ctx.lcddr_phy_gmm.status        = 0x%02X.\n", shdisp_kerl_ctx.boot_ctx.lcddr_phy_gmm.status);
        printk("[SHDISP] boot_ctx.lcddr_phy_gmm.buf           = ");
        for (i = 0; i < SHDISP_LCDDR_PHY_GMM_BUF_MAX; i++) {
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.lcddr_phy_gmm.buf[i]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.lcddr_phy_gmm.applied_voltage = ");
        for (i = 0; i < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; i++) {
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.lcddr_phy_gmm.applied_voltage[i]);
        }
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
        printk("\n");
        printk("[SHDISP] boot_ctx.lcddr_phy_gmm.adv_gmm     = ");
        for (i = 0; i < SHDISP_LCDDR_ADVANCED_GAMMA_SIZE; i++) {
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.lcddr_phy_gmm.adv_gmm[i]);
        }
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */
        printk("\n");
        printk("[SHDISP] boot_ctx.lcddr_phy_gmm.chksum        = 0x%04X.\n", shdisp_kerl_ctx.boot_ctx.lcddr_phy_gmm.chksum);

        printk("[SHDISP] boot_ctx.lut_status                    = 0x%04X.\n", shdisp_kerl_ctx.boot_ctx.lut_status);
        printk("[SHDISP] boot_ctx.argc_lut.red                  = ");
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.red[i][0]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.red[i][1]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.red[i][2]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.argc_lut.green                = ");
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.green[i][0]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.green[i][1]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.green[i][2]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.argc_lut.blue                 = ");
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.blue[i][0]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.blue[i][1]);
        }
        printk("\n    ");
        for (i = 0; i < SHDISP_ARGC_STAGE_NUM; i++) {
            printk("%04X,", shdisp_kerl_ctx.boot_ctx.argc_lut.blue[i][2]);
        }
        printk("\n");

        printk("[SHDISP] boot_ctx.igc_lut.r_data                = ");
        for (i = 0; i < SHDISP_IGC_LUT_ENTRIES; i++) {
            if (i % 16 == 0) {
                printk("\n    ");
            }
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.igc_lut.r_data[i]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.igc_lut.g_data                = ");
        for (i = 0; i < SHDISP_IGC_LUT_ENTRIES; i++) {
            if (i % 16 == 0) {
                printk("\n    ");
            }
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.igc_lut.g_data[i]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.igc_lut.b_data                = ");
        for (i = 0; i < SHDISP_IGC_LUT_ENTRIES; i++) {
            if (i % 16 == 0) {
                printk("\n    ");
            }
            printk("%02X,", shdisp_kerl_ctx.boot_ctx.igc_lut.b_data[i]);
        }
        printk("\n");

        printk("[SHDISP] boot_ctx.bdic_is_exist                 = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_is_exist);
        printk("[SHDISP] boot_ctx.bdic_chipver                  = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_chipver);
        printk("[SHDISP] boot_ctx.bdic_status.power_status      = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_status.power_status);
        printk("[SHDISP] boot_ctx.bdic_status.users             = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.bdic_status.users);
        printk("[SHDISP] boot_ctx.psals_status.power_status     = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.power_status);
        printk("[SHDISP] boot_ctx.psals_status.als_users        = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.psals_status.als_users);
        printk("[SHDISP] boot_ctx.psals_status.ps_um_status     = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.ps_um_status);
        printk("[SHDISP] boot_ctx.psals_status.als_um_status    = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.als_um_status);

        printk("\n");
        for (i = 0; i < SHDISP_NOOS_RESET_NUM; i++) {
            printk("[SHDISP] boot_ctx.err_on[%d]                = %d.\n", i, (int)shdisp_kerl_ctx.boot_ctx.err_on[i]);
            printk("[SHDISP] boot_ctx.err_code[%d].mode         = %d.\n", i, (int)shdisp_kerl_ctx.boot_ctx.err_code[i].mode);
            printk("[SHDISP] boot_ctx.err_code[%d].type         = %d.\n", i, (int)shdisp_kerl_ctx.boot_ctx.err_code[i].type);
            printk("[SHDISP] boot_ctx.err_code[%d].code         = %d.\n", i, (int)shdisp_kerl_ctx.boot_ctx.err_code[i].code);
            printk("[SHDISP] boot_ctx.err_code[%d].subcode      = %d.\n", i, (int)shdisp_kerl_ctx.boot_ctx.err_code[i].subcode);
        }
        printk("[SHDISP] BOOT INFO <<-\n");
        break;
    case SHDISP_DEBUG_INFO_TYPE_KERNEL:
        printk("[SHDISP] KERNEL INFO ->>\n");
        printk("[SHDISP] sizeof(shdisp_kerl_ctx)                = %ld.\n", (long)sizeof(shdisp_kerl_ctx));
        printk("[SHDISP] kerl_ctx.driver_is_open                = %d.\n", shdisp_kerl_ctx.driver_is_open);
        printk("[SHDISP] kerl_ctx.driver_open_cnt               = %d.\n", shdisp_kerl_ctx.driver_open_cnt);
        printk("[SHDISP] kerl_ctx.driver_is_initialized         = %d.\n", shdisp_kerl_ctx.driver_is_initialized);
        printk("[SHDISP] kerl_ctx.thermal_status                = %d.\n", shdisp_kerl_ctx.thermal_status);
        printk("[SHDISP] kerl_ctx.usb_chg_status                = %d.\n", shdisp_kerl_ctx.usb_chg_status);
        printk("[SHDISP] kerl_ctx.main_disp_status              = %d.\n", shdisp_kerl_ctx.main_disp_status);
        printk("[SHDISP] kerl_ctx.main_bkl.mode                 = %d.\n", shdisp_kerl_ctx.main_bkl.mode);
        printk("[SHDISP] kerl_ctx.main_bkl.param                = %d.\n", shdisp_kerl_ctx.main_bkl.param);
        printk("[SHDISP] kerl_ctx.tri_led.red                   = %d.\n", (int)shdisp_kerl_ctx.tri_led.red);
        printk("[SHDISP] kerl_ctx.tri_led.green                 = %d.\n", (int)shdisp_kerl_ctx.tri_led.green);
        printk("[SHDISP] kerl_ctx.tri_led.blue                  = %d.\n", (int)shdisp_kerl_ctx.tri_led.blue);
        printk("[SHDISP] kerl_ctx.tri_led.ext_mode              = %d.\n", shdisp_kerl_ctx.tri_led.ext_mode);
        printk("[SHDISP] kerl_ctx.tri_led.led_mode              = %d.\n", shdisp_kerl_ctx.tri_led.led_mode);
        printk("[SHDISP] kerl_ctx.tri_led.ontime                = %d.\n", shdisp_kerl_ctx.tri_led.ontime);
        printk("[SHDISP] kerl_ctx.tri_led.interval              = %d.\n", shdisp_kerl_ctx.tri_led.interval);
        printk("[SHDISP] kerl_ctx.tri_led.count                 = %d.\n", shdisp_kerl_ctx.tri_led.count);
#ifdef SHDISP_SYSFS_LED
        printk("[SHDISP] kerl_ctx.sysfs_led1.red                = %d.\n", (int)shdisp_kerl_ctx.sysfs_led1.red);
        printk("[SHDISP] kerl_ctx.sysfs_led1.green              = %d.\n", (int)shdisp_kerl_ctx.sysfs_led1.green);
        printk("[SHDISP] kerl_ctx.sysfs_led1.blue               = %d.\n", (int)shdisp_kerl_ctx.sysfs_led1.blue);
        printk("[SHDISP] kerl_ctx.sysfs_led1.ext_mode           = %d.\n", shdisp_kerl_ctx.sysfs_led1.ext_mode);
        printk("[SHDISP] kerl_ctx.sysfs_led1.led_mode           = %d.\n", shdisp_kerl_ctx.sysfs_led1.led_mode);
        printk("[SHDISP] kerl_ctx.sysfs_led1.ontime             = %d.\n", shdisp_kerl_ctx.sysfs_led1.ontime);
        printk("[SHDISP] kerl_ctx.sysfs_led1.interval           = %d.\n", shdisp_kerl_ctx.sysfs_led1.interval);
        printk("[SHDISP] kerl_ctx.sysfs_led1.count              = %d.\n", shdisp_kerl_ctx.sysfs_led1.count);
#ifdef SHDISP_COLOR_LED_TWIN
        printk("[SHDISP] kerl_ctx.sysfs_led2.red                = %d.\n", (int)shdisp_kerl_ctx.sysfs_led2.red);
        printk("[SHDISP] kerl_ctx.sysfs_led2.green              = %d.\n", (int)shdisp_kerl_ctx.sysfs_led2.green);
        printk("[SHDISP] kerl_ctx.sysfs_led2.blue               = %d.\n", (int)shdisp_kerl_ctx.sysfs_led2.blue);
        printk("[SHDISP] kerl_ctx.sysfs_led2.ext_mode           = %d.\n", shdisp_kerl_ctx.sysfs_led2.ext_mode);
        printk("[SHDISP] kerl_ctx.sysfs_led2.led_mode           = %d.\n", shdisp_kerl_ctx.sysfs_led2.led_mode);
        printk("[SHDISP] kerl_ctx.sysfs_led2.ontime             = %d.\n", shdisp_kerl_ctx.sysfs_led2.ontime);
        printk("[SHDISP] kerl_ctx.sysfs_led2.interval           = %d.\n", shdisp_kerl_ctx.sysfs_led2.interval);
        printk("[SHDISP] kerl_ctx.sysfs_led2.count              = %d.\n", shdisp_kerl_ctx.sysfs_led2.count);
#endif /* SHDISP_COLOR_LED_TWIN */
#endif /* SHDISP_SYSFS_LED */
#ifdef SHDISP_LED_INT
        printk("[SHDISP] kerl_ctx.led_auto_low_enable           = %d.\n", shdisp_kerl_ctx.led_auto_low_enable);
#endif /* SHDISP_LED_INT */
        printk("[SHDISP] kerl_ctx.led_set_color_reject          = %d.\n", shdisp_kerl_ctx.led_set_color_reject);
        printk("\n");

        for (i = 0; i < NUM_SHDISP_IRQ_TYPE ; i++) {
            if (shdisp_callback_table[i] != NULL) {
                printk("[SHDISP] shdisp_callback_table[%d]              = subscribed.\n", i);
            } else {
                printk("[SHDISP] shdisp_callback_table[%d]              = no subscribed.\n", i);
            }
        }

        printk("[SHDISP] kerl_ctx.shutdown_in_progress          = %d.\n", shdisp_kerl_ctx.shutdown_in_progress);

        printk("[SHDISP] KERNEL INFO <<-\n");
        break;

    case SHDISP_DEBUG_INFO_TYPE_POWERON:
        printk("[SHDISP] BOOT INFO ->>\n");
        printk("[SHDISP] sizeof(shdisp_kerl_ctx.boot_ctx)                = %ld.\n", (long)sizeof(shdisp_kerl_ctx.boot_ctx));
        printk("[SHDISP] boot_ctx.driver_is_initialized         = %d.\n", shdisp_kerl_ctx.boot_ctx.driver_is_initialized);
        printk("[SHDISP] boot_ctx.hw_handset                    = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.hw_handset);
        printk("[SHDISP] boot_ctx.hw_revision                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.hw_revision);
        printk("[SHDISP] boot_ctx.device_code                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.device_code);
        printk("[SHDISP] boot_ctx.disp_on_status                = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.disp_on_status);
        printk("[SHDISP] boot_ctx.handset_color                 = %d.\n", shdisp_kerl_ctx.boot_ctx.handset_color);
        printk("[SHDISP] boot_ctx.upper_unit_is_connected       = %d.\n", shdisp_kerl_ctx.boot_ctx.upper_unit_is_connected);
        printk("[SHDISP] boot_ctx.main_disp_status              = %d.\n", shdisp_kerl_ctx.boot_ctx.main_disp_status);
        printk("[SHDISP] boot_ctx.is_trickled                   = %d.\n", shdisp_kerl_ctx.boot_ctx.is_trickled);
        printk("[SHDISP] boot_ctx.main_bkl.mode                 = %d.\n", shdisp_kerl_ctx.boot_ctx.main_bkl.mode);
        printk("[SHDISP] boot_ctx.main_bkl.param                = %d.\n", shdisp_kerl_ctx.boot_ctx.main_bkl.param);
        printk("[SHDISP] boot_ctx.tri_led.red                   = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.red);
        printk("[SHDISP] boot_ctx.tri_led.green                 = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.green);
        printk("[SHDISP] boot_ctx.tri_led.blue                  = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.tri_led.blue);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.status       = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.status);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_adj0);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_adj1);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].als_shift);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].clear_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[0].ir_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_adj0);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_adj1);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].als_shift);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].clear_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.als_adjust[1].ir_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.chksum       = 0x%06X.\n",
                (unsigned int)shdisp_kerl_ctx.boot_ctx.photo_sensor_adj.chksum);
        printk("[SHDISP] boot_ctx.bdic_is_exist                 = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_is_exist);
        printk("[SHDISP] boot_ctx.bdic_chipver                  = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_chipver);
        printk("[SHDISP] boot_ctx.bdic_status.power_status      = %d.\n", shdisp_kerl_ctx.boot_ctx.bdic_status.power_status);
        printk("[SHDISP] boot_ctx.bdic_status.users             = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.bdic_status.users);
        printk("[SHDISP] boot_ctx.psals_status.power_status     = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.power_status);
        printk("[SHDISP] boot_ctx.psals_status.als_users        = %d.\n", (int)shdisp_kerl_ctx.boot_ctx.psals_status.als_users);
        printk("[SHDISP] boot_ctx.psals_status.ps_um_status     = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.ps_um_status);
        printk("[SHDISP] boot_ctx.psals_status.als_um_status    = %d.\n", shdisp_kerl_ctx.boot_ctx.psals_status.als_um_status);
        printk("[SHDISP] BOOT INFO <<-\n");
        printk("[SHDISP] KERNEL INFO ->>\n");
        printk("[SHDISP] sizeof(shdisp_kerl_ctx)                = %ld.\n", (long)sizeof(shdisp_kerl_ctx));
        printk("[SHDISP] kerl_ctx.driver_is_open                = %d.\n", shdisp_kerl_ctx.driver_is_open);
        printk("[SHDISP] kerl_ctx.driver_open_cnt               = %d.\n", shdisp_kerl_ctx.driver_open_cnt);
        printk("[SHDISP] kerl_ctx.driver_is_initialized         = %d.\n", shdisp_kerl_ctx.driver_is_initialized);
        printk("[SHDISP] kerl_ctx.thermal_status                = %d.\n", shdisp_kerl_ctx.thermal_status);
        printk("[SHDISP] kerl_ctx.usb_chg_status                = %d.\n", shdisp_kerl_ctx.usb_chg_status);
        printk("[SHDISP] kerl_ctx.main_disp_status              = %d.\n", shdisp_kerl_ctx.main_disp_status);
        printk("[SHDISP] kerl_ctx.main_bkl.mode                 = %d.\n", shdisp_kerl_ctx.main_bkl.mode);
        printk("[SHDISP] kerl_ctx.main_bkl.param                = %d.\n", shdisp_kerl_ctx.main_bkl.param);
        printk("[SHDISP] kerl_ctx.tri_led.red                   = %d.\n", (int)shdisp_kerl_ctx.tri_led.red);
        printk("[SHDISP] kerl_ctx.tri_led.green                 = %d.\n", (int)shdisp_kerl_ctx.tri_led.green);
        printk("[SHDISP] kerl_ctx.tri_led.blue                  = %d.\n", (int)shdisp_kerl_ctx.tri_led.blue);
        printk("[SHDISP] kerl_ctx.tri_led.ext_mode              = %d.\n", shdisp_kerl_ctx.tri_led.ext_mode);
        printk("[SHDISP] kerl_ctx.tri_led.led_mode              = %d.\n", shdisp_kerl_ctx.tri_led.led_mode);
        printk("[SHDISP] kerl_ctx.tri_led.ontime                = %d.\n", shdisp_kerl_ctx.tri_led.ontime);
        printk("[SHDISP] kerl_ctx.tri_led.interval              = %d.\n", shdisp_kerl_ctx.tri_led.interval);
        printk("[SHDISP] kerl_ctx.tri_led.count                 = %d.\n", shdisp_kerl_ctx.tri_led.count);
        printk("[SHDISP] KERNEL INFO <<-\n");
        break;
#ifdef SHDISP_IR2E71Y8
    case SHDISP_DEBUG_INFO_TYPE_BDIC:
        shdisp_bdic_API_DBG_INFO_output();
        break;
    case SHDISP_DEBUG_INFO_TYPE_SENSOR:
        shdisp_bdic_API_PSALS_INFO_output();
        break;
#endif /* SHDISP_IR2E71Y8 */
    case SHDISP_DEBUG_INFO_TYPE_PANEL:
        shdisp_panel_API_dump(0);
        break;
#ifdef SHDISP_IR2E71Y8
    case SHDISP_DEBUG_INFO_TYPE_PM:
        shdisp_pm_API_power_manager_users_dump();
        break;
    case SHDISP_DEBUG_INFO_TYPE_BDIC_OPT:
        shdisp_bdic_API_OPT_INFO_output();
        break;
#endif /* SHDISP_IR2E71Y8 */
    default:
        break;
    }

    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_que                                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_dbg_que(int kind)
{
    unsigned int nRcvGFAC = 0;
    struct shdisp_queue_data_t *qdata = NULL;
    int    i;
    int    bFirstQue = 0;
    int    ret;
    int    nBDIC_QueFac = 0;


    SHDISP_TRACE(": Start");

    switch (kind) {
    case 1:
        nRcvGFAC = 0x00000000;
        break;
    case 2:
        nRcvGFAC = 0x00200000;
        break;
    case 3:
        nRcvGFAC = 0x00000100;
        break;
    case 4:
        nRcvGFAC = 0x00200100;
        break;
    case 5:
        nRcvGFAC = 0x00000008;
        break;
    case 6:
        nRcvGFAC = 0x00200008;
        break;
    case 7:
        nRcvGFAC = 0x00000108;
        break;
    case 8:
        nRcvGFAC = 0x00200108;
        break;
    case 9:
        nRcvGFAC = 0x00080000;
        break;
    case 10:
        nRcvGFAC = 0x00280000;
        break;
    case 11:
        nRcvGFAC = 0x00080100;
        break;
    case 12:
        nRcvGFAC = 0x00280100;
        break;
    case 13:
        nRcvGFAC = 0x00080008;
        break;
    case 14:
        nRcvGFAC = 0x00280008;
        break;
    case 15:
        nRcvGFAC = 0x00080108;
        break;
    case 16:
        nRcvGFAC = 0x00280108;
        break;
    case 17:
        nRcvGFAC = 0x00040000;
        break;
    case 18:
        nRcvGFAC = 0x00240000;
        break;
    case 19:
        nRcvGFAC = 0x00040100;
        break;
    case 20:
        nRcvGFAC = 0x00240100;
        break;
    case 21:
        nRcvGFAC = 0x00040008;
        break;
    case 22:
        nRcvGFAC = 0x00240008;
        break;
    case 23:
        nRcvGFAC = 0x00040108;
        break;
    case 24:
        nRcvGFAC = 0x00240108;
        break;
    case 25:
        nRcvGFAC = 0x000C0000;
        break;
    case 26:
        nRcvGFAC = 0x002C0000;
        break;
    case 27:
        nRcvGFAC = 0x000C0100;
        break;
    case 28:
        nRcvGFAC = 0x002C0100;
        break;
    case 29:
        nRcvGFAC = 0x000C0008;
        break;
    case 30:
        nRcvGFAC = 0x002C0008;
        break;
    case 31:
        nRcvGFAC = 0x000C0108;
        break;
    case 32:
        nRcvGFAC = 0x002C0108;
        break;
    case 33:
        nRcvGFAC = 0x00000200;
        break;
    case 34:
        nRcvGFAC = 0x00080200;
        break;
    case 35:
        nRcvGFAC = 0x00200200;
        break;
    case 36:
        nRcvGFAC = 0x00280200;
        break;
    case 37:
        nRcvGFAC = 0x00000300;
        break;
    case 38:
        nRcvGFAC = 0x00080300;
        break;
    case 39:
        nRcvGFAC = 0x00200300;
        break;
    case 40:
        nRcvGFAC = 0x00280300;
        break;
    case 41:
        nRcvGFAC = 0x00000208;
        break;
    case 42:
        nRcvGFAC = 0x00080208;
        break;
    case 43:
        nRcvGFAC = 0x00200208;
        break;
    case 44:
        nRcvGFAC = 0x00280208;
        break;
    case 45:
        nRcvGFAC = 0x00000308;
        break;
    case 46:
        nRcvGFAC = 0x00080308;
        break;
    case 47:
        nRcvGFAC = 0x00200308;
        break;
    case 48:
        nRcvGFAC = 0x00280308;
        break;
    case 49:
        nRcvGFAC = 0x00040200;
        break;
    case 50:
        nRcvGFAC = 0x000C0200;
        break;
    case 51:
        nRcvGFAC = 0x00240200;
        break;
    case 52:
        nRcvGFAC = 0x002C0200;
        break;
    case 53:
        nRcvGFAC = 0x00040300;
        break;
    case 54:
        nRcvGFAC = 0x000C0300;
        break;
    case 55:
        nRcvGFAC = 0x00240300;
        break;
    case 56:
        nRcvGFAC = 0x002C0300;
        break;
    case 57:
        nRcvGFAC = 0x00040208;
        break;
    case 58:
        nRcvGFAC = 0x000C0208;
        break;
    case 59:
        nRcvGFAC = 0x00240208;
        break;
    case 60:
        nRcvGFAC = 0x002C0208;
        break;
    case 61:
        nRcvGFAC = 0x00040308;
        break;
    case 62:
        nRcvGFAC = 0x000C0308;
        break;
    case 63:
        nRcvGFAC = 0x00240308;
        break;
    case 64:
        nRcvGFAC = 0x002C0308;
        break;

    default:
        nRcvGFAC = 0;
        break;
    }

    shdisp_SYS_API_set_irq(SHDISP_IRQ_DISABLE);
    shdisp_wake_lock();

#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_IRQ_dbg_set_fac(nRcvGFAC);
#endif /* SHDISP_IR2E71Y8 */
    do {
#ifdef SHDISP_IR2E71Y8
        shdisp_semaphore_start();
        ret = shdisp_bdic_API_IRQ_check_fac();
        shdisp_semaphore_end(__func__);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG(": no factory");
            break;
        }
#endif /* SHDISP_IR2E71Y8 */
        down(&shdisp_sem_irq_fac);
        for (i = 0; i < SHDISP_IRQ_MAX_KIND; i++) {
            shdisp_semaphore_start();
#ifdef SHDISP_IR2E71Y8
            nBDIC_QueFac = shdisp_bdic_API_IRQ_get_fac(i);
            shdisp_semaphore_end(__func__);
            if (nBDIC_QueFac == SHDISP_BDIC_IRQ_TYPE_NONE) {
                break;
            }
#endif /* SHDISP_IR2E71Y8 */
            if (shdisp_wq_gpio_task) {
                qdata = kmalloc(sizeof(shdisp_queue_data), GFP_KERNEL);
                if (qdata != NULL) {
                    qdata->irq_GFAC = nBDIC_QueFac;
                    list_add_tail(&qdata->list, &shdisp_queue_data.list);
                    if (bFirstQue == 0) {
                        bFirstQue = 1;
                        shdisp_wake_lock();
                        ret = queue_work(shdisp_wq_gpio_task, &shdisp_wq_gpio_task_wk);
                        if (ret == 0) {
                            shdisp_wake_unlock();
                            SHDISP_ERR("<QUEUE_WORK_FAILURE> ");
                        }
                    }
                } else {
                   SHDISP_ERR("<QUEUE_WORK_FAILURE> :kmalloc failed (BDIC_QueFac=%d)", nBDIC_QueFac);
                }
            }
        }
        up(&shdisp_sem_irq_fac);

    } while (0);

    shdisp_semaphore_start();
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_IRQ_Clear();
#endif /* SHDISP_IR2E71Y8 */
    shdisp_semaphore_end(__func__);
#ifdef SHDISP_IR2E71Y8
    if (shdisp_bdic_API_IRQ_check_DET() != SHDISP_BDIC_IRQ_TYPE_DET) {
        shdisp_SYS_API_set_irq(SHDISP_IRQ_ENABLE);
    }
#endif /* SHDISP_IR2E71Y8 */
    SHDISP_TRACE(": Finish");
    shdisp_wake_unlock();
}
#ifdef SHDISP_IR2E71Y8
/* ------------------------------------------------------------------------- */
/* shdisp_debug_subscribe                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_debug_subscribe(void)
{
    struct shdisp_subscribe dbg_subs;

    dbg_subs.irq_type = SHDISP_IRQ_TYPE_PS;
    dbg_subs.callback = callback_ps;
    shdisp_event_subscribe(&dbg_subs);
}

/* ------------------------------------------------------------------------- */
/* callback_ps                                                               */
/* ------------------------------------------------------------------------- */
static void callback_ps(void)
{
    printk("[SHDISP] callback_ps Start\n");
    shdisp_IO_API_msleep(1000);
    printk("[SHDISP] callback_ps Finish\n");
}
#endif /* SHDISP_IR2E71Y8 */
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_fb_open                                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_fb_open(void)
{
    struct fb_info *info = NULL;

    if (!num_registered_fb) {
        return;
    }
    info = registered_fb[0];
    if (!info) {
        return;
    }
    if (!try_module_get(info->fbops->owner)) {
        return;
    }
    if (info->fbops->fb_open && info->fbops->fb_open(info, 0)) {
        module_put(info->fbops->owner);
        return;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_fb_close                                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_fb_close(void)
{
    struct fb_info *info = NULL;

    info = registered_fb[0];
    if (!info) {
        return;
    }
    if (info->fbops->fb_release) {
        info->fbops->fb_release(info, 0);
    }
    module_put(info->fbops->owner);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_boot_err_output                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_boot_err_output(void)
{
#ifdef SHDISP_RESET_LOG
    int i;
    struct shdisp_dbg_error_code* err_codes;
    int *err_codes_reset;
    int err_codes_count;

    for (i = 0; i < SHDISP_NOOS_RESET_NUM; i++) {
        if (shdisp_kerl_ctx.boot_ctx.err_on[i] == 1) {
            SHDISP_ERR("shdisp_boot_err_output.err_on.mode=%d.type=%d.code=%d.subcode=%d",
                    shdisp_kerl_ctx.boot_ctx.err_code[i].mode,
                    shdisp_kerl_ctx.boot_ctx.err_code[i].type,
                    shdisp_kerl_ctx.boot_ctx.err_code[i].code,
                    shdisp_kerl_ctx.boot_ctx.err_code[i].subcode);
            shdisp_kerl_ctx.boot_ctx.err_on[i] = 0;
            shdisp_dbg_API_err_output(&shdisp_kerl_ctx.boot_ctx.err_code[i], 0);
        }
    }

    shdisp_dbg_API_get_boot_errcodes(&err_codes, &err_codes_reset, &err_codes_count);
    for (i = 0; i < err_codes_count; i++) {
        SHDISP_ERR("shdisp_boot_err_output.err_codes.mode=%d.type=%d.code=%d.subcode=%d",
                err_codes[i].mode,
                err_codes[i].type,
                err_codes[i].code,
                err_codes[i].subcode);
        shdisp_dbg_API_err_output(&err_codes[i], 0);
    }
    shdisp_dbg_API_clear_boot_errcodes();
#endif /* SHDISP_RESET_LOG */
}

#ifdef SHDISP_ALS_INT
#ifdef SHDISP_IR2E71Y8
/* ------------------------------------------------------------------------- */
/* shdisp_input_subsystem_init                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_input_subsystem_init(void)
{
    int ret;

    wake_lock_init(&shdisp_timeout_wake_lock, WAKE_LOCK_SUSPEND, "shdisp_timeout_wake_lock");

    shdisp_input_dev = input_allocate_device();
    if (!shdisp_input_dev) {
        SHDISP_ERR("could not allocate input device\n");
        ret = -ENOMEM;
    }

    shdisp_input_dev->name = "als_interrupt";
    set_bit(EV_ABS, shdisp_input_dev->evbit);
    input_set_abs_params(shdisp_input_dev, ABS_MISC, 0, 9, 0, 0);

    ret = input_register_device(shdisp_input_dev);
    if (ret < 0) {
        SHDISP_ERR("can not register ls input device\n");
        input_free_device(shdisp_input_dev);
    }
    return ret;
}
/* ------------------------------------------------------------------------- */
/* shdisp_input_subsystem_report                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_input_subsystem_report(int val)
{
    int ret = 0;
    SHDISP_TRACE("in");

    wake_lock_timeout(&shdisp_timeout_wake_lock, 1 * HZ);

    input_report_abs(shdisp_input_dev, ABS_MISC, val);
    input_sync(shdisp_input_dev);

    input_report_abs(shdisp_input_dev, ABS_MISC, 0x00);
    input_sync(shdisp_input_dev);

    SHDISP_TRACE("out");
    return ret;
}
#endif /* SHDISP_IR2E71Y8 */
#endif /* SHDISP_ALS_INT */

#ifdef SHDISP_SYSFS_LED
/* ------------------------------------------------------------------------- */
/*shdisp_get_led_color                                                       */
/* ------------------------------------------------------------------------- */
static inline int shdisp_get_led_color(struct led_classdev *led_dev)
{
    if (!strncmp(led_dev->name, RED1_NAME, strlen(RED1_NAME))) {
        return SYSFS_LED_SH_RED;
    } else if (!strncmp(led_dev->name, GREEN1_NAME, strlen(GREEN1_NAME))) {
        return SYSFS_LED_SH_GREEN;
    } else if (!strncmp(led_dev->name, BLUE1_NAME, strlen(BLUE1_NAME))) {
        return SYSFS_LED_SH_BLUE;
    }
#ifdef SHDISP_COLOR_LED_TWIN
    else if (!strncmp(led_dev->name, RED2_NAME, strlen(RED2_NAME))) {
        return SYSFS_LED_SH_RED;
    } else if (!strncmp(led_dev->name, GREEN2_NAME, strlen(GREEN2_NAME))) {
        return SYSFS_LED_SH_GREEN;
    } else if (!strncmp(led_dev->name, BLUE2_NAME, strlen(BLUE2_NAME))) {
        return SYSFS_LED_SH_BLUE;
    }
#endif /* SHDISP_COLOR_LED_TWIN */
    return -1;
}

/* ------------------------------------------------------------------------- */
/*shdisp_get_led_no                                                          */
/* ------------------------------------------------------------------------- */
static inline int shdisp_get_led_no(struct led_classdev *led_cdev)
{
    if (!strncmp(led_cdev->name, RED1_NAME, strlen(RED1_NAME)) ||
        !strncmp(led_cdev->name, GREEN1_NAME, strlen(GREEN1_NAME)) ||
        !strncmp(led_cdev->name, BLUE1_NAME, strlen(BLUE1_NAME))) {
        return SYSFS_LED_SH_LED_1;
    }

#ifdef SHDISP_COLOR_LED_TWIN
    else if (!strncmp(led_cdev->name, RED2_NAME, strlen(RED2_NAME)) ||
        !strncmp(led_cdev->name, GREEN2_NAME, strlen(GREEN2_NAME)) ||
        !strncmp(led_cdev->name, BLUE2_NAME, strlen(BLUE2_NAME))) {
        return SYSFS_LED_SH_LED_2;
    }
#endif /* SHDISP_COLOR_LED_TWIN */

    return -1;
}


/* ------------------------------------------------------------------------- */
/*shdisp_led_brightness_set                                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_led_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	int led_no = 0;	
	int led_color = 0;
    struct shdisp_tri_led led;

    memset(&led, 0x00, sizeof(led));

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return;
    }

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
    if (shdisp_bdic_API_LED_is_running_illumi_triple_color()) {
        shdisp_bdic_API_LED_clear_illumi_triple_color();
    }
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

    led_no = shdisp_get_led_no(led_cdev);
    led_color = shdisp_get_led_color(led_cdev);

    SHDISP_DEBUG("before mode:%d", shdisp_kerl_ctx.sysfs_led1.led_mode);
    switch (shdisp_kerl_ctx.sysfs_led1.led_mode) {
    case SHDISP_TRI_LED_MODE_OFF:
        break;

    case SHDISP_TRI_LED_MODE_NORMAL:
        switch (led_no) {
        case SYSFS_LED_SH_LED_1:
            led.red   = shdisp_kerl_ctx.sysfs_led1.red;
            led.green = shdisp_kerl_ctx.sysfs_led1.green;
            led.blue  = shdisp_kerl_ctx.sysfs_led1.blue;
            break;
#ifdef SHDISP_COLOR_LED_TWIN
        case SYSFS_LED_SH_LED_2:
            led.red   = shdisp_kerl_ctx.sysfs_led2.red;
            led.green = shdisp_kerl_ctx.sysfs_led2.green;
            led.blue  = shdisp_kerl_ctx.sysfs_led2.blue;
            break;
#endif /* SHDISP_COLOR_LED_TWIN */
        default:
            SHDISP_ERR("invalid id. id no:%d", led_no);
            return;
        }
        break;

    case SHDISP_TRI_LED_MODE_BLINK:
    default:
        shdisp_clean_sysfs_led();
        if (value == 0) {
            SHDISP_DEBUG("leds off. no=%d", led_no);
            shdisp_semaphore_start();
            shdisp_bdic_API_TRI_LED_off();
            shdisp_semaphore_end(__func__);
            return;
        }
        break;
    }

    switch (led_color) {
    case SYSFS_LED_SH_RED:
        led.red   = value;
        break;
    case SYSFS_LED_SH_GREEN:
        led.green = value;
        break;
    case SYSFS_LED_SH_BLUE:
        led.blue  = value;
        break;
    default:
        SHDISP_ERR("invalid color. color no:%d", led_color);
        return;
    }

    SHDISP_DEBUG("%s(id:%d, color:%d) is %d\n", led_cdev->name, led_no, led_color, value);
    shdisp_semaphore_start();

    shdisp_SQE_tri_led_on(led_no, &led);

    shdisp_semaphore_end(__func__);
}

/* ------------------------------------------------------------------------- */
/*shdisp_led_brightness_get                                                 */
/* ------------------------------------------------------------------------- */
static enum led_brightness shdisp_led_brightness_get(struct led_classdev *led_cdev)
{
    int led_no = 0;
    int led_color = 0;
    int brightness = 0;

    led_no = shdisp_get_led_no(led_cdev);
    led_color = shdisp_get_led_color(led_cdev);

    switch (shdisp_kerl_ctx.sysfs_led1.led_mode) {
    case SHDISP_TRI_LED_MODE_NORMAL:
        switch (led_no) {
        case SYSFS_LED_SH_LED_1:
            switch (led_color) {
            case SYSFS_LED_SH_RED:
                brightness = shdisp_kerl_ctx.sysfs_led1.red;
                break;
            case SYSFS_LED_SH_GREEN:
                brightness = shdisp_kerl_ctx.sysfs_led1.green;
                break;
            case SYSFS_LED_SH_BLUE:
                brightness = shdisp_kerl_ctx.sysfs_led1.blue;
                break;
            default:
                SHDISP_ERR("invalid color. color no:%d", led_color);
                goto brightness_read_fail;
            }
            break;
#ifdef SHDISP_COLOR_LED_TWIN
        case SYSFS_LED_SH_LED_2:
            switch (led_color) {
            case SYSFS_LED_SH_RED:
                brightness = shdisp_kerl_ctx.sysfs_led2.red;
                break;
            case SYSFS_LED_SH_GREEN:
                brightness = shdisp_kerl_ctx.sysfs_led2.green;
                break;
            case SYSFS_LED_SH_BLUE:
                brightness = shdisp_kerl_ctx.sysfs_led2.blue;
                break;
            default:
                SHDISP_ERR("invalid color. color no:%d", led_color);
                goto brightness_read_fail;
            }
            break;
#endif /* SHDISP_COLOR_LED_TWIN */
        default:
            SHDISP_ERR("invalid id. id no:%d", led_no);
            goto brightness_read_fail;
        }
        break;

    default:
        SHDISP_DEBUG("mode:%d", shdisp_kerl_ctx.sysfs_led1.led_mode);
        goto brightness_read_fail;
    }

brightness_read_fail:
    return brightness;
}

/* ------------------------------------------------------------------------- */
/*shdisp_bdic_API_LED_blink_on                                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_LED_blink_on(int no, unsigned char color, struct shdisp_tri_led led)
{
    if (no == SYSFS_LED_SH_LED_1) {
        shdisp_bdic_API_TRI_LED_blink_on(color, led.ontime, led.interval, led.count);
    } else {
        /* not support SYSFS_LED_SH_LED_2 */
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_tri_led_blink_on                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_tri_led_blink_on(int no, struct shdisp_tri_led *led)
{
    unsigned char color = 0;
    struct shdisp_tri_led led_on;
    int ret;

    if (shdisp_led_set_color_reject()) {
        SHDISP_DEBUG("reject request.");
        return SHDISP_RESULT_SUCCESS;
    }

    color = shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(led);
    if (color == 0) { 
        SHDISP_DEBUG("leds off. no=%d", no); 
        led->led_mode = SHDISP_TRI_LED_MODE_OFF;
        ret = shdisp_bdic_API_LED_off(no);
    } else {
        SHDISP_DEBUG("leds blink on. no=%d, red:%d, green:%d, blue:%d",
                no, led->red, led->green, led->blue);
        led->led_mode = SHDISP_TRI_LED_MODE_BLINK;
        memcpy(&led_on, led, sizeof(led_on));
        led_on.ontime   = SHDISP_TRI_LED_ONTIME_TYPE1;
        led_on.interval = SHDISP_TRI_LED_INTERVAL_5S;
        led_on.count    = 0; 
        ret = shdisp_bdic_API_LED_blink_on(no, color, led_on);
        shdisp_clean_normal_led();
    } 

    switch (no) {
    case SYSFS_LED_SH_LED_1:
        shdisp_kerl_ctx.sysfs_led1 = *led;
        break;
#ifdef SHDISP_COLOR_LED_TWIN
    case SYSFS_LED_SH_LED_2:
        shdisp_kerl_ctx.sysfs_led2 = *led;
        break;
#endif /* SHDISP_COLOR_LED_TWIN */
    default:
        break;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/*shdisp_SQE_tri_led_blink_store                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_tri_led_blink_store(struct led_classdev *led_cdev, unsigned int data)
{
    struct shdisp_tri_led led;
    int led_color = 0;
    int led_no = 0;

    memset(&led, 0x00, sizeof(led));

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out2");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_DEBUG("before mode:%d", shdisp_kerl_ctx.sysfs_led1.led_mode);
    switch (shdisp_kerl_ctx.sysfs_led1.led_mode) {
    case SHDISP_TRI_LED_MODE_OFF:
        break;

    case SHDISP_TRI_LED_MODE_BLINK:
        led_no = shdisp_get_led_no(led_cdev);
        switch (led_no) {
            case SYSFS_LED_SH_LED_1:
                led.red   = shdisp_kerl_ctx.sysfs_led1.red;
                led.green = shdisp_kerl_ctx.sysfs_led1.green;
                led.blue  = shdisp_kerl_ctx.sysfs_led1.blue;
                break;
#ifdef SHDISP_COLOR_LED_TWIN
            case SYSFS_LED_SH_LED_2:
                led.red   = shdisp_kerl_ctx.sysfs_led2.red;
                led.green = shdisp_kerl_ctx.sysfs_led2.green;
                led.blue  = shdisp_kerl_ctx.sysfs_led2.blue;
                break;
#endif /* SHDISP_COLOR_LED_TWIN */
            default:
                SHDISP_ERR("invalid id. id no:%d", led_no);
                return SHDISP_RESULT_FAILURE;
        }
        break;

    case SHDISP_TRI_LED_MODE_NORMAL:
    default:
        shdisp_clean_sysfs_led();
        if (data == 0) {
            SHDISP_DEBUG("leds off. no=%d", led_no);
            shdisp_semaphore_start();
            shdisp_bdic_API_TRI_LED_off();
            shdisp_semaphore_end(__func__);
            return SHDISP_RESULT_SUCCESS;
        }
        break;
    }

    led_color = shdisp_get_led_color(led_cdev);
    switch (led_color) {
    case SYSFS_LED_SH_RED:
        led.red   = (unsigned int)data;
        break;
    case SYSFS_LED_SH_GREEN:
        led.green = (unsigned int)data;
        break;
    case SYSFS_LED_SH_BLUE:
        led.blue  = (unsigned int)data;
        break;
    default:
        SHDISP_ERR("invalid color. color no:%d", led_color);
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_DEBUG("%s: name:%s(id:%d, color:%d) value:%d\n", __func__, led_cdev->name, led_no, led_color, (int)data);

    shdisp_semaphore_start();

    shdisp_SQE_tri_led_blink_on(led_no, &led);

    shdisp_semaphore_end(__func__);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_led_blink_store                                                    */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_led_blink_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long data;
    ssize_t ret = -EINVAL;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);

    ret = kstrtoul(buf, 10, &data);
    if (ret) {
        return ret;
    }

    shdisp_SQE_tri_led_blink_store(led_cdev, (unsigned int)data);

    return count;
}

/* ------------------------------------------------------------------------- */
/* shdisp_led_blink_show                                                     */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_led_blink_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    int blinking = 0;
    int led_no = 0;
    int led_color = 0;

    led_no = shdisp_get_led_no(led_cdev);
    led_color = shdisp_get_led_color(led_cdev);

    switch (shdisp_kerl_ctx.sysfs_led1.led_mode) {
    case SHDISP_TRI_LED_MODE_BLINK:
        switch (led_no) {
        case SYSFS_LED_SH_LED_1:
            switch (led_color) {
            case SYSFS_LED_SH_RED:
                blinking = shdisp_kerl_ctx.sysfs_led1.red;
                break;
            case SYSFS_LED_SH_GREEN:
                blinking = shdisp_kerl_ctx.sysfs_led1.green;
                break;
            case SYSFS_LED_SH_BLUE:
                blinking = shdisp_kerl_ctx.sysfs_led1.blue;
                break;
            default:
                SHDISP_ERR("invalid color. color no:%d", led_color);
                goto blink_read_fail;
            }
            break;
#ifdef SHDISP_COLOR_LED_TWIN
        case SYSFS_LED_SH_LED_2:
            switch (led_color) {
            case SYSFS_LED_SH_RED:
                blinking = shdisp_kerl_ctx.sysfs_led2.red;
                break;
            case SYSFS_LED_SH_GREEN:
                blinking = shdisp_kerl_ctx.sysfs_led2.green;
                break;
            case SYSFS_LED_SH_BLUE:
                blinking = shdisp_kerl_ctx.sysfs_led2.blue;
                break;
            default:
                SHDISP_ERR("invalid color. color no:%d", led_color);
                goto blink_read_fail;
            }
            break;
#endif /* SHDISP_COLOR_LED_TWIN */
        default:
            SHDISP_ERR("invalid id. id no:%d", led_no);
            goto blink_read_fail;
        }
        break;

    default:
        SHDISP_DEBUG("mode:%d", shdisp_kerl_ctx.sysfs_led1.led_mode);
        goto blink_read_fail;
    }

blink_read_fail:
    return snprintf(buf, PAGE_SIZE, "%d\n", blinking);
}

/* ------------------------------------------------------------------------- */
/* shdisp_led_on_off_ms_store                                                */
/* ------------------------------------------------------------------------- */
static ssize_t shdisp_led_on_off_ms_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    int on_ms  = 0;
    int off_ms = 0;
    int ret;
    unsigned int data = 1;

    SHDISP_TRACE("in");
    ret = sscanf(buf, "%d %d", &on_ms, &off_ms);
    if (ret < 2) {
        return -EINVAL;
    }

    if ((on_ms <= 0) && (off_ms <= 0)) {
        data = 0;
    }

    SHDISP_DEBUG("on_ms:%d, off_ms:%d", on_ms, off_ms);
    shdisp_SQE_tri_led_blink_store(led_cdev, data);
    SHDISP_TRACE("out");

    return size;
}

/* ------------------------------------------------------------------------- */
/* shdisp_init_blink_sysfs                                                   */
/* ------------------------------------------------------------------------- */
static inline void shdisp_init_blink_sysfs(void)
{
    int rc = 0;

    if (red1_led.dev) {
        rc = sysfs_create_group(&red1_led.dev->kobj, &blink_attr_group);
        if (rc) {
            SHDISP_ERR("error sysfs blink_attr_group");
        }
    }
    if (green1_led.dev) {
        rc = sysfs_create_group(&green1_led.dev->kobj, &blink_attr_group);
        if (rc) {
            SHDISP_ERR("error sysfs blink_attr_group");
        }
    }
    if (blue1_led.dev) {
        rc = sysfs_create_group(&blue1_led.dev->kobj, &blink_attr_group);
        if (rc) {
            SHDISP_ERR("error sysfs blink_attr_group");
        }
    }
#ifdef SHDISP_COLOR_LED_TWIN
    if (red2_led.dev) {
        rc = sysfs_create_group(&red2_led.dev->kobj, &blink_attr_group);
        if (rc) {
            SHDISP_ERR("error sysfs blink_attr_group");
        }
    }
    if (green2_led.dev) {
        rc = sysfs_create_group(&green2_led.dev->kobj, &blink_attr_group);
        if (rc) {
            SHDISP_ERR("error sysfs blink_attr_group");
        }
    }
    if (blue2_led.dev) {
        rc = sysfs_create_group(&blue2_led.dev->kobj, &blink_attr_group);
        if (rc) {
            SHDISP_ERR("error sysfs blink_attr_group");
        }
    }
#endif /* SHDISP_COLOR_LED_TWIN */
}
#endif /* SHDISP_SYSFS_LED */

/* ------------------------------------------------------------------------- */
/* shdisp_init                                                               */
/* ------------------------------------------------------------------------- */
static int __init shdisp_init(void)
{
    int ret;
#ifdef SHDISP_IR2E71Y8
    struct shdisp_bdic_state_str    state_str;
#endif /* SHDISP_IR2E71Y8 */
    int shdisp_subscribe_type;
    int i;
    struct shdisp_main_bkl_ctl bkl_ctl;
    struct shdisp_tri_led tri_led;
    struct shdisp_panel_context shdisp_panel_ctx;

#if defined(CONFIG_ANDROID_ENGINEERING)
    struct proc_dir_entry *entry;
#endif /* CONFIG_ANDROID_ENGINEERING */

    memset(&tri_led, 0x00, sizeof(tri_led));
    memset(&bkl_ctl, 0x00, sizeof(bkl_ctl));
    memset(&shdisp_panel_ctx, 0x00, sizeof(shdisp_panel_ctx));

#ifdef SHDISP_IR2E71Y8
	memset(&state_str, 0x00, sizeof(state_str));
#endif /* SHDISP_IR2E71Y8 */

#ifdef SHDISP_NOT_SUPPORT_NO_OS
    SHDISP_TRACE("in NOT SUPPORT NO_OS\n")
#else   /* SHDISP_NOT_SUPPORT_NO_OS */
    SHDISP_TRACE("in SUPPORT NO_OS\n")
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

    shdisp_kerl_register_driver();

    shdisp_dbg_API_init();

    shdisp_init_context();

    shdisp_panel_API_create();

    shdisp_IO_API_Host_gpio_init();

    ret = alloc_chrdev_region(&shdisp_dev, 0, 1, SHDISP_NAME);

    if (!ret) {
        shdisp_major = MAJOR(shdisp_dev);
        shdisp_minor = MINOR(shdisp_dev);
    } else {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_1;
    }

    cdev_init(&shdisp_cdev, &shdisp_fops);

    shdisp_cdev.owner = THIS_MODULE;
    shdisp_cdev.ops   = &shdisp_fops;

    ret = cdev_add(&shdisp_cdev, MKDEV(shdisp_major, 0), 1);

    if (ret) {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_2;
    }

    shdisp_class = class_create(THIS_MODULE, SHDISP_NAME);

    if (IS_ERR(shdisp_class)) {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_3;
    }

    device_create(shdisp_class, NULL,
                  shdisp_dev, &shdisp_cdev, SHDISP_NAME);

    ret = shdisp_IO_API_bdic_i2c_init();

    if (ret) {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_4;
    }

    ret = shdisp_IO_API_sensor_i2c_init();

    if (ret) {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_6;
    }


    shdisp_panel_ctx.device_code    = shdisp_kerl_ctx.boot_ctx.device_code;
    shdisp_panel_ctx.disp_on_status = shdisp_kerl_ctx.boot_ctx.disp_on_status;
    shdisp_panel_ctx.vcom           = shdisp_kerl_ctx.boot_ctx.vcom;
    shdisp_panel_ctx.vcom_low       = shdisp_kerl_ctx.boot_ctx.vcom_low;
    shdisp_panel_ctx.vcom_nvram     = shdisp_kerl_ctx.boot_ctx.vcom_nvram;

    memcpy(&(shdisp_panel_ctx.lcddr_phy_gmm), &(shdisp_kerl_ctx.boot_ctx.lcddr_phy_gmm),
            sizeof(struct shdisp_lcddr_phy_gmm_reg));

    ret = shdisp_panel_API_init_io(&shdisp_panel_ctx);

    if (ret) {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_61;
    }

    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        first_lcd_on = true;
    }

#if defined(CONFIG_ANDROID_ENGINEERING)
    memset(&proc_fops, 0, sizeof(struct file_operations));
    proc_fops.read = shdisp_proc_file_read;
    proc_fops.write = shdisp_proc_file_write;

    entry = proc_create("driver/SHDISP", 0666, NULL, &proc_fops);

    if (entry == NULL) {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_7;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    sema_init(&shdisp_sem, 1);

    sema_init(&shdisp_sem_callback, 1);
    sema_init(&shdisp_sem_irq_fac, 1);
    sema_init(&shdisp_sem_timer, 1);
    sema_init(&shdisp_sem_req_recovery_lcd, 1);
    sema_init(&shdisp_sem_req_recovery_psals, 1);

    spin_lock_init(&shdisp_q_lock);
    spin_lock_init(&shdisp_wake_spinlock);
#ifdef SHDISP_IR2E71Y8
    shdisp_SYS_API_set_irq_init();
#endif /* SHDISP_IR2E71Y8 */
    shdisp_wake_lock_init();

    memset(&shdisp_queue_data, 0, sizeof(shdisp_queue_data));
    INIT_LIST_HEAD( &shdisp_queue_data.list);

    shdisp_wq_gpio = create_singlethread_workqueue("shdisp_gpio_queue");

#ifdef SHDISP_IR2E71Y8
    if (shdisp_wq_gpio) {
        INIT_WORK(&shdisp_wq_gpio_wk,
                  shdisp_workqueue_handler_gpio);
    } else {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_8;
    }

    shdisp_wq_gpio_task = create_singlethread_workqueue("shdisp_gpio_queue_task");
    if (shdisp_wq_gpio_task) {
        INIT_WORK(&shdisp_wq_gpio_task_wk,
                  shdisp_workqueue_gpio_task);
    } else {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_9;
    }
#endif /* SHDISP_IR2E71Y8 */
    shdisp_wq_recovery = create_singlethread_workqueue("shdisp_recovery_task");

    if (shdisp_wq_recovery) {
        INIT_WORK(&shdisp_wq_recovery_lcd_wk,
                  shdisp_workqueue_handler_recovery_lcd);
        INIT_WORK(&shdisp_wq_recovery_psals_wk,
                  shdisp_workqueue_handler_recovery_psals);
    } else {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_91;
    }

    down(&shdisp_sem_callback);
    for (i = 0; i < NUM_SHDISP_IRQ_TYPE ; i++) {
        shdisp_callback_table[i] = NULL;
    }
    up(&shdisp_sem_callback);

    init_timer(&shdisp_timer);

    shdisp_wq_timer_task = create_singlethread_workqueue("shdisp_timer_queue_task");
    if (shdisp_wq_timer_task) {
        INIT_WORK(&shdisp_wq_timer_task_wk, shdisp_workqueue_timer_task);
    } else {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_10;
    }

#ifdef SHDISP_FPS_LED_PANEL_SUPPORT
    shdisp_fps_led_ctx.panel_on = shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON;
    shdisp_fps_led_ctx.workq = create_singlethread_workqueue("shdisp_fps_led_workq");
    INIT_DELAYED_WORK(&shdisp_fps_led_ctx.work, shdisp_fps_led_work);
#endif /* SHDISP_FPS_LED_PANEL_SUPPORT */

    for (i = 0; i < NUM_SHDISP_IRQ_TYPE ; i++) {
        shdisp_subscribe_type = SHDISP_SUBSCRIBE_TYPE_INT;
        shdisp_subscribe_type_table[i] = shdisp_subscribe_type;
    }

    shdisp_kerl_ctx.driver_is_initialized = SHDISP_DRIVER_IS_INITIALIZED;

#ifdef SHDISP_IR2E71Y8
    state_str.bdic_chipver  = shdisp_kerl_ctx.boot_ctx.bdic_chipver;
    state_str.handset_color = shdisp_kerl_ctx.boot_ctx.handset_color;

    memcpy(&(state_str.photo_sensor_adj),
                                &(shdisp_kerl_ctx.boot_ctx.photo_sensor_adj), sizeof(struct shdisp_photo_sensor_adj));

    shdisp_bdic_API_kerl_init();
#ifdef SHDISP_NOT_SUPPORT_NO_OS
    shdisp_kerl_ctx.boot_ctx.bdic_is_exist = shdisp_bdic_API_boot_init(&state_str.bdic_chipver);
    shdisp_kerl_ctx.boot_ctx.bdic_chipver = state_str.bdic_chipver;
    shdisp_bdic_API_boot_init_2nd();
    shdisp_bdic_API_initialize(&state_str);
#else  /* SHDISP_NOT_SUPPORT_NO_OS*/
    if (shdisp_check_bdic_exist() == SHDISP_RESULT_SUCCESS) {
        shdisp_bdic_API_initialize(&state_str);
    }
#endif /* SHDISP_NOT_SUPPORT_NO_OS*/

    bkl_ctl.mode  = shdisp_kerl_ctx.main_bkl.mode;
    bkl_ctl.param = shdisp_kerl_ctx.main_bkl.param;

    shdisp_bdic_API_LCD_BKL_set_request(SHDISP_MAIN_BKL_DEV_TYPE_APP, &bkl_ctl);

    tri_led.red      = shdisp_kerl_ctx.tri_led.red;
    tri_led.green    = shdisp_kerl_ctx.tri_led.green;
    tri_led.blue     = shdisp_kerl_ctx.tri_led.blue;
    tri_led.ext_mode = shdisp_kerl_ctx.tri_led.ext_mode;
    tri_led.led_mode = shdisp_kerl_ctx.tri_led.led_mode;
    tri_led.ontime   = shdisp_kerl_ctx.tri_led.ontime;
    tri_led.interval = shdisp_kerl_ctx.tri_led.interval;
    tri_led.count    = shdisp_kerl_ctx.tri_led.count;

    shdisp_kerl_ctx.sysfs_led1.red   = ((shdisp_kerl_ctx.tri_led.red   > 0) ? 255 : 0);
    shdisp_kerl_ctx.sysfs_led1.green = ((shdisp_kerl_ctx.tri_led.green > 0) ? 255 : 0);
    shdisp_kerl_ctx.sysfs_led1.blue  = ((shdisp_kerl_ctx.tri_led.blue  > 0) ? 255 : 0);
    if ((tri_led.red + tri_led.green + tri_led.blue) != 0) {
        shdisp_kerl_ctx.sysfs_led1.led_mode = SHDISP_TRI_LED_MODE_NORMAL;
    } else {
        shdisp_kerl_ctx.sysfs_led1.led_mode = SHDISP_TRI_LED_MODE_OFF;
    }

    shdisp_bdic_API_TRI_LED_set_request(&tri_led);
    if (shdisp_kerl_ctx.boot_ctx.bdic_is_exist == SHDISP_BDIC_IS_EXIST) {
        ret = shdisp_SYS_API_request_irq(shdisp_gpio_int_isr);
    }
    if (ret) {
        SHDISP_ERR("shdisp_init failed. l=%d\n", __LINE__);
        goto shdisp_err_top;
    }
    shdisp_pm_API_init(&(shdisp_kerl_ctx.boot_ctx));
#endif /* SHDISP_IR2E71Y8 */
    shdisp_fb_open();

#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_LCDPOW, shdisp_kerl_ctx.main_disp_status);
#endif  /* CONFIG_SHTERM */
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_bkl_notify(SHDISP_INFO_BACKLIGHT | SHDISP_INFO_BACKLIGHT_LEV);

#ifdef SHDISP_ALS_INT
    shdisp_input_subsystem_init();
#endif /* SHDISP_ALS_INT */
#endif /* SHDISP_IR2E71Y8 */

    SHDISP_TRACE("out\n")

    return SHDISP_RESULT_SUCCESS;

#ifdef SHDISP_IR2E71Y8
shdisp_err_top:
    flush_workqueue(shdisp_wq_timer_task);
    destroy_workqueue(shdisp_wq_timer_task);
    shdisp_wq_timer_task = NULL;
#endif /* SHDISP_IR2E71Y8 */

shdisp_err_10:
    flush_workqueue(shdisp_wq_recovery);
    destroy_workqueue(shdisp_wq_recovery);
    shdisp_wq_recovery = NULL;

shdisp_err_91:
    flush_workqueue(shdisp_wq_gpio_task);
    destroy_workqueue(shdisp_wq_gpio_task);
    shdisp_wq_gpio_task = NULL;
#ifdef SHDISP_IR2E71Y8
shdisp_err_9:
    flush_workqueue(shdisp_wq_gpio);
    destroy_workqueue(shdisp_wq_gpio);
    shdisp_wq_gpio = NULL;
shdisp_err_8:
#endif /* SHDISP_IR2E71Y8 */
#if defined(CONFIG_ANDROID_ENGINEERING)
shdisp_err_7:
#endif /* CONFIG_ANDROID_ENGINEERING */
    shdisp_panel_API_exit_io();

shdisp_err_61:

shdisp_err_6:
    shdisp_IO_API_sensor_i2c_exit();
    shdisp_IO_API_bdic_i2c_exit();

shdisp_err_4:
    device_destroy(shdisp_class, MKDEV(shdisp_major, 0));

shdisp_err_3:
    class_destroy(shdisp_class);

shdisp_err_2:
    cdev_del(&shdisp_cdev);

shdisp_err_1:
    shdisp_IO_API_Host_gpio_exit();
    unregister_chrdev_region(MKDEV(shdisp_major, 0), 1);
    return -EIO;
}
module_init(shdisp_init);

/* ------------------------------------------------------------------------- */
/* shdisp_exit                                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_exit(void)
{
    shdisp_fb_close();

#ifdef SHDISP_FPS_LED_PANEL_SUPPORT
    shdisp_fps_led_stop();
    if (shdisp_fps_led_ctx.workq) {
        flush_workqueue(shdisp_fps_led_ctx.workq);
        destroy_workqueue(shdisp_fps_led_ctx.workq);
        shdisp_fps_led_ctx.workq = NULL;
    }
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */

#ifdef SHDISP_ALS_INT
    wake_unlock(&shdisp_timeout_wake_lock);
    wake_lock_destroy(&shdisp_timeout_wake_lock);
#endif /* SHDISP_ALS_INT */
    wake_lock_destroy(&shdisp_wake_lock_wq);

    shdisp_SYS_API_free_irq();
    if (shdisp_wq_gpio) {
        flush_workqueue(shdisp_wq_gpio);
        destroy_workqueue(shdisp_wq_gpio);
        shdisp_wq_gpio = NULL;
    }

    if (shdisp_wq_gpio_task) {
        flush_workqueue(shdisp_wq_gpio_task);
        destroy_workqueue(shdisp_wq_gpio_task);
        shdisp_wq_gpio_task = NULL;
    }
#ifdef SHDISP_IR2E71Y8
    shdisp_bdic_API_TRI_LED_exit();
#endif /* SHDISP_IR2E71Y8 */
    shdisp_panel_API_exit_io();
    shdisp_IO_API_sensor_i2c_exit();
    shdisp_IO_API_bdic_i2c_exit();
    shdisp_IO_API_Host_gpio_exit();
    device_destroy(shdisp_class, MKDEV(shdisp_major, 0));
    class_destroy(shdisp_class);
    cdev_del(&shdisp_cdev);
    unregister_chrdev_region(MKDEV(shdisp_major, 0), 1);
    return;
}
module_exit(shdisp_exit);

/* ------------------------------------------------------------------------- */
/* shdisp_kerl_probe                                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_kerl_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
    int rc = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in pdev = 0x%p", pdev);

    if (pdev) {
        if (&(pdev->dev) != NULL) {
#ifdef SHDISP_NOT_SUPPORT_NO_OS
            memset(&testmode_info, 0x00, sizeof(testmode_info));
            testmode_info.gpio = of_get_named_gpio(pdev->dev.of_node, "test_mode_gpio", 0);
            if (!gpio_is_valid(testmode_info.gpio)) {
                SHDISP_ERR("testmode gpio not specified");
            } else {
                SHDISP_DEBUG("testmode gpio succusess!");
            }
            testmode_info.pinctrl = devm_pinctrl_get(&pdev->dev);
            if (IS_ERR_OR_NULL(testmode_info.pinctrl)) {
                SHDISP_ERR("testmode pinctrl not specified");
            } else {
                SHDISP_DEBUG("testmode pinctrl succusess!");
            }
            testmode_info.state_active = pinctrl_lookup_state(testmode_info.pinctrl, "test_mode_pull_up");
            if (IS_ERR_OR_NULL(testmode_info.state_active)) {
                SHDISP_ERR("testmode gpio_state_active not specified");
            } else {
                SHDISP_DEBUG("testmode gpio_state_active succusess!");
            }
            testmode_info.state_suspend = pinctrl_lookup_state(testmode_info.pinctrl, "test_mode_pull_down");
            if (IS_ERR_OR_NULL(testmode_info.state_suspend)) {
                SHDISP_ERR("testmode gpio_state_suspend not specified");
            } else {
                SHDISP_DEBUG("testmode gpio_state_suspend succusess!");
            }
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */
#ifdef SHDISP_SYSFS_LED
            if (led_classdev_register(&pdev->dev, &red1_led)) {
                SHDISP_ERR("led_classdev_register(red1) failed");
            }
            if (led_classdev_register(&pdev->dev, &green1_led)) {
                SHDISP_ERR("led_classdev_register(green1) failed");
            }
            if (led_classdev_register(&pdev->dev, &blue1_led)) {
                SHDISP_ERR("led_classdev_register(blue1) failed");
            }
#ifdef SHDISP_COLOR_LED_TWIN
            if (led_classdev_register(&pdev->dev, &red2_led)) {
                SHDISP_ERR("led_classdev_register(red2) failed");
            }
            if (led_classdev_register(&pdev->dev, &green2_led)) {
                SHDISP_ERR("led_classdev_register(green2) failed");
            }
            if (led_classdev_register(&pdev->dev, &blue2_led)) {
                SHDISP_ERR("led_classdev_register(blue2) failed");
            }
#endif /* SHDISP_COLOR_LED_TWIN */
            /* Add sysfs for rgb */
            shdisp_init_blink_sysfs();
#endif /* SHDISP_SYSFS_LED */
        } else {
            SHDISP_ERR("pdev->dev is NULL");
        }
    } else {
        SHDISP_ERR("pdev is NULL");
    }
#ifdef SHDISP_NOT_SUPPORT_NO_OS
    SHDISP_TRACE("out testmode_gpio = %d", testmode_info.gpio);
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

    return rc;
#else   /* CONFIG_OF */
    return SHDISP_RESULT_SUCCESS;
#endif  /* CONFIG_OF */
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_kerl_match[] = {
    { .compatible = "sharp,shdisp_kerl", },
    {}
};
#else   /* CONFIG_OF */
#define shdisp_kerl_match NULL
#endif  /* CONFIG_OF */

static struct platform_driver shdisp_kerl_driver = {
    .probe = shdisp_kerl_probe,
    .remove = NULL,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_kerl",
        .of_match_table = shdisp_kerl_match,
    },
};

/* ------------------------------------------------------------------------- */
/* shdisp_kerl_register_driver                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_kerl_register_driver(void)
{
    return platform_driver_register(&shdisp_kerl_driver);
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
