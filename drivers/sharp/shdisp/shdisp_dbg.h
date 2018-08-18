/* drivers/sharp/shdisp/shdisp_dbg.h  (Display Driver)
 *
 * Copyright (C) 2012 SHARP CORPORATION
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
#ifndef SHDISP_DBG_H
#define SHDISP_DBG_H


#include <linux/tty.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_ANDROID_ENGINEERING)
    #define SHDISP_LOG_ENABLE
#endif /* CONFIG_ANDROID_ENGINEERING */


/*#define SHDISP_RESET_LOG*/


#ifdef SHDISP_LOG_ENABLE
    extern unsigned short shdisp_log_lv;
    #define SHDISP_SET_LOG_LV(lv) shdisp_log_lv = lv;
    #define SHDISP_PRINTK(lv, fmt, args...) \
            if ((lv & (SHDISP_LOG_LV_DEBUG | SHDISP_LOG_LV_TRACE | SHDISP_LOG_LV_WARN | SHDISP_LOG_LV_ERR)) != 0) { \
                shdisp_dbg_API_printk(fmt, ## args); \
            } \
            if ((shdisp_log_lv & lv) != 0) { \
                printk(fmt, ## args); \
            }
#else   /* SHDISP_LOG_ENABLE */
    #define SHDISP_SET_LOG_LV(lv)
    #define SHDISP_PRINTK(lv, fmt, args...) \
            if ((lv & (SHDISP_LOG_LV_DEBUG | SHDISP_LOG_LV_TRACE | SHDISP_LOG_LV_WARN | SHDISP_LOG_LV_ERR)) != 0) { \
                shdisp_dbg_API_printk(fmt, ## args); \
            }
#endif   /* SHDISP_LOG_ENABLE */

#define SHDISP_LOG_LV_ERR               (0x01)
#define SHDISP_LOG_LV_WARN              (0x02)
#define SHDISP_LOG_LV_DEBUG             (0x04)
#define SHDISP_LOG_LV_TRACE             (0x08)
#define SHDISP_LOG_LV_INFO              (0x10)
#define SHDISP_LOG_LV_PERFORM           (0x20)
#define SHDISP_LOG_LV_PERFORM_DEBUG     (0x40)
#define SHDISP_LOG_LV_I2CLOG            (0x80)
#define SHDISP_LOG_LV_DSILOG            (0x100)
#define SHDISP_LOG_LV_WAITLOG           (0x200)
#define SHDISP_LOG_LV_ALL               (0x1FF)

#if defined(CONFIG_ANDROID_ENGINEERING)
    #define SHDISP_ERR(fmt, args...) \
    SHDISP_PRINTK(SHDISP_LOG_LV_ERR, KERN_ERR "[SHDISP_ERROR][%s] " fmt "\n", __func__, ## args)
#else   /* defined (CONFIG_ANDROID_ENGINEERING) */
    #define SHDISP_ERR(fmt, args...) printk(KERN_ERR "[SHDISP_ERROR][%s] " fmt "\n", __func__, ## args); \
        SHDISP_PRINTK(SHDISP_LOG_LV_ERR, KERN_ERR "[SHDISP_ERROR][%s] " fmt "\n", __func__, ## args);
#endif  /* defined (CONFIG_ANDROID_ENGINEERING) */

#define SHDISP_TRACE(fmt, args...) \
        SHDISP_PRINTK(SHDISP_LOG_LV_TRACE, KERN_INFO "[SHDISP_TRACE][%s] " fmt "\n", __func__, ## args)

#define SHDISP_DEBUG(fmt, args...) \
        SHDISP_PRINTK(SHDISP_LOG_LV_DEBUG, KERN_DEBUG "[SHDISP_DEBUG][%s] " fmt "\n", __func__, ## args)

#define SHDISP_WARN(fmt, args...) \
        SHDISP_PRINTK(SHDISP_LOG_LV_WARN, KERN_WARNING "[SHDISP_WARNING][%s] " fmt "\n", __func__, ## args)

#define SHDISP_INFO(fmt, args...) \
        SHDISP_PRINTK(SHDISP_LOG_LV_INFO, KERN_INFO "[SHDISP_INFO][%s] " fmt "\n", __func__, ## args)

#define SHDISP_I2CLOG(fmt, args...) \
        SHDISP_PRINTK(SHDISP_LOG_LV_I2CLOG, KERN_DEBUG "[SHDISP_I2CLOG][%s] " fmt "\n", __func__, ## args)

#define SHDISP_DSI_LOG(fmt, args...) \
        SHDISP_PRINTK(SHDISP_LOG_LV_DSILOG, KERN_DEBUG "[SHDISP_DSI][%s] " fmt "\n", __func__, ## args)

#define SHDISP_WAIT_LOG(fmt, args...) \
        SHDISP_PRINTK(SHDISP_LOG_LV_WAITLOG, KERN_DEBUG "[SHDISP_WAIT][%s] " fmt "\n", __func__, ## args)

#if defined(CONFIG_ANDROID_ENGINEERING)
    #define SHDISP_PERFORMANCE(fmt, args...) \
            SHDISP_PRINTK(SHDISP_LOG_LV_PERFORM, ",[SHDISP_PERFORM]" fmt "\n", ## args)
    #define SHDISP_PERFORMANCE_DEBUG(fmt, args...) \
            SHDISP_PRINTK(SHDISP_LOG_LV_PERFORM_DEBUG, ",[SHDISP_PERFORM_DEBUG]" fmt "\n", ## args)
#else /* CONFIG_ANDROID_ENGINEERING */
    #define SHDISP_PERFORMANCE(fmt, args...)
    #define SHDISP_PERFORMANCE_DEBUG(fmt, args...)
#endif /* CONFIG_ANDROID_ENGINEERING */

extern struct tty_struct *shdisp_tty;

#if defined(CONFIG_ANDROID_ENGINEERING)
#define SHDISP_DBG_FAIL_RETRY_OFF_PANEL_PRESSOR         (2)
#define SHDISP_DBG_FAIL_RETRY_ON                        (0)

#define SHDISP_DBG_RESET_PANEL_RETRY_OVER               (2)
#define SHDISP_DBG_DUMP_RESET                           (1)
#define SHDISP_DBG_RESET_OFF                            (0)

#define SHDISP_DBG_RECOVERY_ERROR_NONE                  (0)
#define SHDISP_DBG_RECOVERY_ERROR_DISPON                (1)
#define SHDISP_DBG_RECOVERY_ERROR_DETLOW                (2)
#define SHDISP_DBG_RECOVERY_ERROR_PSALS                 (3)
#define SHDISP_DBG_RECOVERY_ERROR_DISPON_READ           (4)
#define SHDISP_DBG_BDIC_ERROR_DCDC1                     (5)
#endif /* CONFIG_ANDROID_ENGINEERING */

#define SHDISP_DBG_ERR_HEAP_NULL                        (-1)
#define SHDISP_DBG_INFO_NO_OS                           (-2)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
#define LOG_BUF_SIZE              (8192)
#define LOG_BUF_SIZE_TO_KERNEL    (LOG_BUF_SIZE / 2)
#define LINE_BUF_SIZE             (1024)
#define LOG_BUF_SIZE_ST           (4096)

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
struct shdisp_dbg_error_code;

extern void shdisp_dbg_API_init(void);
extern int shdisp_dbg_API_printk(const char *fmt, ...);
extern size_t shdisp_dbg_API_stacktrace_dump(char *buf, size_t length);
extern int shdisp_dbg_API_ringbuffer_dump(char *buf);

#if defined(CONFIG_ANDROID_ENGINEERING)
void shdisp_dbg_API_set_reset_flg(int sw);
int shdisp_dbg_API_get_reset_flg(void);

void shdisp_dbg_API_set_recovery_check_error(int flag, int count);
int  shdisp_dbg_API_get_recovery_check_error(void);
void shdisp_dbg_API_update_recovery_check_error(int flag);
#endif /* CONFIG_ANDROID_ENGINEERING */

enum {
    SHDISP_DBG_MODE_LINUX           = 0,
    SHDISP_DBG_MODE_APPSBL,
    SHDISP_DBG_MODE_NoOS,
    SHDISP_DBG_MODE_LINUX_BOOTED,
    SHDISP_DBG_MODE_LINUX_BOOTING,
    SHDISP_DBG_MODE_MAX
};

enum {
    SHDISP_DBG_TYPE_PANEL           = 0,
    SHDISP_DBG_TYPE_BDIC,
    SHDISP_DBG_TYPE_PSALS,
    SHDISP_DBG_TYPE_MAX
};

enum {
    SHDISP_DBG_CODE_RETRY_OVER      = 0,
    SHDISP_DBG_CODE_READ_ERROR,
    SHDISP_DBG_CODE_ERROR_DETECT,
    SHDISP_DBG_CODE_MAX
};

enum {
    SHDISP_DBG_SUBCODE_NONE         = 0,
    SHDISP_DBG_SUBCODE_DISPON_CHK,
    SHDISP_DBG_SUBCODE_DET_LOW,
    SHDISP_DBG_SUBCODE_ESD_DETIN,
    SHDISP_DBG_SUBCODE_ESD_MIPI,
    SHDISP_DBG_SUBCODE_I2C_READ,
    SHDISP_DBG_SUBCODE_I2C_ERROR,
    SHDISP_DBG_SUBCODE_PS_REQ,
    SHDISP_DBG_SUBCODE_RECOVERY_NG,
    SHDISP_DBG_SUBCODE_PSALS_ON_NG,
    SHDISP_DBG_SUBCODE_POWER_ON_NG,
    SHDISP_DBG_SUBCODE_DEVCODE,
    SHDISP_DBG_SUBCODE_MAX
};

extern int shdisp_dbg_API_err_output(struct shdisp_dbg_error_code *code, int reset);
extern int shdisp_dbg_API_add_err_log(struct shdisp_dbg_error_code *code);
extern int shdisp_dbg_API_err_countup(struct shdisp_dbg_error_code *code);
extern int shdisp_dbg_API_set_subcode(int);
extern int shdisp_dbg_API_get_subcode(void);
#ifdef SHDISP_RESET_LOG
extern void shdisp_dbg_API_get_boot_errcodes(struct shdisp_dbg_error_code **codes, int **reset, int *count);
extern void shdisp_dbg_API_clear_boot_errcodes(void);
#endif /* SHDISP_RESET_LOG */

#ifdef SHDISP_DBG_DUMP
struct shdisp_dbg_ptrinfo {
    unsigned char *ptr;
    size_t length;
    int need_free;
};

extern int shdisp_dbg_zip_out_buflen_override;
extern int shdisp_dbg_dispdump_worker_wait_ms;

extern int shdisp_dbg_API_display_dump(
        int mode,
        struct shdisp_dbg_ptrinfo *ring_dump_buf,
        struct shdisp_dbg_ptrinfo *stack_dump_buf,
        struct shdisp_dbg_ptrinfo *edram_dump_buf,
        struct shdisp_dbg_ptrinfo *sram_dump_buf);
#endif /* SHDISP_DBG_DUMP */

#endif /* SHDISP_DBG_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
