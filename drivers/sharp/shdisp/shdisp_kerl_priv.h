/* drivers/sharp/shdisp/shdisp_kerl_priv.h  (Display Driver)
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
#ifndef SHDISP_KERL_PRIV_H
#define SHDISP_KERL_PRIV_H
unsigned short shdisp_API_get_hw_revision(void);
unsigned short shdisp_API_get_hw_handset(void);
int shdisp_API_get_bdic_is_exist(void);
int shdisp_API_is_open(void);
int shdisp_API_do_lcd_det_recovery(void);
int shdisp_API_is_lcd_det_recovering(void);
void shdisp_API_psals_recovery_subscribe( void );
void shdisp_API_psals_recovery_unsubscribe(void);
int shdisp_API_check_upper_unit(void);
void shdisp_API_semaphore_start(void);
void shdisp_API_semaphore_end(void);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
struct shdisp_queue_data_t {
    int                 irq_GFAC;
    struct list_head    list;
};

enum {
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_POL,
    NUM_SHDISP_SUBSCRIBE_TYPE
};

enum {
    SHDISP_DEBUG_PROCESS_STATE_OUTPUT,
    SHDISP_DEBUG_TRACE_LOG_SWITCH,
    SHDISP_DEBUG_BDIC_I2C_WRITE,
    SHDISP_DEBUG_BDIC_I2C_READ,
    SHDISP_DEBUG_PROX_SENSOR_CTL,
    SHDISP_DEBUG_BKL_CTL,
    SHDISP_DEBUG_LIGHT_SENSOR_CTL,
    SHDISP_DEBUG_IRQ_LOGIC_CHK = 10,
    SHDISP_DEBUG_BDIC_IRQ_ALL_CLEAR = 11,
    SHDISP_DEBUG_BDIC_IRQ_CLEAR = 12,
    SHDISP_DEBUG_DUMMY_SUBSCRIBE = 13,
    SHDISP_DEBUG_DUMMY_UNSUBSCRIBE_PS = 14,
    SHDISP_DEBUG_BDIC_WRITE = 20,
    SHDISP_DEBUG_BDIC_READ = 21,
    SHDISP_DEBUG_KEY_BKL_LED = 24,
    SHDISP_DEBUG_RGB_LED = 25,
    SHDISP_DEBUG_LED_REG_DUMP = 26,
    SHDISP_DEBUG_BDIC_RESTART = 27,
    SHDISP_DEBUG_LCD_POWER_CHG = 30,
    SHDISP_DEBUG_MIPI_TX_FREQ_CHG = 40,
    SHDISP_DEBUG_FPS_LED = 41,
    SHDISP_DEBUG_DISPLAYLOG_ERROR_LOG_TEST = 45,
    SHDISP_DEBUG_DISPLAYLOG_SUMMARY_TEST = 46,
    SHDISP_DEBUG_CHARGE_BLK_MODE = 51,
    SHDISP_DEBUG_LOW_BLK_MODE = 52,
    SHDISP_DEBUG_EMG_BLK_MODE = 53,
    SHDISP_DEBUG_RECOVERY_NG = 60,
    SHDISP_DEBUG_ALS_REPORT = 70,
    SHDISP_DEBUG_DSI_DCS_WRITE = 90,
    SHDISP_DEBUG_DSI_DCS_READ = 91,
    SHDISP_DEBUG_DSI_GEN_WRITE = 92,
    SHDISP_DEBUG_DSI_GEN_READ = 93,
    SHDISP_DEBUG_SET_IRQ = 98,
    SHDISP_DEBUG_DO_RECOVERY = 99,
};

enum {
    SHDISP_DEBUG_INFO_TYPE_BOOT,
    SHDISP_DEBUG_INFO_TYPE_KERNEL,
    SHDISP_DEBUG_INFO_TYPE_BDIC,
    SHDISP_DEBUG_INFO_TYPE_SENSOR,
    SHDISP_DEBUG_INFO_TYPE_POWERON,
    SHDISP_DEBUG_INFO_TYPE_PANEL,
    SHDISP_DEBUG_INFO_TYPE_PM,
    SHDISP_DEBUG_INFO_TYPE_BDIC_OPT,
    NUM_SHDISP_DEBUG_INFO_TYPE
};

enum {
    SHDISP_BKL_MODE_OFF,
    SHDISP_BKL_MODE_ON,
    SHDISP_BKL_MODE_AUTO,
    NUM_SHDISP_BKL_MODE
};

enum {
    SHDISP_ALS_IRQ_SUBSCRIBE_TYPE_BKL_CTRL,
    SHDISP_ALS_IRQ_SUBSCRIBE_TYPE_DBC_IOCTL,
    NUM_SHDISP_ALS_IRQ_SUBSCRIBE_TYPE
};

enum {
    SHDISP_INFO_BACKLIGHT = 0,
    SHDISP_INFO_BACKLIGHT_LEV
};

#endif /* SHDISP_KERL_PRIV_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
