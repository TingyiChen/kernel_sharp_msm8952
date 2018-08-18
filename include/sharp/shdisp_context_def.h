/* include/sharp/shdisp_context_def.h  (Display Driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
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
#ifndef SHDISP_CONTEXT_DEF_H
#define SHDISP_CONTEXT_DEF_H

#define SHDISP_LCDC_EWB_TBL_SIZE                    (256)
#define SHDISP_NOOS_RESET_NUM                       (3)

#define SHDISP_ARGC_MBR_NUM                         (3)
#define SHDISP_ARGC_STAGE_NUM                       (16)
#define SHDISP_IGC_LUT_ENTRIES                      (256)
#define SHDISP_IGC_LUT_MAX_NUM                      (0xFFF)

#if defined(CONFIG_SHDISP_PANEL_SAZABI) || defined(USER_CONFIG_SHDISP_PANEL_SAZABI)
#define SHDISP_LCDDR_BUF_MAX    (71)
#else  /* USER_CONFIG_SHDISP_PANEL_SAZABI */
#define SHDISP_LCDDR_BUF_MAX    (64)
#endif  /* USER_CONFIG_SHDISP_PANEL_SAZABI */

struct shdisp_dbg_error_code {
    unsigned char mode;
    unsigned char type;
    unsigned char code;
    unsigned char subcode;
};

struct shdisp_main_bkl_ctl {
    int mode;
    int param;
};

struct shdisp_tri_led {
    unsigned int red;
    unsigned int green;
    unsigned int blue;
    int ext_mode;
    int led_mode;
    int ontime;
    int interval;
    int count;
};

struct shdisp_key_bkl_ctl {
    unsigned char key_left;
    unsigned char key_center;
    unsigned char key_right;
    int ontime;
    int interval;
};

struct shdisp_als_adjust {
    unsigned short als_adj0;
    unsigned short als_adj1;
    unsigned char als_shift;
    unsigned char clear_offset;
    unsigned char ir_offset;
};

struct shdisp_photo_sensor_adj {
    unsigned char status;
    unsigned char key_backlight;
    unsigned int chksum;
    struct shdisp_als_adjust als_adjust[2];
};

struct shdisp_ledc_req {
    unsigned int red[2];
    unsigned int green[2];
    unsigned int blue[2];
    int led_mode;
    int on_count;
};

struct shdisp_ledc_status {
    int ledc_is_exist;
    int power_status;
    struct shdisp_ledc_req ledc_req;
};

struct shdisp_argc_lut {
    unsigned short  red[SHDISP_ARGC_STAGE_NUM][SHDISP_ARGC_MBR_NUM];
    unsigned short  green[SHDISP_ARGC_STAGE_NUM][SHDISP_ARGC_MBR_NUM];
    unsigned short  blue[SHDISP_ARGC_STAGE_NUM][SHDISP_ARGC_MBR_NUM];
};

struct shdisp_igc_lut {
    unsigned char   r_data[SHDISP_IGC_LUT_ENTRIES];
    unsigned char   g_data[SHDISP_IGC_LUT_ENTRIES];
    unsigned char   b_data[SHDISP_IGC_LUT_ENTRIES];
};

enum {
    SHDISP_MAIN_DISP_OFF,
    SHDISP_MAIN_DISP_ON,
    NUM_SHDISP_MAIN_DISP_STATUS
};

enum {
    SHDISP_UPPER_UNIT_IS_NOT_CONNECTED,
    SHDISP_UPPER_UNIT_IS_CONNECTED,
    NUM_UPPER_UNIT_STATUS
};

enum {
    SHDISP_BDIC_IS_NOT_EXIST,
    SHDISP_BDIC_IS_EXIST,
    NUM_BDIC_EXIST_STATUS
};

enum {
    SHDISP_MAIN_BKL_MODE_OFF,
    SHDISP_MAIN_BKL_MODE_FIX,
    SHDISP_MAIN_BKL_MODE_AUTO,
    SHDISP_MAIN_BKL_MODE_DTV_OFF,
    SHDISP_MAIN_BKL_MODE_DTV_FIX,
    SHDISP_MAIN_BKL_MODE_DTV_AUTO,
    NUM_SHDISP_MAIN_BKL_MODE
};

#define SHDISP_MAIN_BKL_PARAM_OFF           (0)
#define SHDISP_MAIN_BKL_PARAM_WEAK          (1)
#define SHDISP_MAIN_BKL_PARAM_DEFAULT       (115)
#define SHDISP_MAIN_BKL_PARAM_MIN           (0)
#define SHDISP_MAIN_BKL_PARAM_MAX           (255)
#define SHDISP_MAIN_BKL_PARAM_MIN_AUTO      (2)
#define SHDISP_MAIN_BKL_PARAM_MAX_AUTO      (255)

enum {
    SHDISP_MAIN_DISP_ALS_RANGE_001 = 0,
    SHDISP_MAIN_DISP_ALS_RANGE_002,
    SHDISP_MAIN_DISP_ALS_RANGE_004,
    SHDISP_MAIN_DISP_ALS_RANGE_008,
    SHDISP_MAIN_DISP_ALS_RANGE_016,
    SHDISP_MAIN_DISP_ALS_RANGE_032,
    SHDISP_MAIN_DISP_ALS_RANGE_064,
    SHDISP_MAIN_DISP_ALS_RANGE_128,
    NUM_SHDISP_MAIN_DISP_ALS_RANGE
};

enum {
    SHDISP_TRI_LED_EXT_MODE_DISABLE,
    SHDISP_TRI_LED_EXT_MODE_ENABLE,
    NUM_SHDISP_TRI_LED_EXT_MODE
};

enum {
    SHDISP_KEY_BKL_OFF,
    SHDISP_KEY_BKL_NORMAL,
    SHDISP_KEY_BKL_BLINK,
    SHDISP_KEY_BKL_DIM,
    NUM_SHDISP_KEY_BKL_MODE
};

enum {
    SHDISP_KEY_BKL_LEFT,
    SHDISP_KEY_BKL_CENTER,
    SHDISP_KEY_BKL_RIGHT,
    NUM_SHDISP_KEY_SELECT
};

enum {
    SHDISP_TRI_LED_INTERVAL_NONE,
    SHDISP_TRI_LED_INTERVAL_1S,
    SHDISP_TRI_LED_INTERVAL_2S,
    SHDISP_TRI_LED_INTERVAL_3S,
    SHDISP_TRI_LED_INTERVAL_4S,
    SHDISP_TRI_LED_INTERVAL_5S,
    SHDISP_TRI_LED_INTERVAL_6S,
    SHDISP_TRI_LED_INTERVAL_7S,
    SHDISP_TRI_LED_INTERVAL_8S,
    SHDISP_TRI_LED_INTERVAL_9S,
    SHDISP_TRI_LED_INTERVAL_10S,
    SHDISP_TRI_LED_INTERVAL_11S,
    SHDISP_TRI_LED_INTERVAL_12S,
    SHDISP_TRI_LED_INTERVAL_13S,
    SHDISP_TRI_LED_INTERVAL_14S,
    SHDISP_TRI_LED_INTERVAL_15S,
    NUM_SHDISP_TRI_LED_INTERVAL
};

enum {
    SHDISP_KEY_BKL_INTERVAL_1S = 1,
    SHDISP_KEY_BKL_INTERVAL_2S,
    SHDISP_KEY_BKL_INTERVAL_3S,
    SHDISP_KEY_BKL_INTERVAL_4S,
    SHDISP_KEY_BKL_INTERVAL_5S,
    SHDISP_KEY_BKL_INTERVAL_6S,
    SHDISP_KEY_BKL_INTERVAL_7S,
    SHDISP_KEY_BKL_INTERVAL_8S,
    SHDISP_KEY_BKL_INTERVAL_9S,
    SHDISP_KEY_BKL_INTERVAL_10S,
    SHDISP_KEY_BKL_INTERVAL_11S,
    SHDISP_KEY_BKL_INTERVAL_12S,
    SHDISP_KEY_BKL_INTERVAL_13S,
    SHDISP_KEY_BKL_INTERVAL_14S,
    SHDISP_KEY_BKL_INTERVAL_15S,
    NUM_SHDISP_KEY_BKL_INTERVAL
};

enum {
    SHDISP_TRI_LED_MODE_OFF,
    SHDISP_TRI_LED_MODE_NORMAL,
    SHDISP_TRI_LED_MODE_BLINK,
    SHDISP_TRI_LED_MODE_FIREFLY,
    SHDISP_TRI_LED_MODE_HISPEED,
    SHDISP_TRI_LED_MODE_STANDARD,
    SHDISP_TRI_LED_MODE_BREATH,
    SHDISP_TRI_LED_MODE_LONG_BREATH,
    SHDISP_TRI_LED_MODE_WAVE,
    SHDISP_TRI_LED_MODE_FLASH,
    SHDISP_TRI_LED_MODE_AURORA,
    SHDISP_TRI_LED_MODE_RAINBOW,
    SHDISP_TRI_LED_MODE_EMOPATTERN,
    SHDISP_TRI_LED_MODE_PATTERN1,
    SHDISP_TRI_LED_MODE_PATTERN2,
    SHDISP_TRI_LED_MODE_ILLUMI_TRIPLE_COLOR,
    NUM_SHDISP_TRI_LED_MODE
};

enum {
    SHDISP_TRI_LED_ONTIME_TYPE0,
    SHDISP_TRI_LED_ONTIME_TYPE1,
    SHDISP_TRI_LED_ONTIME_TYPE2,
    SHDISP_TRI_LED_ONTIME_TYPE3,
    SHDISP_TRI_LED_ONTIME_TYPE4,
    SHDISP_TRI_LED_ONTIME_TYPE5,
    SHDISP_TRI_LED_ONTIME_TYPE6,
    SHDISP_TRI_LED_ONTIME_TYPE7,
    SHDISP_TRI_LED_ONTIME_TYPE8,
    SHDISP_TRI_LED_ONTIME_TYPE9,
    SHDISP_TRI_LED_ONTIME_TYPE10,
    SHDISP_TRI_LED_ONTIME_TYPE11,
    SHDISP_TRI_LED_ONTIME_TYPE12,
    SHDISP_TRI_LED_ONTIME_TYPE13,
    NUM_SHDISP_TRI_LED_ONTIME
};

enum {
    SHDISP_KEY_BKL_ONTIME_TYPE0,
    SHDISP_KEY_BKL_ONTIME_TYPE1,
    SHDISP_KEY_BKL_ONTIME_TYPE2,
    SHDISP_KEY_BKL_ONTIME_TYPE3,
    SHDISP_KEY_BKL_ONTIME_TYPE4,
    SHDISP_KEY_BKL_ONTIME_TYPE5,
    SHDISP_KEY_BKL_ONTIME_TYPE6,
    SHDISP_KEY_BKL_ONTIME_TYPE7,
    NUM_SHDISP_KEY_BKL_ONTIME
};

enum {
    SHDISP_TRI_LED_COUNT_NONE,
    SHDISP_TRI_LED_COUNT_1,
    SHDISP_TRI_LED_COUNT_2,
    SHDISP_TRI_LED_COUNT_3,
    SHDISP_TRI_LED_COUNT_4,
    SHDISP_TRI_LED_COUNT_5,
    SHDISP_TRI_LED_COUNT_6,
    SHDISP_TRI_LED_COUNT_7,
    NUM_SHDISP_TRI_LED_COUNT
};

enum {
    SHDISP_LEDC_ONCOUNT_REPEAT,
    SHDISP_LEDC_ONCOUNT_1SHOT,
    NUM_SHDISP_LEDC_ONCOUNT
};

enum {
    SHDISP_LEDC_IS_NOT_EXIST,
    SHDISP_LEDC_IS_EXIST,
    NUM_LEDC_EXIST_STATUS
};

struct shdisp_pharaoh_reg {
    unsigned char address;
    unsigned char size;
    unsigned char buf[32];
};

struct shdisp_ledc_rgb {
    unsigned int mode;
    unsigned int red[2];
    unsigned int green[2];
    unsigned int blue[2];
};

struct shdisp_diag_ledc_reg {
    unsigned char reg;
    unsigned char val;
};

struct shdisp_ledc_mono {
    unsigned int led;
    int led_mode;
    int on_count;
};

struct shdisp_diag_flicker_param {
    unsigned short  request;
    unsigned short  master_alpha;
    unsigned short  slave_alpha;
};

struct shdisp_diag_lcdc_reg {
    unsigned short reg;
    unsigned int   value;
};

struct shdisp_diag_lcdc_i2c {
    unsigned char  slave_addr;
    unsigned char  buf[256];
    unsigned short size;
    unsigned int   timeout;
};

struct shdisp_diag_ewb_tbl {
    unsigned char   valR[SHDISP_LCDC_EWB_TBL_SIZE];
    unsigned char   valG[SHDISP_LCDC_EWB_TBL_SIZE];
    unsigned char   valB[SHDISP_LCDC_EWB_TBL_SIZE];
};

struct shdisp_diag_set_ewb {
    unsigned short  valR;
    unsigned short  valG;
    unsigned short  valB;
};

struct shdisp_diag_read_ewb {
    unsigned char   level;
    unsigned short  valR;
    unsigned short  valG;
    unsigned short  valB;
};

#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY)
    #define SHDISP_PANEL_GMM_TBL_SIZE               (60)
    #define SHDISP_LCDDR_PHY_GMM_BUF_MAX            (SHDISP_PANEL_GMM_TBL_SIZE * 3)
    #define SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE       (9)
#elif defined(CONFIG_SHDISP_PANEL_HAYABUSA) || defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA)
    #define SHDISP_PANEL_GMM_TBL_SIZE               (60)
    #define SHDISP_LCDDR_PHY_GMM_BUF_MAX            (SHDISP_PANEL_GMM_TBL_SIZE * 3)
    #define SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE       (7)
    #define SHDISP_LCDDR_ADVANCED_GAMMA_SIZE        (30)
#elif defined(CONFIG_SHDISP_PANEL_SAZABI) || defined(USER_CONFIG_SHDISP_PANEL_SAZABI)
    #define SHDISP_PANEL_GMM_TBL_SIZE               (60)
    #define SHDISP_LCDDR_PHY_GMM_BUF_MAX            (SHDISP_PANEL_GMM_TBL_SIZE * 3)
    #define SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE       (12)
#elif defined(CONFIG_SHDISP_PANEL_ARIA) || defined(USER_CONFIG_SHDISP_PANEL_ARIA)
    #define SHDISP_PANEL_GMM_TBL_SIZE               (60)
    #define SHDISP_LCDDR_PHY_GMM_BUF_MAX            (SHDISP_PANEL_GMM_TBL_SIZE * 3)
    #define SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE       (11)
#elif defined(CONFIG_SHDISP_PANEL_CARIN) || defined(USER_CONFIG_SHDISP_PANEL_CARIN)
    #define SHDISP_PANEL_GMM_TBL_SIZE               (30)
    #define SHDISP_LCDDR_PHY_GMM_BUF_MAX            SHDISP_PANEL_GMM_TBL_SIZE
    #define SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE       (9)
#else   /* defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY) */
    #define SHDISP_PANEL_GMM_TBL_SIZE               (24)
    #error "SHDISP SWITCH NOTHING!!"
    #define SHDISP_LCDDR_PHY_GMM_BUF_MAX            (SHDISP_PANEL_GMM_TBL_SIZE * 3)
    #define SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE       (9)
#endif  /* defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY) */

#define SHDISP_LCDDR_GMM_SET_POINT_INI              (255)

struct shdisp_bdic_status {
    int power_status;
    unsigned int users;
};

struct shdisp_psals_status {
    int power_status;
    unsigned int als_users;
    int ps_um_status;
    int als_um_status;
};

struct shdisp_freq_params {
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA) || defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA)
    int internal_osc;
#else /* defined(CONFIG_SHDISP_PANEL_HAYABUSA) || defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA) */
    int internal_osc;
#endif /* defined(CONFIG_SHDISP_PANEL_HAYABUSA) || defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA) */
};

struct shdisp_lcddr_phy_gmm_reg {
#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY)
    unsigned char  status;
    unsigned short buf[SHDISP_LCDDR_PHY_GMM_BUF_MAX];
    unsigned char  applied_voltage[SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE];
    unsigned int   chksum;
#elif defined(CONFIG_SHDISP_PANEL_HAYABUSA) || defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA)
    unsigned char  status;
    unsigned short buf[SHDISP_LCDDR_PHY_GMM_BUF_MAX];
    unsigned char  applied_voltage[SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE];
    unsigned char  adv_gmm[SHDISP_LCDDR_ADVANCED_GAMMA_SIZE];
    unsigned int   chksum;
#elif defined(CONFIG_SHDISP_PANEL_SAZABI) || defined(USER_CONFIG_SHDISP_PANEL_SAZABI)
    unsigned char  status;
    unsigned short buf[SHDISP_LCDDR_PHY_GMM_BUF_MAX];
    unsigned char  applied_voltage[SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE];
    unsigned int   chksum;
#elif defined(CONFIG_SHDISP_PANEL_ARIA) || defined(USER_CONFIG_SHDISP_PANEL_ARIA)
    unsigned char  status;
    unsigned short buf[SHDISP_LCDDR_PHY_GMM_BUF_MAX];
    unsigned char  applied_voltage[SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE];
    unsigned int   chksum;
#elif defined(CONFIG_SHDISP_PANEL_CARIN) \
   || defined(USER_CONFIG_SHDISP_PANEL_CARIN)
    unsigned char  status;
    unsigned char  buf[SHDISP_LCDDR_PHY_GMM_BUF_MAX];
    unsigned char  applied_voltage[SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE];
    unsigned short chksum;
#else   /* defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY) */
    unsigned char  status;
    unsigned char  buf[SHDISP_LCDDR_PHY_GMM_BUF_MAX];
    unsigned char  applied_voltage[SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE];
    unsigned short chksum;
#endif  /* defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY) */
};

struct shdisp_diag_gamma_info {
#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY)
    unsigned short  gammaR[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned short  gammaG[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned short  gammaB[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned char   vgh;
    unsigned char   vgl;
    unsigned char   gvddp;
    unsigned char   gvddn;
    unsigned char   gvddp2;
    unsigned char   vgho;
    unsigned char   vglo;
    unsigned char   avddr;
    unsigned char   aveer;
#elif defined(CONFIG_SHDISP_PANEL_HAYABUSA) || defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA)
    unsigned short  gammaR[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned short  gammaG[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned short  gammaB[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned char   vgh;
    unsigned char   vgl;
    unsigned char   gvddp;
    unsigned char   gvddn;
    unsigned char   gvddp2;
    unsigned char   vgho;
    unsigned char   vglo;
    unsigned char   adv_gamma[SHDISP_LCDDR_ADVANCED_GAMMA_SIZE];
#elif defined(CONFIG_SHDISP_PANEL_SAZABI) || defined(USER_CONFIG_SHDISP_PANEL_SAZABI)
    unsigned short  gammaR[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned short  gammaG[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned short  gammaB[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned char   vgh;
    unsigned char   vgl;
    unsigned char   vgho1;
    unsigned char   vgho2;
    unsigned char   vglo1;
    unsigned char   vglo2;
    unsigned char   vgmp2;
    unsigned char   vgmp1;
    unsigned char   vgsp;
    unsigned char   vgmn2;
    unsigned char   vgmn1;
    unsigned char   vgsn;
#elif defined(CONFIG_SHDISP_PANEL_ARIA) || defined(USER_CONFIG_SHDISP_PANEL_ARIA)
    unsigned short  gammaR[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned short  gammaG[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned short  gammaB[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned char   dca;
    unsigned char   dcab;
    unsigned char   dcb;
    unsigned char   bta;
    unsigned char   vgh;
    unsigned char   vgl;
    unsigned char   vcl;
    unsigned char   gvddp;
    unsigned char   gvddn;
    unsigned char   vgho;
    unsigned char   vglo;
#elif defined(CONFIG_SHDISP_PANEL_CARIN) || defined(USER_CONFIG_SHDISP_PANEL_CARIN)
    unsigned char   gamma[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned char   vlm1;
    unsigned char   vlm2;
    unsigned char   vlm3;
    unsigned char   vc1;
    unsigned char   vc2;
    unsigned char   vc3;
    unsigned char   vpl;
    unsigned char   vnl;
    unsigned char   svss_svdd;
#else   /* defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY) */
    unsigned char   gammaR[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned char   gammaG[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned char   gammaB[SHDISP_PANEL_GMM_TBL_SIZE];
    unsigned char   vlm1;
    unsigned char   vlm2;
    unsigned char   vlm3;
    unsigned char   vc1;
    unsigned char   vc2;
    unsigned char   vc3;
    unsigned char   vpl;
    unsigned char   vnl;
    unsigned char   svss_svdd;
#endif  /* defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY) */
};

struct shdisp_diag_gamma {
#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY)
    unsigned char   level;
    unsigned short  gammaR_p;
    unsigned short  gammaR_n;
    unsigned short  gammaG_p;
    unsigned short  gammaG_n;
    unsigned short  gammaB_p;
    unsigned short  gammaB_n;
#elif defined(CONFIG_SHDISP_PANEL_HAYABUSA) || defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA)
    unsigned char   level;
#elif defined(CONFIG_SHDISP_PANEL_SAZABI) || defined(USER_CONFIG_SHDISP_PANEL_SAZABI)
    unsigned char   level;
#elif defined(CONFIG_SHDISP_PANEL_ARIA) || defined(USER_CONFIG_SHDISP_PANEL_ARIA)
    unsigned char   level;
    unsigned short  gammaR_p;
    unsigned short  gammaR_n;
    unsigned short  gammaG_p;
    unsigned short  gammaG_n;
    unsigned short  gammaB_p;
    unsigned short  gammaB_n;
#elif defined(CONFIG_SHDISP_PANEL_CARIN) || defined(USER_CONFIG_SHDISP_PANEL_CARIN)
    unsigned char   level;
    unsigned char   gamma_p;
    unsigned char   gamma_n;
#else   /* defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY) */
    unsigned char   level;
    unsigned char   gammaR_p;
    unsigned char   gammaR_n;
    unsigned char   gammaG_p;
    unsigned char   gammaG_n;
    unsigned char   gammaB_p;
    unsigned char   gammaB_n;
#endif  /* defined(CONFIG_SHDISP_PANEL_ANDY) || defined(USER_CONFIG_SHDISP_PANEL_ANDY) */
};

#endif /* SHDISP_CONTEXT_DEF_H */
