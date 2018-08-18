/* include/sharp/shdisp_define.h  (Display Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
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

#ifndef SHDISP_DEFINE_H
#define SHDISP_DEFINE_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#define SHDISP_TRV_NM2
#define SHDISP_LOWBKL

/* ------------------------------------------------------------------------- */
#if defined(CONFIG_ARCH_DECKARD_AL35) || defined(FEATURE_SH_MODEL_AL35)
#define SHDISP_AL

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_SYSFS_LED

#define USER_CONFIG_SHDISP_PANEL_SAZABI
#define SHDISP_PICADJ_USE_QDCM
#define SHDISP_MAIN_WIDTH  720
#define SHDISP_MAIN_HEIGHT 1280
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_DECKARD_AL37) || defined(FEATURE_SH_MODEL_AL37)
#define SHDISP_AL
#define SHDISP_MODEL_MID

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_KEY_LED
#define SHDISP_SYSFS_LED

#define USER_CONFIG_SHDISP_PANEL_SAZABI
#define SHDISP_MAIN_WIDTH  720
#define SHDISP_MAIN_HEIGHT 1280
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_PA35) || defined(FEATURE_SH_MODEL_PA35)
#define SHDISP_NOT_SUPPORT_DET
#define SHDISP_PA

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_SYSFS_LED

#define USER_CONFIG_SHDISP_PANEL_SAZABI
#define SHDISP_PICADJ_USE_QDCM
#define SHDISP_MAIN_WIDTH  720
#define SHDISP_MAIN_HEIGHT 1280
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_PA37) || defined(FEATURE_SH_MODEL_PA37)
#define SHDISP_PA
#define SHDISP_MODEL_MID

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_KEY_LED
#define SHDISP_SYSFS_LED

#define USER_CONFIG_SHDISP_PANEL_SAZABI
#define SHDISP_MAIN_WIDTH  720
#define SHDISP_MAIN_HEIGHT 1280
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_PA39) || defined(FEATURE_SH_MODEL_PA39)
#define SHDISP_PA
#define SHDISP_MODEL_MID

#define SHDISP_HAYABUSA_OSC_SWITCH
#define SHDISP_USE_QUALCOMM_RECOVERY

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_PICADJ_USE_QDCM
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_PE46) || defined(FEATURE_SH_MODEL_PE46)
#define SHDISP_PA

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED
#define SHDISP_SYSFS_LED

#define USER_CONFIG_SHDISP_PANEL_SAZABI
#define SHDISP_PICADJ_USE_QDCM
#define SHDISP_MAIN_WIDTH  720
#define SHDISP_MAIN_HEIGHT 1280
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_DECKARD_AL50) || defined(FEATURE_SH_MODEL_AL50)
#define SHDISP_AL
#define SHDISP_MODEL_MID

#define SHDISP_HAYABUSA_OSC_SWITCH
#define SHDISP_USE_QUALCOMM_RECOVERY

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_PICADJ_USE_QDCM
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#else  /* CONFIG_ARCH_XXX || FEATURE_SH_MODEL_XXX */
#define SHDISP_NOT_SUPPORT_DET
#define SHDISP_AL

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_SAZABI
#define SHDISP_MAIN_WIDTH  720
#define SHDISP_MAIN_HEIGHT 1280
/* ------------------------------------------------------------------------- */
#endif  /* CONFIG_ARCH_XXX || FEATURE_SH_MODEL_XXX */


#ifdef SHDISP_HAYABUSA_HF
#define SHDISP_FPS_HIGH_ENABLE
#endif  /* SHDISP_HAYABUSA_HF */



#endif /* SHDISP_DEFINE_H */
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
