/* drivers/sharp/shdisp/data/shdisp_bl71y8_led_senior.h  (Display Driver)
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

#include "shdisp_bl71y8_led_default.h"
/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_BL71Y8_LED_SENIOR_H
#define SHDISP_BL71Y8_LED_SENIOR_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_KEY_LED_COLOR_TBL_NUM                (27)

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static const struct shdisp_bdic_key_led_color_index shdisp_key_led_index_tbl[SHDISP_KEY_LED_COLOR_TBL_NUM] = {
/* 0 : SHDISP_KEY_BKL_OFF */
/* 1 : SHDISP_KEY_BKL_NORMAL or SHDISP_KEY_BKL_DIM */
/* 2 : SHDISP_KEY_BKL_BLINK */
    { 0, 0, 0, 0  },
    { 0, 0, 1, 1  },
    { 0, 1, 0, 2  },
    { 0, 1, 1, 3  },
    { 1, 0, 0, 4  },
    { 1, 0, 1, 5  },
    { 1, 1, 0, 6  },
    { 1, 1, 1, 7  },    /* index  > 7 ;  timer_st = 1 ; en_lposc =1 */
    { 0, 0, 2, 8  },
    { 0, 2, 0, 9  },
    { 0, 2, 2, 10 },
    { 2, 0, 0, 11 },
    { 2, 0, 2, 12 },
    { 2, 2, 0, 13 },
    { 2, 2, 2, 14 },
    { 0, 1, 2, 15 },
    { 0, 2, 1, 16 },
    { 1, 0, 2, 17 },
    { 1, 2, 0, 18 },
    { 2, 0, 1, 19 },
    { 2, 1, 0, 20 },
    { 1, 1, 2, 21 },
    { 1, 2, 1, 22 },
    { 1, 2, 2, 23 },
    { 2, 1, 1, 24 },
    { 2, 1, 2, 25 },
    { 2, 2, 1, 26 },
};

static const unsigned char shdisp_key_led_tbl[NUM_SHDISP_KEY_SELECT][NUM_SHDISP_KEY_BKL_MODE] = {
    { 0x00, 0x0C, 0x0C, 0x06 },    /* BKL_LEFT       (TEL  :GREEN)    { OFF, NORMAL, BLINK, DIM }, */
    { 0x00, 0x1A, 0x1A, 0x0D },    /* BKL_CENTER     (HOME :WHITE)    { OFF, NORMAL, BLINK, DIM }, */
    { 0x00, 0x1E, 0x1E, 0x0F }     /* KEY_BKL_RIGHT  (MAIL :BLUE )    { OFF, NORMAL, BLINK, DIM }, */
/* OFF      :  SHDISP_KEY_BKL_OFF */
/* NORMAL   :  SHDISP_KEY_BKL_NORMAL */
/* BLINK    :  SHDISP_KEY_BKL_BLINK */
/* DIM      :  SHDISP_KEY_BKL_DIM */
};
#endif /* SHDISP_BL71Y8_LED_SENIOR_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
