/* drivers/leds/leds-an32180a.h
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

#ifndef LEDS_AN32180A_H
#define LEDS_AN32180A_H

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    LEDS_STR,
    LEDS_SET,
    LEDS_CLR,
    LEDS_RMW,
    LEDS_STRM,
    LEDS_WAIT
};

#define LEDS_REG_RST		(0x01)
#define LEDS_REG_POWERCNT	(0x02)

#define LEDS_REG_OPTION		(0x04)
#define LEDS_REG_MTXON		(0x05)
#define LEDS_REG_PWMEN1		(0x06)
#define LEDS_REG_PWMEN2		(0x07)
#define LEDS_REG_MDLEN1		(0x08)
#define LEDS_REG_MDLEN2		(0x09)
#define LEDS_REG_MDLMODE2	(0x0A)
#define LEDS_REG_MDLCOM		(0x0B)
#define LEDS_REG_THOLD		(0x0C)
#define LEDS_REG_XCONTST	(0x0D)
#define LEDS_REG_YCONTST	(0x0E)
#define LEDS_REG_SLPTIME	(0x0F)

#define LEDS_REG_DTA1		(0x10)
#define LEDS_REG_DTA2		(0x11)
#define LEDS_REG_DTA3		(0x12)
#define LEDS_REG_DTA4		(0x13)

#define LEDS_REG_DTB1		(0x14)
#define LEDS_REG_DTB2		(0x15)
#define LEDS_REG_DTB3		(0x16)
#define LEDS_REG_DTB4		(0x17)

#define LEDS_REG_DTC1		(0x18)
#define LEDS_REG_DTC2		(0x19)
#define LEDS_REG_DTC3		(0x1A)
#define LEDS_REG_DTC4		(0x1B)

#define LEDS_REG_DTD1		(0x1C)
#define LEDS_REG_DTD2		(0x1D)
#define LEDS_REG_DTD3		(0x1E)
#define LEDS_REG_DTD4		(0x1F)

#define LEDS_REG_A1			(0x20)
#define LEDS_REG_A2			(0x21)
#define LEDS_REG_A3			(0x22)
#define LEDS_REG_A4			(0x23)

#define LEDS_REG_B1			(0x24)
#define LEDS_REG_B2			(0x25)
#define LEDS_REG_B3			(0x26)
#define LEDS_REG_B4			(0x27)

#define LEDS_REG_C1			(0x28)
#define LEDS_REG_C2			(0x29)
#define LEDS_REG_C3			(0x2A)
#define LEDS_REG_C4			(0x2B)

#define LEDS_REG_D1			(0x2C)
#define LEDS_REG_D2			(0x2D)
#define LEDS_REG_D3			(0x2E)
#define LEDS_REG_D4			(0x2F)

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
typedef struct {
	unsigned char	addr;
	unsigned char	data;
	unsigned long	wait;
} leds_regsetting_t;

typedef struct {
	int				handset_color;
	unsigned char	imax;
	unsigned char	brt_b;
	unsigned char	brt_c;
	unsigned char	brt_d;
} leds_clrvari_t;

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static const unsigned char leds_reg_get_dtaddr[3][3] = {
	{ LEDS_REG_DTB1, LEDS_REG_DTC1, LEDS_REG_DTD1},
	{ LEDS_REG_DTB2, LEDS_REG_DTC2, LEDS_REG_DTD2},
	{ LEDS_REG_DTB3, LEDS_REG_DTC3, LEDS_REG_DTD3}
};

const leds_regsetting_t reg_rst = { LEDS_REG_RST, 0x01, 0 };

const leds_regsetting_t reg_option = { LEDS_REG_OPTION, 0x04, 0 };

const leds_regsetting_t reg_pwmen1 = { LEDS_REG_PWMEN1, 0xFF, 0 };
const leds_regsetting_t reg_pwmen2 = { LEDS_REG_PWMEN2, 0xFF, 0 };

const leds_regsetting_t reg_mdlen1 = { LEDS_REG_MDLEN1, 0xFF, 0 };
const leds_regsetting_t reg_mdlen2 = { LEDS_REG_MDLEN2, 0xFF, 0 };

const leds_regsetting_t reg_thold = { LEDS_REG_THOLD, 0x01, 0 };

leds_regsetting_t reg_mtxon  = { LEDS_REG_MTXON, 0x1F, 0 };

leds_regsetting_t reg_b1 = { LEDS_REG_B1, 0x60, 0 };
leds_regsetting_t reg_b2 = { LEDS_REG_B2, 0x60, 0 };
leds_regsetting_t reg_b3 = { LEDS_REG_B3, 0x60, 0 };

leds_regsetting_t reg_c1 = { LEDS_REG_C1, 0x60, 0 };
leds_regsetting_t reg_c2 = { LEDS_REG_C2, 0x60, 0 };
leds_regsetting_t reg_c3 = { LEDS_REG_C3, 0x60, 0 };

leds_regsetting_t reg_d1 = { LEDS_REG_D1, 0x60, 0 };
leds_regsetting_t reg_d2 = { LEDS_REG_D2, 0x60, 0 };
leds_regsetting_t reg_d3 = { LEDS_REG_D3, 0x60, 0 };

static const leds_regsetting_t* leds_an32180_init1[] = {
	&reg_rst,
	&reg_mtxon,
	&reg_pwmen1,
	&reg_pwmen2,
	&reg_b1,
	&reg_b2,
	&reg_b3,

	&reg_c1,
	&reg_c2,
	&reg_c3,

	&reg_d1,
	&reg_d2,
	&reg_d3
};

static const leds_regsetting_t* leds_an32180_blink_init1[] = {
	&reg_rst,
	&reg_option,
	&reg_mtxon,
	&reg_pwmen1,
	&reg_pwmen2,
	&reg_mdlen1,
	&reg_mdlen2,
	&reg_thold,
	&reg_b1,
	&reg_b2,
	&reg_b3,

	&reg_c1,
	&reg_c2,
	&reg_c3,

	&reg_d1,
	&reg_d2,
	&reg_d3
};

static const leds_clrvari_t leds_an32180_clrvari_tbl[] = {
#if defined(CONFIG_ARCH_DECKARD_AL50)
	{ 0x01,		0x17,	0xA0,	0x40,	0xD0 },
	{ 0x04,		0x1B,	0xA0,	0x10,	0x20 },
	{ 0x02,		0x19,	0x40,	0x30,	0xD0 },
#else	/* CONFIG_ARCH_DECKARD_AL50 */
	{ 0x01,		0x17,	0xA0,	0x40,	0xD0 },
	{ 0x05,		0x15,	0x50,	0x10,	0xD0 },
	{ 0x06,		0x1F,	0x30,	0x10,	0x80 },
	{ 0x04,		0x17,	0xF0,	0x20,	0x60 },
	{ 0x02,		0x1B,	0x20,	0x30,	0xB0 },
#endif	/* CONFIG_ARCH_DECKARD_AL50 */
};

#endif /* LEDS_AN32180A_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
