/* kernel/include/linux/input/gp2ap030.h - GP2AP030*00F v1.0.2ambient light sensor and proximity sensor driver
 *
 * Copyright (C) 2012 Sharp Corporation
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

#ifndef __GP2AP030A_H__
#define __GP2AP030A_H__

// Reg.
#define REG_ADR_00	0x00	// Read & Write
#define REG_ADR_01	0x01	// Read & Write
#define REG_ADR_02	0x02	// Read & Write
#define REG_ADR_03	0x03	// Read & Write
#define REG_ADR_04	0x04	// Read & Write
#define REG_ADR_05	0x05	// Read & Write
#define REG_ADR_06	0x06	// Read & Write
#define REG_ADR_07	0x07	// Read & Write
#define REG_ADR_08	0x08	// Read & Write
#define REG_ADR_09	0x09	// Read & Write
#define REG_ADR_0A	0x0A	// Read & Write
#define REG_ADR_0B	0x0B	// Read & Write
#define REG_ADR_0C	0x0C	// Read  Only
#define REG_ADR_0D	0x0D	// Read  Only
#define REG_ADR_0E	0x0E	// Read  Only
#define REG_ADR_0F	0x0F	// Read  Only
#define REG_ADR_10	0x10	// Read  Only
#define REG_ADR_11	0x11	// Read  Only
#define REG_ADR_16	0x16	// Read  Only

/* event code */
#define ABS_WAKE				( ABS_BRAKE )
#define ABS_CONTROL_REPORT		( ABS_THROTTLE )
#define ABS_LUX_REPORT			( ABS_MISC )
#define ABS_DISTANCE_REPORT		( ABS_DISTANCE )

#define usleep(arg) usleep_range(arg, arg)

/* platform data */
struct gp2ap030_platform_data
{
	int		gpio ;
} ;

struct gp2ap030_ado_tbl {
    unsigned long range_low;
    unsigned long range_high;
    unsigned long param_a;
    long param_b;
};

static const struct gp2ap030_ado_tbl gp2ap030_ado_tbl[5] = {
    {    0,      4,   232,     -29},
    {    4,     41,   247,    -123},
    {   41,    372,   270,   -1291},
    {  372,   4558,   203,   24306},
    { 4558,  65536,   359, -827726}
};

#endif
