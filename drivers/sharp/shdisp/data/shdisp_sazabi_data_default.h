/* drivers/sharp/shdisp/data/shdisp_sazabi_data_default.h  (Display Driver)
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

#ifndef SHDISP_SAZABI_DATA_H
#define SHDISP_SAZABI_DATA_H

/*************************************************/
/*   switch to page                              */
/*************************************************/
static char switchtopage0_payloads[] = {
    0xF0,
    0x46,0x23,0x11,0x01,0x00,
};
static char switchtopage1_payloads[] = {
    0xF0,
    0x46,0x23,0x11,0x01,0x01
};

static struct shdisp_dsi_cmd_desc switchtopage0_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(switchtopage0_payloads),          switchtopage0_payloads,          0,0}
};
static struct shdisp_dsi_cmd_desc switchtopage1_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(switchtopage1_payloads),          switchtopage1_payloads,          0,0}
};

/*************************************************/
/*   adjust voltage setting                      */
/*************************************************/
static char tmp_voltage_settings_adrB1_payloads[2] = {0, 0};
static char tmp_voltage_settings_adrB2_payloads[2] = {0, 0};
static char tmp_voltage_settings_adrB3_payloads[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static char tmp_voltage_settings_adrB4_payloads[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static char tmp_voltage_settings_adrC3_payloads[4] = {0, 0, 0, 0};
static char tmp_voltage_settings_adrC4_payloads[4] = {0, 0, 0, 0};

static char voltage_settings_adrB1_payloads[] = {
    0xB1,
    0x15
};
static char voltage_settings_adrB2_payloads[] = {
    0xB2,
    0x15
};
static char voltage_settings_adrB3_payloads[] = {
    0xB3,
    0x05,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x0C,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00
};
static char voltage_settings_adrB4_payloads[] = {
    0xB4,
    0x0F,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x08,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00

};
static char voltage_settings_adrC3_payloads[] = {
    0xC3,
    0x00,0x90,0x00
};
static char voltage_settings_adrC4_payloads[] = {
    0xC4,
    0x00,0x90,0x00
};

static char adjusted_voltage_settings_adrB1_payloads[2] = {
    0xB1,
    0x00
};
static char adjusted_voltage_settings_adrB2_payloads[] = {
    0xB2,
    0x00
};
static char adjusted_voltage_settings_adrB3_payloads[] = {
    0xB3,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00
};
static char adjusted_voltage_settings_adrB4_payloads[] = {
    0xB4,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00

};
static char adjusted_voltage_settings_adrC3_payloads[] = {
    0xC3,
    0x00,0x00,0x00
};
static char adjusted_voltage_settings_adrC4_payloads[] = {
    0xC4,
    0x00,0x00,0x00
};

static struct shdisp_dsi_cmd_desc voltage_setting_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(switchtopage1_payloads),          switchtopage1_payloads,          0,0},
    { SHDISP_DTYPE_GEN_WRITE2,  sizeof(voltage_settings_adrB1_payloads), voltage_settings_adrB1_payloads, 0,0},
    { SHDISP_DTYPE_GEN_WRITE2,  sizeof(voltage_settings_adrB2_payloads), voltage_settings_adrB2_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(voltage_settings_adrB3_payloads), voltage_settings_adrB3_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(voltage_settings_adrB4_payloads), voltage_settings_adrB4_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(voltage_settings_adrC3_payloads), voltage_settings_adrC3_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(voltage_settings_adrC4_payloads), voltage_settings_adrC4_payloads, 0,0},
};

static struct shdisp_dsi_cmd_desc adjusted_voltage_setting_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(switchtopage1_payloads),          switchtopage1_payloads,          0,0},
    { SHDISP_DTYPE_GEN_WRITE2,  sizeof(adjusted_voltage_settings_adrB1_payloads), adjusted_voltage_settings_adrB1_payloads, 0,0},
    { SHDISP_DTYPE_GEN_WRITE2,  sizeof(adjusted_voltage_settings_adrB2_payloads), adjusted_voltage_settings_adrB2_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(adjusted_voltage_settings_adrB3_payloads), adjusted_voltage_settings_adrB3_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(adjusted_voltage_settings_adrB4_payloads), adjusted_voltage_settings_adrB4_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(adjusted_voltage_settings_adrC3_payloads), adjusted_voltage_settings_adrC3_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(adjusted_voltage_settings_adrC4_payloads), adjusted_voltage_settings_adrC4_payloads, 0,0},
};

#define VGH_ADR           (1)
#define VGL_ADR           (1)
#define VGHO1_ADR         (1)
#define VGHO2_ADR         (9)
#define VGLO1_ADR         (1)
#define VGLO2_ADR         (9)
#define VGMP2_ADR         (1)
#define VGMP1_ADR         (2)
#define VGSP_ADR          (3)
#define VGMN2_ADR         (1)
#define VGMN1_ADR         (2)
#define VGSN_ADR          (3)
static char *ptr_vgh   = &adjusted_voltage_settings_adrB1_payloads[VGH_ADR];
static char *ptr_vgl   = &adjusted_voltage_settings_adrB2_payloads[VGL_ADR];
static char *ptr_vgho1 = &adjusted_voltage_settings_adrB3_payloads[VGHO1_ADR];
static char *ptr_vgho2 = &adjusted_voltage_settings_adrB3_payloads[VGHO2_ADR];
static char *ptr_vglo1 = &adjusted_voltage_settings_adrB4_payloads[VGLO1_ADR];
static char *ptr_vglo2 = &adjusted_voltage_settings_adrB4_payloads[VGLO2_ADR];
static char *ptr_vgmp2 = &adjusted_voltage_settings_adrC3_payloads[VGMP2_ADR];
static char *ptr_vgmp1 = &adjusted_voltage_settings_adrC3_payloads[VGMP1_ADR];
static char *ptr_vgsp  = &adjusted_voltage_settings_adrC3_payloads[VGSP_ADR];
static char *ptr_vgmn2 = &adjusted_voltage_settings_adrC4_payloads[VGMN2_ADR];
static char *ptr_vgmn1 = &adjusted_voltage_settings_adrC4_payloads[VGMN1_ADR];
static char *ptr_vgsn  = &adjusted_voltage_settings_adrC4_payloads[VGSN_ADR];

/*************************************************/
/*   adjust gmm setting                          */
/*************************************************/
static char initial_setting_page1_gmm_D1_payloads[] = {
    0xD1,
    0x00,0x00,0x00,0x00,0x00,
    0x2B,0x00,0x3F,0x00,0x5E,
    0x00,0x72,0x00,0x83,0x00,
    0x94,0x00,0xA1,0x00,0xD1,
    0x00,0xF7,0x01,0x2D,0x01,
    0x58,0x01,0x9C,0x01,0xD6,
    0x01,0xD8,0x02,0x16,0x02,
    0x63,0x02,0x94,0x02,0xD3,
    0x02,0xFC,0x03,0x2E,0x03,
    0x3D,0x03,0x4D,0x03,0x5E,
    0x03,0x71,0x03,0x86,0x03,
    0x9E,0x03,0xB8,0x03,0xC7
};

static char initial_setting_page1_gmm_D2_payloads[] = {
    0xD2,
    0x00,0x00,0x00,0x00,0x00,
    0x2B,0x00,0x3F,0x00,0x5E,
    0x00,0x72,0x00,0x83,0x00,
    0x94,0x00,0xA1,0x00,0xD1,
    0x00,0xF7,0x01,0x2D,0x01,
    0x58,0x01,0x9C,0x01,0xD6,
    0x01,0xD8,0x02,0x16,0x02,
    0x63,0x02,0x94,0x02,0xD3,
    0x02,0xFC,0x03,0x2E,0x03,
    0x3D,0x03,0x4D,0x03,0x5E,
    0x03,0x71,0x03,0x86,0x03,
    0x9E,0x03,0xB8,0x03,0xC7
};

static char initial_setting_page1_gmm_D3_payloads[] = {
    0xD3,
    0x00,0x00,0x00,0x00,0x00,
    0x2B,0x00,0x3F,0x00,0x5E,
    0x00,0x72,0x00,0x83,0x00,
    0x94,0x00,0xA1,0x00,0xD1,
    0x00,0xF7,0x01,0x2D,0x01,
    0x58,0x01,0x9C,0x01,0xD6,
    0x01,0xD8,0x02,0x16,0x02,
    0x63,0x02,0x94,0x02,0xD3,
    0x02,0xFC,0x03,0x2E,0x03,
    0x3D,0x03,0x4D,0x03,0x5E,
    0x03,0x71,0x03,0x86,0x03,
    0x9E,0x03,0xB8,0x03,0xC7
};

static char initial_setting_page1_gmm_D4_payloads[] = {
    0xD4,
    0x00,0x0E,0x00,0x19,0x00,
    0x45,0x00,0x58,0x00,0x76,
    0x00,0x88,0x00,0x99,0x00,
    0xA8,0x00,0xB4,0x00,0xE0,
    0x01,0x00,0x01,0x35,0x01,
    0x5E,0x01,0x9F,0x01,0xD6,
    0x01,0xD8,0x02,0x12,0x02,
    0x5A,0x02,0x89,0x02,0xC5,
    0x02,0xEE,0x03,0x1F,0x03,
    0x2D,0x03,0x3D,0x03,0x4E,
    0x03,0x60,0x03,0x75,0x03,
    0x8D,0x03,0xA7,0x03,0xB5
};

static char initial_setting_page1_gmm_D5_payloads[] = {
    0xD5,
    0x00,0x0E,0x00,0x19,0x00,
    0x45,0x00,0x58,0x00,0x76,
    0x00,0x88,0x00,0x99,0x00,
    0xA8,0x00,0xB4,0x00,0xE0,
    0x01,0x00,0x01,0x35,0x01,
    0x5E,0x01,0x9F,0x01,0xD6,
    0x01,0xD8,0x02,0x12,0x02,
    0x5A,0x02,0x89,0x02,0xC5,
    0x02,0xEE,0x03,0x1F,0x03,
    0x2D,0x03,0x3D,0x03,0x4E,
    0x03,0x60,0x03,0x75,0x03,
    0x8D,0x03,0xA7,0x03,0xB5
};

static char initial_setting_page1_gmm_D6_payloads[] = {
    0xD6,
    0x00,0x0E,0x00,0x19,0x00,
    0x45,0x00,0x58,0x00,0x76,
    0x00,0x88,0x00,0x99,0x00,
    0xA8,0x00,0xB4,0x00,0xE0,
    0x01,0x00,0x01,0x35,0x01,
    0x5E,0x01,0x9F,0x01,0xD6,
    0x01,0xD8,0x02,0x12,0x02,
    0x5A,0x02,0x89,0x02,0xC5,
    0x02,0xEE,0x03,0x1F,0x03,
    0x2D,0x03,0x3D,0x03,0x4E,
    0x03,0x60,0x03,0x75,0x03,
    0x8D,0x03,0xA7,0x03,0xB5
};

static struct shdisp_dsi_cmd_desc initial_setting_page1_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D1_payloads), initial_setting_page1_gmm_D1_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D2_payloads), initial_setting_page1_gmm_D2_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D3_payloads), initial_setting_page1_gmm_D3_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D4_payloads), initial_setting_page1_gmm_D4_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D5_payloads), initial_setting_page1_gmm_D5_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D6_payloads), initial_setting_page1_gmm_D6_payloads,0,0},
};

static struct shdisp_dsi_cmd_desc initial_setting_D1_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D1_payloads), initial_setting_page1_gmm_D1_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc initial_setting_D2_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D2_payloads), initial_setting_page1_gmm_D2_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc initial_setting_D3_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D3_payloads), initial_setting_page1_gmm_D3_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc initial_setting_D4_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D4_payloads), initial_setting_page1_gmm_D4_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc initial_setting_D5_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D5_payloads), initial_setting_page1_gmm_D5_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc initial_setting_D6_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(initial_setting_page1_gmm_D6_payloads), initial_setting_page1_gmm_D6_payloads,0,0},
};

static char adjusted_setting_page1_gmm_D1_payloads[] = {
    0xD1,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00
};

static char adjusted_setting_page1_gmm_D2_payloads[] = {
    0xD2,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00
};


static char adjusted_setting_page1_gmm_D3_payloads[] = {
    0xD3,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00
};


static char adjusted_setting_page1_gmm_D4_payloads[] = {
    0xD4,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00
};


static char adjusted_setting_page1_gmm_D5_payloads[] = {
    0xD5,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00
};


static char adjusted_setting_page1_gmm_D6_payloads[] = {
    0xD6,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00
};

static struct shdisp_dsi_cmd_desc adjusted_setting_page1_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D1_payloads), adjusted_setting_page1_gmm_D1_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D2_payloads), adjusted_setting_page1_gmm_D2_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D3_payloads), adjusted_setting_page1_gmm_D3_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D4_payloads), adjusted_setting_page1_gmm_D4_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D5_payloads), adjusted_setting_page1_gmm_D5_payloads,0,0},
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D6_payloads), adjusted_setting_page1_gmm_D6_payloads,0,0},
};

static struct shdisp_dsi_cmd_desc adjusted_setting_D1_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D1_payloads), adjusted_setting_page1_gmm_D1_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc adjusted_setting_D2_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D2_payloads), adjusted_setting_page1_gmm_D2_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc adjusted_setting_D3_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D3_payloads), adjusted_setting_page1_gmm_D3_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc adjusted_setting_D4_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D4_payloads), adjusted_setting_page1_gmm_D4_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc adjusted_setting_D5_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D5_payloads), adjusted_setting_page1_gmm_D5_payloads,0,0},
};
static struct shdisp_dsi_cmd_desc adjusted_setting_D6_gmm_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE, sizeof(adjusted_setting_page1_gmm_D6_payloads), adjusted_setting_page1_gmm_D6_payloads,0,0},
};

/*************************************************/
/*   initial settings                            */
/*************************************************/
static char voltage_settings_adrB5_payloads[] = {
    0xB5,
    0x00,0x00
};
static char voltage_settings_adrB7_payloads[] = {
    0xB7,
    0x44,0x00,0x00,0x00
};
static char voltage_settings_adrB8_payloads[] = {
    0xB8,
    0x54,0x00,0x00,0x00
};
static char voltage_settings_adrB9_payloads[] = {
    0xB9,
    0x00
};

static char voltage_settings_adrC6_unadjust_payloads[] = {
    0xC6,
    0x00,0x00,0x00,0x13
};

static char voltage_settings_adrC6_adjusted_payloads[] = {
    0xC6,
    0x00,0x00,0x00,0x00
};

static struct shdisp_dsi_cmd_desc adjusted_setting_vcom_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(voltage_settings_adrC6_adjusted_payloads), 
                                                                voltage_settings_adrC6_adjusted_payloads, 0,0},
};

static char page1_te_connecter_settings_payloads[] = {
    0xCB,
    0x00,0x00,0x00,0x00,0x0C,
    0x08,0xCC,0x20,0x20,0x00
};
static char initial_setting_page0_adrB2_payloads[] = {
    0xB2,
    0x01,0x9B,0x31,0x42,0x07,
    0xFF,0x06
};
static char initial_setting_page0_adrB3_payloads[] = {
    0xB3,
    0xE9
};
static char initial_setting_page0_adrBA_payloads[] = {
    0xBA,
    0x0F,0x90,0x30,0x59
};
static char initial_setting_page0_adrBB_payloads[] = {
    0xBB,
    0x05,0x69,0x09,0x02
};
static char initial_setting_page0_adrBC_payloads[] = {
    0xBC,
    0x00,0x80
};
static char initial_setting_page0_adrBD_payloads[] = {
    0xBD,
    0xC6,0x00,0x00,0x20,0x00,
    0x30
};
static char initial_setting_page0_adrBE_payloads[] = {
    0xBE,
    0x01,0x04,0x0F,0xFF
};
static char initial_setting_page0_adrBF_payloads[] = {
    0xBF,
    0x0A
};
static char initial_setting_page0_adrC0_payloads[] = {
    0xC0,
    0x02,0x80
};
static char initial_setting_page0_adrC1_payloads[] = {
    0xC1,
    0x10,0x00
};
static char initial_setting_page0_adrC2_payloads[] = {
    0xC2,
    0x18,0x07
};
static char initial_setting_page0_adrC3_payloads[] = {
    0xC3,
    0x00,0x18,0x18,0x18
};
static char initial_setting_page0_adrC5_payloads[] = {
    0xC5,
    0x01,0x01,0xC0,0x00
};
static char initial_setting_page0_adrC7_payloads[] = {
    0xC7,
    0x3D,0xFA,0x08,0x55
};
static char initial_setting_page0_adrC9_payloads[] = {
    0xC9,
    0x09,0x01,0x2A,0x02,0x04,
    0x08,0x00,0x09,0x40,0x04,
    0x02,0x00,0x07,0x01,0x90,
    0xE0,0x01,0x03,0x03,0x03,
    0x01,0x08,0x01,0x90,0xE0,
    0x00,0x04,0x02,0x00,0x01,
    0x08,0x11,0x80,0x20,0x01,
    0x01,0x01,0x01,0x01,0x08,
    0x11,0x80,0x20,0x00,0x02,
    0xE4,0xEA,0xE4,0xE4,0xE4,
    0xE4,0x3F,0xCC,0xCC,0xCC,
    0x8A,0xCC,0x00,0x55,0x00,
    0x00,0x00,0x00,0x00,0x02,
    0x15,0x16,0x02,0x16,0x16,
    0x16
};
static char initial_setting_page0_adrCA_payloads[] = {
    0xCA,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00
};
static char initial_setting_page0_adrCB_payloads[] = {
    0xCB,
    0x00,0x02,0x00,0x11,0x80,
    0x20,0x66,0x00,0x02,0x11,
    0x80,0x80,0x66,0x00,0x02,
    0x11,0x80,0x80,0x06,0x05,
    0x03,0x00,0x05,0x05,0x00,
    0x04,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x01,
    0x01,0x01,0x66,0x01,0x03,
    0x66,0x01,0x03,0x00,0x12,
    0x0F
};
static char initial_setting_page0_adrCC_payloads[] = {
    0xCC,
    0x30,0x01,0x81,0x00,0x00,
    0x11,0x80,0x20,0x01,0x03,
    0x00,0x20,0x00,0x00,0x00,
    0x00,0x00,0x00,0x80,0x01,
    0x00,0x02,0x00,0x00
};
static char initial_setting_page0_adrCD_payloads[] = {
    0xCD,
    0x12,0x10,0x00,0x00,0x00,
    0x00,0x10,0x10,0x08,0x08,
    0x31,0x00,0x00,0x10,0x12,
    0x10,0x81,0x40,0x00,0x00,
    0x11
};
static char initial_setting_page0_adrCE_payloads[] = {
    0xCE,
    0x54,0x55,0x58,0x59,0x00,
    0x00,0x00,0x00,0x08,0x08,
    0x04,0x05,0x06,0x07,0x6E,
    0x6F,0x6A,0x6B,0x6C,0x6D,
    0x68,0x69,0x08,0x08,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x50,0x51,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x00
};
static char initial_setting_page0_adrEF_payloads[] = {
    0xEF,
    0x00,0x51,0x2C,0x74,0x02,
    0x18,0x84,0x84,0x70
};
static char initial_setting_page0_adrEE_payloads[] = {
    0xEE,
    0x71,
};

static char initial_setting_page0_adrFA_payloads[] = {
    0xFA,
    0x00,0x00,0x64,0x00,0x00
};
static struct shdisp_dsi_cmd_desc initial_setting_cmds[] = {
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(switchtopage1_payloads),               switchtopage1_payloads,               0,0},
    { SHDISP_DTYPE_GEN_WRITE2,  sizeof(tmp_voltage_settings_adrB1_payloads),  tmp_voltage_settings_adrB1_payloads,  0,0},
    { SHDISP_DTYPE_GEN_WRITE2,  sizeof(tmp_voltage_settings_adrB2_payloads),  tmp_voltage_settings_adrB2_payloads,  0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(tmp_voltage_settings_adrB3_payloads),  tmp_voltage_settings_adrB3_payloads,  0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(tmp_voltage_settings_adrB4_payloads),  tmp_voltage_settings_adrB4_payloads,  0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(voltage_settings_adrB5_payloads),      voltage_settings_adrB5_payloads,      0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(voltage_settings_adrB7_payloads),      voltage_settings_adrB7_payloads,      0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(voltage_settings_adrB8_payloads),      voltage_settings_adrB8_payloads,      0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(voltage_settings_adrB9_payloads),      voltage_settings_adrB9_payloads,      0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(tmp_voltage_settings_adrC3_payloads),  tmp_voltage_settings_adrC3_payloads,  0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(tmp_voltage_settings_adrC4_payloads),  tmp_voltage_settings_adrC4_payloads,  0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(page1_te_connecter_settings_payloads), page1_te_connecter_settings_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(switchtopage0_payloads),               switchtopage0_payloads,               0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrB2_payloads), initial_setting_page0_adrB2_payloads, 0,0},
    { SHDISP_DTYPE_GEN_WRITE2,  sizeof(initial_setting_page0_adrB3_payloads), initial_setting_page0_adrB3_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrBA_payloads), initial_setting_page0_adrBA_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrBB_payloads), initial_setting_page0_adrBB_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrBC_payloads), initial_setting_page0_adrBC_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrBD_payloads), initial_setting_page0_adrBD_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrBE_payloads), initial_setting_page0_adrBE_payloads, 0,0},
    { SHDISP_DTYPE_GEN_WRITE2,  sizeof(initial_setting_page0_adrBF_payloads), initial_setting_page0_adrBF_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrC0_payloads), initial_setting_page0_adrC0_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrC1_payloads), initial_setting_page0_adrC1_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrC2_payloads), initial_setting_page0_adrC2_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrC3_payloads), initial_setting_page0_adrC3_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrC5_payloads), initial_setting_page0_adrC5_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrC7_payloads), initial_setting_page0_adrC7_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrC9_payloads), initial_setting_page0_adrC9_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrCA_payloads), initial_setting_page0_adrCA_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrCB_payloads), initial_setting_page0_adrCB_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrCC_payloads), initial_setting_page0_adrCC_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrCD_payloads), initial_setting_page0_adrCD_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrCE_payloads), initial_setting_page0_adrCE_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrEF_payloads), initial_setting_page0_adrEF_payloads, 0,0},
    { SHDISP_DTYPE_GEN_WRITE2,  sizeof(initial_setting_page0_adrEE_payloads), initial_setting_page0_adrEE_payloads, 0,0},
    { SHDISP_DTYPE_GEN_LWRITE,  sizeof(initial_setting_page0_adrFA_payloads), initial_setting_page0_adrFA_payloads, 0,0}
};

static char exit_sleep_payloads[] = {
    0x11,
};

static struct shdisp_dsi_cmd_desc exit_sleep_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, sizeof(exit_sleep_payloads), exit_sleep_payloads,0,0}
};

static char disp_on_payloads[] = {
    0x29,
};

static struct shdisp_dsi_cmd_desc disp_on_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, sizeof(disp_on_payloads), disp_on_payloads,0,0}
};

/*************************************************/
/*   display off settings                        */
/*************************************************/
static char disp_off_payloads[] = {
    0x28,
};

static char sleep_in_payloads[] = {
    0x10,
};

static struct shdisp_dsi_cmd_desc disp_off_enter_sleep_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, sizeof(disp_off_payloads), disp_off_payloads, 16666,0},
    { SHDISP_DTYPE_DCS_WRITE, sizeof(sleep_in_payloads), sleep_in_payloads, 120*1000,0}
};

#ifdef SHDISP_SAZABI_VIDEO_STOP
static struct shdisp_dsi_cmd_desc disp_off_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, sizeof(disp_off_payloads), disp_off_payloads, 0, 0}
};
#endif /* SHDISP_SAZABI_VIDEO_STOP */

static char reset_number_of_erros_on_dsi_payloads[] = {
    0x05, 0x00
};

static char clear_dsi_error_bit_payloads[] = {
    0x0E, 0x00
};

static struct shdisp_dsi_cmd_desc clear_dsi_error_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE1, sizeof(reset_number_of_erros_on_dsi_payloads), reset_number_of_erros_on_dsi_payloads, 0,0},
    { SHDISP_DTYPE_DCS_WRITE1, sizeof(clear_dsi_error_bit_payloads), clear_dsi_error_bit_payloads, 0,0},
};

#endif /* SHDISP_SAZABI_DATA_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */

