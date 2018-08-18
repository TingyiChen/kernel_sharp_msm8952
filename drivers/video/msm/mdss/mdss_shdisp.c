/* drivers/video/msm/mdss/mdss_shdisp.c  (Display Driver)
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
#include <mdss_shdisp.h>
#include <linux/types.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shdisp_dsi.h>
#include "mdss_fb.h"
#include <mdss_dsi.h>
#include <mdss_mdp.h>

#define MDSS_SHDISP_DSI_COLLECT_MAX      (512)
#ifndef CONFIG_USES_SHLCDC
#define MDSS_SHDISP_DSI_PAYLOAD_BUF_LEN  (4 * MDSS_SHDISP_DSI_COLLECT_MAX)
#endif /* CONFIG_USES_SHLCDC */

#define MDSS_DSI_CTRL_DSI_EN             BIT(0)
#define MDSS_DSI_CTRL_VIDEO_MODE_EN      BIT(1)
#define MDSS_DSI_CTRL_CMD_MODE_EN        BIT(2)

#define MDSS_SHDISP_PRINT_CMD_DESC(cmds, size)												\
		do {																				\
			int i;																			\
			char buf[64];																	\
			struct shdisp_dsi_cmd_desc *cmd;												\
			for (i = 0; i < size; i++) {													\
				cmd = &cmds[i];																\
				hex_dump_to_buffer(cmd->payload, cmd->dlen, 16, 1, buf, sizeof(buf), 0);	\
				pr_debug("%s: dtype=%02x dlen=%d mode=%02x wait=%d data=%s\n", __func__,	\
						(unsigned char)cmd->dtype, cmd->dlen, cmd->mode, cmd->wait, buf);	\
			}																				\
		} while (0)

#define MDSS_SHDISP_PRINT_RX_DATA(data, size)												\
		do {																				\
			char buf[64];																	\
			hex_dump_to_buffer(data, size, 16, 1, buf, sizeof(buf), 0);						\
			pr_debug("%s: rx_data=%s\n", __func__, buf);									\
		} while (0)

static bool lcd_disp_on = false;
struct mdss_dsi_ctrl_pdata *mdss_dsi_ctrl = NULL;
struct mdss_dsi_ctrl_pdata *mdss_dsi_sctrl = NULL;
static int mdss_shdisp_callback_data = 0;
#ifndef CONFIG_USES_SHLCDC
static int mdss_shdisp_collect_cmd_cnt = 0;
static struct dsi_cmd_desc mdss_shdisp_collect_cmds[MDSS_SHDISP_DSI_COLLECT_MAX];
static char mdss_shdisp_collect_payloads[MDSS_SHDISP_DSI_PAYLOAD_BUF_LEN];
static int mdss_shdisp_used_payloads = 0;
#endif /* CONFIG_USES_SHLCDC */
static bool mdss_shdisp_video_transfer_ctrl_kickoff_flg = false;
static bool mdss_shdisp_is_required_dsi_clk_ctrl = true;
static int mdss_shdisp_bdic_bkl_fixed = false;
static int mdss_shdisp_bdic_bkl_param = 0;

#ifndef CONFIG_USES_SHLCDC
static int mdss_shdisp_is_cmdmode_eng_on(struct mdss_dsi_ctrl_pdata *ctrl);
static void mdss_shdisp_cmdmode_eng_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, int enable);
static int mdss_shdisp_first_display_done = 0;
#endif /* CONFIG_USES_SHLCDC */
static int mdss_shdisp_video_transfer_ctrl_no_commit(struct msm_fb_data_type *mfd, int onoff);

static struct semaphore mdss_shdisp_blank_sem;
static struct semaphore mdss_shdisp_disp_sem;
static struct semaphore mdss_shdisp_host_dsi_cmd_sem;

#ifndef CONFIG_USES_SHLCDC
static void mdss_shdisp_set_required_clk_ctrl(bool onoff);
#endif /* CONFIG_USES_SHLCDC */
static int mdss_shdisp_mdp_cmd_clk_ctrl(bool onoff);

int mdss_shdisp_mdp_hr_video_suspend(void);
int mdss_shdisp_mdp_hr_video_resume(void);
#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#ifndef SHDISP_DISABLE_HR_VIDEO
static int mdss_shdisp_mdp_hr_video_clk_on(void);
static int mdss_shdisp_mdp_hr_video_clk_off(void);
#endif /* SHDISP_DISABLE_HR_VIDEO */
#endif /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_HAYABUSA */

#ifdef SHDISP_DET_DSI_MIPI_ERROR
extern void mdss_dsi_phy_dln0_err_clear(struct mdss_dsi_ctrl_pdata *ctrl);
extern void mdss_dsi_phy_dln0_err_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, bool onoff);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */
#ifndef SHDISP_DISABLE_HR_VIDEO
extern int mdss_mdp_hr_video_fps_led_start(struct mdss_mdp_ctl *ctl);
extern void mdss_mdp_hr_video_fps_led_stop(struct mdss_mdp_ctl *ctl);
extern int mdss_mdp_hr_video_suspend(struct mdss_mdp_ctl *ctl, int tg_en_flg);
extern int mdss_mdp_hr_video_resume(struct mdss_mdp_ctl *ctl, int tg_en_flg);
extern void mdss_mdp_hr_video_transfer_ctrl(struct msm_fb_data_type *mfd, int onoff, int commit);
extern int mdss_mdp_hr_video_clk_ctrl(struct mdss_mdp_ctl *ctl, int onoff);
#else  /* SHDISP_DISABLE_HR_VIDEO */
extern void mdss_mdp_video_transfer_ctrl(struct msm_fb_data_type *mfd, int onoff, int commit);
#if defined(CONFIG_SHDISP_PANEL_SAZABI)
extern int mdss_mdp_dynamic_fps_led(int onoff, struct mdss_mdp_ctl *ctl);
#endif /* CONFIG_SHDISP_PANEL_SAZABI */
#endif /* SHDISP_DISABLE_HR_VIDEO */
extern struct fb_info *mdss_fb_get_fbinfo(int id);
extern int mdss_dsi_cmd_bus_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, bool enable);
extern int mdss_mdp_cmd_tearcheck_enable(struct mdss_mdp_ctl *ctl, bool enable);

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lock_recovery(void)
{
	down(&mdss_shdisp_blank_sem);
	down(&mdss_shdisp_disp_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_unlock_recovery(void)
{
	up(&mdss_shdisp_disp_sem);
	up(&mdss_shdisp_blank_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lock_display(void)
{
	down(&mdss_shdisp_disp_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_unlock_display(void)
{
	up(&mdss_shdisp_disp_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lock_blank(void)
{
	down(&mdss_shdisp_blank_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_unlock_blank(void)
{
	up(&mdss_shdisp_blank_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lock_host_dsi_cmd(void)
{
	down(&mdss_shdisp_host_dsi_cmd_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_unlock_host_dsi_cmd(void)
{
	up(&mdss_shdisp_host_dsi_cmd_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
bool mdss_shdisp_get_disp_status(void)
{
#ifndef SHDISP_NOT_SUPPORT_NO_OS
	shdisp_api_get_boot_context();

	if( shdisp_api_get_boot_disp_status() ) {
		lcd_disp_on = true;
		return true;
	}
#endif  /* SHDISP_NOT_SUPPORT_NO_OS */

	return false;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
bool mdss_shdisp_get_upper_unit_is_connected(void)
{
	return (shdisp_API_check_upper_unit() == SHDISP_RESULT_SUCCESS);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
bool mdss_shdisp_get_bdic_is_exist(void)
{
	return shdisp_API_get_bdic_is_exist();
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (ctrl->ndx != DSI_CTRL_0) {
		return;
	}
	mdss_shdisp_set_dsi_ctrl(ctrl);

	if (lcd_disp_on == true) {
		pr_debug("%s: already power on.\n", __func__);
		return;
	}
	shdisp_api_main_lcd_power_on();
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (ctrl->ndx != DSI_CTRL_0) {
		return;
	}
	shdisp_api_main_lcd_power_off();

	mdss_dsi_ctrl = NULL;
	mdss_dsi_sctrl = NULL;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_dsi_enable_panel_analog_power(int enable)
{
	int rc = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = mdss_dsi_ctrl;
	pr_debug("LCDDBG:[%s] enter - ()\n", __func__);
	if (!ctrl_pdata) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return rc;
	}

	rc = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, enable);
	if (rc < 0) {
		pr_err("%s: failed to analog power control. rc=%d\n", __func__, rc);
		rc = -EIO;
	}
	return rc;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_on(struct mdss_panel_data *pdata)
{
#ifndef CONFIG_USES_SHLCDC
	int cmdengon;
#endif /* CONFIG_USES_SHLCDC */
	struct mdss_dsi_ctrl_pdata *ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (ctrl->ndx != DSI_CTRL_0) {
		return;
	}
#ifndef CONFIG_USES_SHLCDC
	mdss_shdisp_set_required_clk_ctrl(false);

	cmdengon = mdss_shdisp_is_cmdmode_eng_on(ctrl);
	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(ctrl, 1);
	}

	mdss_shdisp_collect_cmd_cnt = 0;
	mdss_shdisp_used_payloads = 0;
#endif /* CONFIG_USES_SHLCDC */
	shdisp_api_main_lcd_disp_on();
	if (pdata->panel_info.type == MIPI_VIDEO_PANEL) {
		shdisp_api_main_lcd_start_display();
		lcd_disp_on = true;
	}

#ifndef CONFIG_USES_SHLCDC
	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(ctrl, 0);
	}

	mdss_shdisp_set_required_clk_ctrl(true);
#endif /* CONFIG_USES_SHLCDC */
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_set_dsi_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (ctrl_pdata->ndx != DSI_CTRL_0) {
		return;
	}
	mdss_dsi_ctrl = ctrl_pdata;
	if (ctrl_pdata->panel_data.next) {
		mdss_dsi_sctrl = container_of(ctrl_pdata->panel_data.next, struct mdss_dsi_ctrl_pdata, panel_data);
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_off(struct mdss_panel_data *pdata)
{
#ifndef CONFIG_USES_SHLCDC
	int cmdengon;
	struct mdss_dsi_ctrl_pdata *ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (ctrl->ndx != DSI_CTRL_0) {
		return;
	}

	mdss_shdisp_set_required_clk_ctrl(false);

	cmdengon = mdss_shdisp_is_cmdmode_eng_on(ctrl);
	if(!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(ctrl, 1);
	}

	mdss_shdisp_collect_cmd_cnt = 0;
	mdss_shdisp_used_payloads = 0;
#endif /* CONFIG_USES_SHLCDC */
	shdisp_api_main_lcd_disp_off();
	lcd_disp_on = false;

#ifndef CONFIG_USES_SHLCDC
	if(!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(ctrl, 0);
	}
	mdss_shdisp_first_display_done = 0;

	mdss_shdisp_set_required_clk_ctrl(true);
#endif /* CONFIG_USES_SHLCDC */

}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
bool mdss_shdisp_is_disp_on(void)
{
	return lcd_disp_on;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_start_display(void)
{
#ifndef CONFIG_USES_SHLCDC
	int cmdengon;
	struct mdss_mdp_ctl *ctl;

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return;
	}

	ctl = mdss_shdisp_get_mdpctrl(0);
	if (!ctl) {
		pr_err("%s: ctl is NULL.\n", __func__);
		return;
	}

	cmdengon = mdss_shdisp_is_cmdmode_eng_on(mdss_dsi_ctrl);
	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(mdss_dsi_ctrl, 1);
	}
	mdss_shdisp_collect_cmd_cnt = 0;
	mdss_shdisp_used_payloads = 0;
#endif /* CONFIG_USES_SHLCDC */
	shdisp_api_main_lcd_start_display();
	lcd_disp_on = true;
#ifndef CONFIG_USES_SHLCDC
	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(mdss_dsi_ctrl, 0);
	}

	if (mdss_dsi_ctrl->panel_mode == DSI_CMD_MODE) {
		mdss_mdp_cmd_tearcheck_enable(ctl, true);
	}
#endif /* CONFIG_USES_SHLCDC */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_post_video_start()
{
#ifndef CONFIG_USES_SHLCDC
	shdisp_api_main_lcd_post_video_start();
#endif /* CONFIG_USES_SHLCDC */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_bkl_ctl(u32 bl_level)
{
	struct shdisp_main_bkl_ctl bkl;

	pr_debug("%s: called bl_level=%u\n", __func__, bl_level);

	if( bl_level == 0 ) {
		shdisp_api_main_bkl_off();
	} else {
		bkl.mode = SHDISP_MAIN_BKL_MODE_FIX;
		bkl.param = bl_level;
		shdisp_api_main_bkl_on(&bkl);
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_tri_led_set_color(char red, char green, char blue)
{
#ifndef CONFIG_USES_SHLCDC
	struct shdisp_tri_led param;

	param.red = red;
	param.green = green;
	param.blue = blue;
	param.ext_mode = SHDISP_TRI_LED_EXT_MODE_DISABLE;
	param.led_mode = SHDISP_TRI_LED_MODE_NORMAL;

	shdisp_api_tri_led_set_color(&param);
#endif /* CONFIG_USES_SHLCDC */
}

#ifdef CONFIG_USES_SHLCDC
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_pll_ctl(int ctl)
{
	return shdisp_api_main_pll_ctl(ctl);
}
#endif /* CONFIG_USES_SHLCDC */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_shutdown(int seq)
{
	shdisp_api_shutdown(seq);
}

#ifndef CONFIG_USES_SHLCDC
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_dsi_to_mdss_dsi(const struct shdisp_dsi_cmd_desc * shdisp_cmds,  struct dsi_cmd_desc * mdss_cmds,
								int size, int isAck)
{
	int cnt;
	for (cnt = 0; cnt != size; cnt++) {
		if (mdss_shdisp_used_payloads + shdisp_cmds->dlen < MDSS_SHDISP_DSI_PAYLOAD_BUF_LEN) {
			mdss_cmds->dchdr.dtype  = shdisp_cmds->dtype;
			mdss_cmds->dchdr.last = shdisp_cmds->wait ? 1 : 0;
			mdss_cmds->dchdr.vc	    = 0;
			mdss_cmds->dchdr.ack    = isAck ? 1 : 0;
			mdss_cmds->dchdr.wait   = shdisp_cmds->wait ? ((shdisp_cmds->wait+999)/1000) : 0; /* mdss_dsi(ms) <- shdisp_dsi(usec) */
			mdss_cmds->dchdr.dlen   = shdisp_cmds->dlen;
			mdss_cmds->payload      = mdss_shdisp_collect_payloads + mdss_shdisp_used_payloads;
			memcpy(mdss_cmds->payload, shdisp_cmds->payload, shdisp_cmds->dlen);
			mdss_shdisp_used_payloads += shdisp_cmds->dlen;
		} else {
			pr_err("LCDERR: buffer size over %s: shdisp_cmds->dlen=%d, mdss_shdisp_used_payloads = %d\n",
			                                __func__, shdisp_cmds->dlen, mdss_shdisp_used_payloads );
		}
		mdss_cmds++;
		shdisp_cmds++;
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_collect_cmd(struct shdisp_dsi_cmd_desc * shdisp_cmds, int size)
{
	if (mdss_shdisp_collect_cmd_cnt + size >= MDSS_SHDISP_DSI_COLLECT_MAX) {
		pr_err("LCDERR: buffer size over %s: size=%d, mdss_shdisp_collect_cmd_cnt = %d\n",
		                                __func__, size, mdss_shdisp_collect_cmd_cnt );
		return;
	}
	mdss_shdisp_dsi_to_mdss_dsi(shdisp_cmds, &mdss_shdisp_collect_cmds[mdss_shdisp_collect_cmd_cnt], size, 0);
	mdss_shdisp_collect_cmd_cnt += size;
	pr_debug("%s: size=%d, mdss_shdisp_collect_cmd_cnt = %d\n", __func__, size, mdss_shdisp_collect_cmd_cnt );
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_kick_collect_cmd(struct mdss_dsi_ctrl_pdata *ctrl, struct shdisp_dsi_cmd_desc *cmds)
{
	int ret = 0;
	int cmdengon;
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: begin cnt=%d", __func__, mdss_shdisp_collect_cmd_cnt);
	if (!mdss_shdisp_collect_cmd_cnt) {
		return SHDISP_RESULT_SUCCESS;
	}

	mdss_shdisp_collect_cmds[mdss_shdisp_collect_cmd_cnt-1].dchdr.last   = 1;
	pr_debug("%s: kick_count = %d\n", __func__, mdss_shdisp_collect_cmd_cnt );

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.flags = CMD_REQ_COMMIT;
	if(cmds->mode) {
		pr_debug("%s: cmds->mode=0x%x\n", __func__, cmds->mode);
		cmdreq.flags |= cmds->mode;
	}
	cmdreq.cmds = mdss_shdisp_collect_cmds;
	cmdreq.cmds_cnt = mdss_shdisp_collect_cmd_cnt;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	cmdengon = mdss_shdisp_is_cmdmode_eng_on(ctrl);
	if(!cmdengon){
		mdss_shdisp_cmdmode_eng_ctrl(ctrl, 1);
	}
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(ctrl, 0);
	}

	mdss_shdisp_collect_cmd_cnt = 0;
	mdss_shdisp_used_payloads = 0;

	pr_debug("%s: end. ret=%d", __func__, ret);
	if (ret > 0) {
		return SHDISP_RESULT_SUCCESS;
	} else {
		return SHDISP_RESULT_FAILURE;
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_is_cmdmode_eng_on(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int dsi_ctrl;
	int cmdmode_on = MDSS_DSI_CTRL_DSI_EN | MDSS_DSI_CTRL_CMD_MODE_EN;
	int ret = 0;

	mdss_shdisp_clk_ctrl(true);

	dsi_ctrl = MIPI_INP((ctrl->ctrl_base) + 0x0004);

	if ((dsi_ctrl&cmdmode_on) == cmdmode_on) {
		ret = 1;
	} else {
		ret = 0;
	}
	mdss_shdisp_clk_ctrl(false);

	return ret;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_cmdmode_eng_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, int enable)
{
	int mode;

	mode = (enable ? DSI_CMD_MODE : \
                             (ctrl->panel_mode == DSI_CMD_MODE ? \
                                  DSI_CMD_MODE : DSI_VIDEO_MODE \
                             ) \
                   );

	pr_debug("%s: request=%d\n", __func__, enable);

	mdss_dsi_op_mode_config(mode, &ctrl->panel_data);

	return;
}
#endif /* CONFIG_USES_SHLCDC */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_host_dsi_init_cbdata(void)
{
	mdss_shdisp_callback_data = 0xffff;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_host_dsi_get_cbdata(void)
{
	return mdss_shdisp_callback_data;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_host_dsi_cb(int data)
{
	mdss_shdisp_callback_data = data;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_host_dsi_tx(int commit, struct shdisp_dsi_cmd_desc * shdisp_cmds, int size)
{
	int ret = 0;

	pr_debug("%s: in commit=%d\n", __func__, commit);
	MDSS_SHDISP_PRINT_CMD_DESC(shdisp_cmds, size);

	mdss_shdisp_lock_host_dsi_cmd();
	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		mdss_shdisp_unlock_host_dsi_cmd();
		return SHDISP_RESULT_FAILURE;
	}

#ifndef CONFIG_USES_SHLCDC
	mdss_shdisp_collect_cmd(shdisp_cmds, size);
#endif /* CONFIG_USES_SHLCDC */

	if (commit) {
		mdss_shdisp_clk_ctrl(true);

#ifndef CONFIG_USES_SHLCDC
		ret = mdss_shdisp_kick_collect_cmd(mdss_dsi_ctrl, shdisp_cmds);
#endif /* CONFIG_USES_SHLCDC */

		mdss_shdisp_clk_ctrl(false);
	}

	mdss_shdisp_unlock_host_dsi_cmd();

	pr_debug("%s: out ret=%d\n", __func__, ret);
	return ret;
}

#ifndef CONFIG_USES_SHLCDC
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_host_dsi_rx(struct shdisp_dsi_cmd_desc * cmds, unsigned char * rx_data, int rx_size)
{
	int ret = 0;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc mdss_cmds;
	char payload[2];
	int cmdengon;

	pr_debug("%s: in rx_size=%d\n", __func__, rx_size);
	MDSS_SHDISP_PRINT_CMD_DESC(cmds, 1);

	mdss_shdisp_lock_host_dsi_cmd();

	if ( !mdss_dsi_ctrl ) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		mdss_shdisp_unlock_host_dsi_cmd();
		return SHDISP_RESULT_FAILURE;
	}

	mdss_shdisp_clk_ctrl(true);

	ret = mdss_shdisp_kick_collect_cmd(mdss_dsi_ctrl, cmds);
	if (ret != SHDISP_RESULT_SUCCESS) {
		pr_err("LCDERR: %s mdss_shdisp_kick_collect_cmd(cmds) ret=%d", __func__, ret);
		mdss_shdisp_clk_ctrl(false);
		mdss_shdisp_unlock_host_dsi_cmd();
		return SHDISP_RESULT_FAILURE;
	}

	memset(&cmdreq, 0, sizeof(cmdreq) );
	memset(&mdss_cmds, 0, sizeof(mdss_cmds) );
	mdss_shdisp_dsi_to_mdss_dsi(cmds, &mdss_cmds, 1, 1);
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	if(cmds->mode) {
		cmdreq.flags |= cmds->mode;
	}
	cmdreq.cmds = &mdss_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.cb = mdss_shdisp_host_dsi_cb;
	cmdreq.rbuf = rx_data;
	cmdreq.rlen = rx_size;

	if (mdss_cmds.dchdr.dlen>1) {
		payload[0] = mdss_cmds.payload[0];
		payload[1] = mdss_cmds.payload[1];
	} else {
		payload[0] = mdss_cmds.payload[0];
		payload[1] = 0;
	}

	mdss_cmds.payload = payload;
	mdss_shdisp_used_payloads = 0;

	cmdengon = mdss_shdisp_is_cmdmode_eng_on(mdss_dsi_ctrl);
	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(mdss_dsi_ctrl, 1);
	}

	mdss_shdisp_host_dsi_init_cbdata();
	mdss_dsi_cmdlist_put(mdss_dsi_ctrl, &cmdreq);

	if (cmdreq.rlen != mdss_shdisp_host_dsi_get_cbdata()) {
		pr_err("LCDERR: %s callback_data=%d\n", __func__, mdss_shdisp_host_dsi_get_cbdata());
		if (!cmdengon) {
			mdss_shdisp_cmdmode_eng_ctrl(mdss_dsi_ctrl, 0);
		}
		mdss_shdisp_clk_ctrl(false);
		mdss_shdisp_unlock_host_dsi_cmd();
		return SHDISP_RESULT_FAILURE;
	}

	if (!cmdengon) {
		mdss_shdisp_cmdmode_eng_ctrl(mdss_dsi_ctrl, 0);
	}

	mdss_shdisp_clk_ctrl(false);
	mdss_shdisp_unlock_host_dsi_cmd();

	MDSS_SHDISP_PRINT_RX_DATA(rx_data, rx_size);
	pr_debug("%s: out\n", __func__);
	return SHDISP_RESULT_SUCCESS;
}
#endif /* CONFIG_USES_SHLCDC */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_video_transfer_ctrl(int onoff, int commit)
{
	struct fb_info * fbi;
	struct msm_fb_data_type *mfd;
	fbi = mdss_fb_get_fbinfo(0);
	mfd = (struct msm_fb_data_type *)fbi->par;
#ifndef SHDISP_DISABLE_HR_VIDEO
	mdss_mdp_hr_video_transfer_ctrl(mfd, onoff, commit);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	mdss_mdp_video_transfer_ctrl(mfd, onoff, commit);
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_video_transfer_ctrl_no_commit(struct msm_fb_data_type *mfd, int onoff)
{
	int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_DISABLE_HR_VIDEO
	struct mdss_mdp_ctl *ctl;
#endif /* SHDISP_DISABLE_HR_VIDEO */

	pr_debug("%s : onoff=%d\n", __func__, onoff);

#ifndef SHDISP_DISABLE_HR_VIDEO
	ctl = mfd_to_ctl(mfd);
	if (ctl == NULL) {
		pr_debug("%s : ctl is NULL\n", __func__);
		return -EINVAL;
	}

	if (onoff) {
		ret = mdss_mdp_hr_video_resume(ctl, false);
	} else {
		ret = mdss_mdp_hr_video_suspend(ctl, false);
	}
#else  /* SHDISP_DISABLE_HR_VIDEO */
	mdss_mdp_video_transfer_ctrl(mfd, onoff, false);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	pr_debug("%s : ret=%d\n", __func__, ret);
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_video_transfer_ctrl_set_flg(struct msm_fb_data_type *mfd, bool change)
{
	if ((mfd->panel.type == MIPI_VIDEO_PANEL) ||
		(mfd->panel.type == MIPI_CMD_PANEL)) {
		if (mdss_shdisp_video_transfer_ctrl_kickoff_flg != change) {
			mdss_shdisp_video_transfer_ctrl_kickoff_flg = change;
			pr_debug("%s : video_transfer_ctrl_kickoff_flg=%d\n", __func__, change);
		}
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_video_transfer_ctrl_kickoff(struct msm_fb_data_type *mfd, int onoff)
{
	struct mdss_mdp_ctl *ctl;
	int ret = -EPERM;

	ctl = mfd_to_ctl(mfd);

	if (mdss_shdisp_video_transfer_ctrl_kickoff_flg == true) {
		if (mfd->panel.type == MIPI_VIDEO_PANEL) {
			ret = mdss_shdisp_video_transfer_ctrl_no_commit(mfd, onoff);
		} else if (mfd->panel.type == MIPI_CMD_PANEL) {
			if (onoff == false) {
				mdss_mdp_display_wait4pingpong(ctl, false);
			}
			ret = SHDISP_RESULT_SUCCESS;
		}
	}
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_display_done(void)
{
	int ret = SHDISP_RESULT_SUCCESS;
#ifndef CONFIG_USES_SHLCDC
	if (mdss_shdisp_first_display_done == 0) {
		mdss_shdisp_first_display_done = 1;
		ret = shdisp_api_main_display_done();
	}
#endif /* CONFIG_USES_SHLCDC */
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
struct mdss_mdp_ctl* mdss_shdisp_get_mdpctrl(int fbinx)
{
	return ((struct mdss_overlay_private*)((struct msm_fb_data_type*)mdss_fb_get_fbinfo(fbinx)->par)->mdp.private1)->ctl;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_mfr_ctl(int brightness)
{
	int ret = SHDISP_RESULT_SUCCESS;

	ret = shdisp_api_mfr_ctl(brightness);
	return ret;
}


#ifndef CONFIG_USES_SHLCDC
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_set_required_clk_ctrl(bool onoff)
{
	if ( !mdss_dsi_ctrl ){
		mdss_shdisp_is_required_dsi_clk_ctrl = false;
		return;
	}

	mdss_shdisp_is_required_dsi_clk_ctrl = onoff;
}
#endif /* CONFIG_USES_SHLCDC */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_clk_ctrl(bool onoff)
{
	if ( !mdss_dsi_ctrl ){
		return;
	}

	if ( !mdss_shdisp_is_required_dsi_clk_ctrl ) {
		return;
	}

	if ( mdss_dsi_ctrl->panel_mode == DSI_CMD_MODE ) {
		mdss_shdisp_mdp_cmd_clk_ctrl(onoff);
	} else {
#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#ifndef SHDISP_DISABLE_HR_VIDEO
		if (onoff) {
			mdss_shdisp_mdp_hr_video_clk_on();
		} else {
			mdss_shdisp_mdp_hr_video_clk_off();
		}
#endif /* SHDISP_DISABLE_HR_VIDEO */
#endif /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_HAYABUSA */
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_mdp_cmd_clk_ctrl(bool onoff)
{
	pr_debug("LCDDBG:[%s] enter - (onoff=%d)\n", __func__, onoff);

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR:[%s] invalid DSI control.\n", __func__);
		return -EINVAL;
	}

	if (onoff) {
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

		mdss_dsi_clk_ctrl(mdss_dsi_ctrl, DSI_ALL_CLKS, 1);
	} else {
		mdss_dsi_clk_ctrl(mdss_dsi_ctrl, DSI_ALL_CLKS, 0);

		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	}

	pr_debug("LCDDBG:[%s] leave - (ret=0)\n", __func__);
	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_mdp_cmd_kickoff()
{
	int ret;
	int ret2;
	struct mdss_mdp_ctl *pctl;
	struct mdss_mdp_ctl *sctl;

	pr_debug("LCDDBG:[%s] enter - ()\n", __func__);

	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] mdpctrl is NULL.\n", __func__);
		return;
	}

	if (!pctl->ops.display_fnc) {
		pr_err("LCDERR:[%s] display_fnc is NULL.\n", __func__);
		return;
	}
	sctl = mdss_mdp_get_split_ctl(pctl);

	mutex_lock(&pctl->lock);

	mdss_mdp_ctl_perf_set_transaction_status(pctl, PERF_SW_COMMIT_STATE, PERF_STATUS_BUSY);
	if (sctl) {
		mdss_mdp_ctl_perf_set_transaction_status(sctl, PERF_SW_COMMIT_STATE, PERF_STATUS_BUSY);
	}
	mdss_mdp_ctl_perf_update_ctl(pctl, 1);

	if (pctl->ops.wait_pingpong) {
		ret2 = pctl->ops.wait_pingpong(pctl, NULL);
		if(ret2){
			pr_err("%s: failed to wait_pingpong. ret=%d\n", __func__, ret2);
		}
	}

	if (sctl && sctl->ops.wait_pingpong) {
		ret2 = sctl->ops.wait_pingpong(sctl, NULL);
		if(ret2){
			pr_err("%s: failed to wait_pingpong. ret=%d\n", __func__, ret2);
		}
	}
	ret = pctl->ops.display_fnc(pctl, NULL);
	if (ret) {
		pr_err("LCDERR:[%s] failed to display_fnc(). (ret=%d)\n", __func__, ret);
		mutex_unlock(&pctl->lock);
		return;
	}

	mutex_unlock(&pctl->lock);

	pr_debug("LCDDBG:[%s] leave - ()\n", __func__);
	return;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_mdp_hr_video_suspend(void)
{
#ifndef SHDISP_DISABLE_HR_VIDEO
	struct mdss_mdp_ctl *pctl;
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] pctl is NULL.\n", __func__);
		return -EINVAL;
	}
	return mdss_mdp_hr_video_suspend(pctl, true);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	return 0;
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_mdp_hr_video_resume(void)
{
#ifndef SHDISP_DISABLE_HR_VIDEO
	struct mdss_mdp_ctl *pctl;
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] pctl is NULL.\n", __func__);
		return -EINVAL;
	}
	return mdss_mdp_hr_video_resume(pctl, false);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	return 0;
#endif /* SHDISP_DISABLE_HR_VIDEO */
}

#if defined(CONFIG_SHDISP_PANEL_ANDY) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#ifndef SHDISP_DISABLE_HR_VIDEO
static int mdss_shdisp_mdp_hr_video_clk_on(void)
{
	struct mdss_mdp_ctl *pctl;
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] pctl is NULL.\n", __func__);
		return -EINVAL;
	}
	return mdss_mdp_hr_video_clk_ctrl(pctl, 1);
}

static int mdss_shdisp_mdp_hr_video_clk_off(void)
{
	struct mdss_mdp_ctl *pctl;
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] pctl is NULL.\n", __func__);
		return -EINVAL;
	}
	return mdss_mdp_hr_video_clk_ctrl(pctl, 0);
}
#endif /* SHDISP_DISABLE_HR_VIDEO */
#endif /* CONFIG_SHDISP_PANEL_ANDY || CONFIG_SHDISP_PANEL_HAYABUSA */

#ifdef SHDISP_DET_DSI_MIPI_ERROR
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_mipi_err_clear()
{
	pr_debug("LCDDBG:[%s] enter - ()\n", __func__);

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR:[%s] invalid DSI control.\n", __func__);
		return;
	}

	mdss_shdisp_clk_ctrl(true);
	mdss_dsi_phy_dln0_err_clear(mdss_dsi_ctrl);
	if (mdss_dsi_sctrl) {
		mdss_dsi_phy_dln0_err_clear(mdss_dsi_sctrl);
	}
	mdss_shdisp_clk_ctrl(false);

	pr_debug("LCDDBG:[%s] leave - ()\n", __func__);
	return;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_dsi_mipi_err_ctrl(bool enable)
{
	pr_debug("LCDDBG:[%s] enter - (enable=%d)\n", __func__, enable);

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR:[%s] invalid DSI control.\n", __func__);
		return -EINVAL;
	}

	mdss_shdisp_clk_ctrl(true);
	mdss_dsi_phy_dln0_err_ctrl(mdss_dsi_ctrl, enable);
	if (mdss_dsi_sctrl) {
		mdss_dsi_phy_dln0_err_ctrl(mdss_dsi_sctrl, enable);
	}
	mdss_shdisp_clk_ctrl(false);

	pr_debug("LCDDBG:[%s] leave - (ret=0)\n", __func__);
	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_det_recovery(struct mdss_dsi_ctrl_pdata *ctrl, int status)
{
	int nret;

	pr_debug("LCDDBG:[%s] enter - (status=%d)\n", __func__, status);

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR:[%s] invalid DSI control.\n", __func__);
		return;
	}

	if (!ctrl) {
		pr_err("LCDERR:[%s] invalid DSI control, will be exited without recovery. ()\n", __func__);
		return;
	}
	pr_err("LCDERR:[%s] MIPI error DSI%d detected!!! (status=%#x)\n", __func__, ctrl->ndx, status);

	nret = shdisp_api_do_mipi_dsi_det_recovery();
	if (nret) {
		pr_err("LCDERR:[%s] faild to recovery. (ret=%d)\n", __func__, nret);
		mdss_shdisp_clk_ctrl(true);
		mdss_dsi_phy_dln0_err_ctrl(mdss_dsi_ctrl, true);
		if (mdss_dsi_sctrl) {
			mdss_dsi_phy_dln0_err_ctrl(mdss_dsi_sctrl, true);
		}
		mdss_shdisp_clk_ctrl(false);
		return;
	}

	pr_debug("LCDDBG:[%s] leave - ()\n", __func__);
	return;
}
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_set_lp00_mode_sub(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable, u32 *backup)
{
	u32 dsi_ctrl;

	dsi_ctrl = MIPI_INP((ctrl_pdata->ctrl_base) + 0x0004);
	pr_debug("%s: addr=0x%p INP dsi_ctrl=0x%02X\n", __func__, ((ctrl_pdata->ctrl_base) + 0x0004), dsi_ctrl);
	if (!enable) {
		*backup = dsi_ctrl;
		dsi_ctrl = 0x000;
	} else {
		dsi_ctrl |= *backup;
		*backup = 0;
	}
	pr_debug("%s: addr=0x%p OUTP dsi_ctrl=0x%02X\n", __func__, ((ctrl_pdata->ctrl_base) + 0x0004), dsi_ctrl);
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0004, dsi_ctrl);
	wmb();
	if (enable && (dsi_ctrl & BIT(0))) {
		mdss_dsi_sw_reset(ctrl_pdata, true);
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_set_lp00_mode(int enable)
{
	static u32 backup_master = 0;
	static u32 backup_slave = 0;

	pr_debug("%s: called\n", __func__);

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return SHDISP_RESULT_FAILURE;
	}

	mdss_shdisp_set_lp00_mode_sub(mdss_dsi_ctrl, enable, &backup_master);
	if (mdss_dsi_sctrl) {
		mdss_shdisp_set_lp00_mode_sub(mdss_dsi_sctrl, enable, &backup_slave);
	}

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_fps_led_start(void)
{
#if !defined(SHDISP_DISABLE_HR_VIDEO) || defined(CONFIG_SHDISP_PANEL_SAZABI)
	int ret;
	struct mdss_mdp_ctl *ctl;

	ctl = mdss_shdisp_get_mdpctrl(0);
	if (!ctl) {
		pr_err("LCDERR:[%s] ctl is NULL.\n", __func__);
		return SHDISP_RESULT_FAILURE;
	}

#ifndef SHDISP_DISABLE_HR_VIDEO
	ret = mdss_mdp_hr_video_fps_led_start(ctl);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	ret = mdss_mdp_dynamic_fps_led(true, ctl);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	if (!ret) {
		return SHDISP_RESULT_SUCCESS;
	} else {
		return SHDISP_RESULT_FAILURE;
	}
#else  /* !defined(SHDISP_DISABLE_HR_VIDEO) || defined(CONFIG_SHDISP_PANEL_SAZABI) */
	return SHDISP_RESULT_SUCCESS;
#endif /* !defined(SHDISP_DISABLE_HR_VIDEO) || defined(CONFIG_SHDISP_PANEL_SAZABI) */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_fps_led_stop(void)
{
#if !defined(SHDISP_DISABLE_HR_VIDEO) || defined(CONFIG_SHDISP_PANEL_SAZABI)
	struct mdss_mdp_ctl *ctl;

	ctl = mdss_shdisp_get_mdpctrl(0);
	if (!ctl) {
		pr_err("LCDERR:[%s] ctl is NULL.\n", __func__);
		return;
	}

#ifndef SHDISP_DISABLE_HR_VIDEO
	mdss_mdp_hr_video_fps_led_stop(ctl);
#else  /* SHDISP_DISABLE_HR_VIDEO */
	mdss_mdp_dynamic_fps_led(false, ctl);
#endif /* SHDISP_DISABLE_HR_VIDEO */
#endif /* !defined(SHDISP_DISABLE_HR_VIDEO) || defined(CONFIG_SHDISP_PANEL_SAZABI) */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_dsi_bus_clk_ctrl(bool enable)
{
	if (mdss_dsi_ctrl) {
		return mdss_dsi_cmd_bus_ctrl(mdss_dsi_ctrl, enable);
	} else {
		pr_err("%s: invalid mdss_dsi_ctrl\n", __func__);
		return -ENODEV;
	}
	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_pre_blank_notify(void)
{
	shdisp_api_pre_blank_notify();
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_cmd_tearcheck_enable(bool enable)
{
	int ret;
	struct mdss_mdp_ctl *ctl;

	if (!mdss_dsi_ctrl) {
		pr_err("%s: mdss_dsi_ctrl is NULL.\n", __func__);
		return SHDISP_RESULT_FAILURE;
	}

	ctl = mdss_shdisp_get_mdpctrl(0);
	if (!ctl) {
		pr_err("%s: ctl is NULL.\n", __func__);
		return SHDISP_RESULT_FAILURE;
	}

	if (mdss_dsi_ctrl->panel_mode == DSI_CMD_MODE) {
		ret = mdss_mdp_cmd_tearcheck_enable(ctl, enable);
		if (ret) {
			return SHDISP_RESULT_FAILURE;
		}
	}

	return SHDISP_RESULT_SUCCESS;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_bdic_bkl_set(bool fixed, int param)
{
	mdss_shdisp_bdic_bkl_fixed = fixed;
	mdss_shdisp_bdic_bkl_param = param;
	pr_debug("%s: bdic_bkl fixed=%d param=%d\n", __func__, mdss_shdisp_bdic_bkl_fixed, mdss_shdisp_bdic_bkl_param);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_bdic_bkl_get_param(void)
{
	return mdss_shdisp_bdic_bkl_param;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
bool mdss_shdisp_bdic_bkl_get_fixed(void)
{
	return mdss_shdisp_bdic_bkl_fixed;
}

#ifdef SHDISP_USE_QUALCOMM_RECOVERY
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_report_panel_dead(void)
{
    struct mdss_mdp_ctl *ctl;

    ctl = mdss_shdisp_get_mdpctrl(0);
    if (!ctl) {
        pr_err("%s: ctl is NULL.\n", __func__);
        return SHDISP_RESULT_FAILURE;
    }
    mdss_fb_report_panel_dead(ctl->mfd);
    return 0;
}
#endif /* SHDISP_USE_QUALCOMM_RECOVERY */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int __init mdss_shdisp_init(void)
{
	sema_init(&mdss_shdisp_disp_sem, 1);
	sema_init(&mdss_shdisp_blank_sem, 1);
	sema_init(&mdss_shdisp_host_dsi_cmd_sem, 1);
	return 0;
}
module_init(mdss_shdisp_init);
