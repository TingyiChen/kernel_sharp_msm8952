/* drivers/video/msm/mdss/mdss_diag.c  (Display Driver)
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
#include <mdss_shdisp.h>
#include <mdss_diag.h>
#include <linux/types.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shdisp_dsi.h>
#include "mdss_fb.h"
#include <mdss_dsi.h>
#include <mdss_mdp.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include "mdss_debug.h"

#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#define MDSS_DIAG_MIPI_CHECK_ENABLE
#define MDSS_DIAG_MIPI_CLKCHG_ENABLE
#elif defined(CONFIG_SHDISP_PANEL_SAZABI)
#define MDSS_DIAG_MIPI_CHECK_ENABLE
#define MDSS_DIAG_MIPI_CLKCHG_ENABLE
#endif  /* CONFIG_SHDISP_PANEL_HAYABUSA */

#define MDSS_DSI_DSIPHY_REGULATOR_CTRL_0	(0x00)
#define MDSS_DIAG_WAIT_1FRAME_US			(16666)

#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
static uint8_t mdss_diag_mipi_check_amp_data;
static int mdss_diag_mipi_check_exec_state = false;
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */

#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
static int mdss_diag_mipi_check_exec(uint8_t *result, uint8_t frame_cnt, uint8_t amp, uint8_t sensitiv,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl,
			uint8_t *sensitiv_master, uint8_t *sensitiv_slave);
static int mdss_diag_mipi_check_manual(struct mdp_mipi_check_param *mipi_check_param,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl,
			uint8_t *sensitiv_master, uint8_t *sensitiv_slave);
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave,
			struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_diag_mipi_check_test(uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static int mdss_diag_mipi_check_test_video(uint8_t frame_cnt, struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_diag_mipi_check_test_cmd(uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static int mdss_diag_read_sensitiv(uint8_t *sensitiv_master, uint8_t *sensitiv_slave);
static int mdss_diag_write_sensitiv(uint8_t *sensitiv_master, uint8_t *sensitiv_slave);
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl);
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */

#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
static int mdss_diag_mipi_clkchg_setparam(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl);
static void mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_panel_data *pdata);
static int __mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl, struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl);
static int mdss_diag_mipi_clkchg_panel_clk_update(struct mdss_panel_data *pdata);
static int __mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_panel_porch_update(struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_panel(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl);
static void mdss_diag_mipi_clkchg_param_log(struct mdp_mipi_clkchg_param *mdp_mipi_clkchg_param);
static int mdss_diag_mipi_stop_clkln_hs(struct mdss_panel_data *pdata);
static void mdss_diag_mipi_start_clkln_hs(struct mdss_panel_data *pdata, int cnt);
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

#if defined(CONFIG_SHDISP_PANEL_ANDY)
extern int shdisp_api_set_freq_param(mdp_mipi_clkchg_panel_t *freq);
#endif  /* defined(CONFIG_SHDISP_PANEL_ANDY) */
#ifndef SHDISP_DET_DSI_MIPI_ERROR
extern void mdss_dsi_err_intr_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, u32 mask,int enable);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */
extern void mdss_shdisp_video_transfer_ctrl(int onoff, int commit);
extern int mdss_shdisp_host_dsi_tx(int commit,
		struct shdisp_dsi_cmd_desc *shdisp_cmds, int size);
extern int mdss_shdisp_host_dsi_rx(struct shdisp_dsi_cmd_desc *cmds,
		unsigned char *rx_data, int rx_size);
#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
extern void mdss_dsi_pll_relock(struct mdss_dsi_ctrl_pdata *ctrl);
extern void mdss_dsi_dsc_config(struct mdss_dsi_ctrl_pdata *ctrl, struct dsc_desc *dsc);
extern void mdss_dsi_clkchg_phy_config(struct mdss_dsi_ctrl_pdata *ctrl);
#ifndef SHDISP_DISABLE_HR_VIDEO
extern int mdss_mdp_hr_video_suspend(struct mdss_mdp_ctl *ctl, int tg_en_flg);
extern int mdss_mdp_hr_video_resume(struct mdss_mdp_ctl *ctl, int tg_en_flg);
extern int mdss_mdp_hr_video_clkchg_mdp_update(struct mdss_mdp_ctl *ctl);
#else  /* SHDISP_DISABLE_HR_VIDEO */
extern int mdss_mdp_video_clkchg_mdp_update(struct mdss_mdp_ctl *ctl);
#endif /* SHDISP_DISABLE_HR_VIDEO */
extern void mdss_dsi_hs_clk_lane_enable(bool enable);
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_check_get_exec_state(void)
{
#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
	return mdss_diag_mipi_check_exec_state;
#else  /* MDSS_DIAG_MIPI_CHECK_ENABLE */
	return false;
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_check(struct mdp_mipi_check_param *mipi_check_param, struct mdss_panel_data *pdata)
{
#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
	int ret;
	u32 isr;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct mdss_dsi_ctrl_pdata *sctrl_pdata = NULL;
	#define SENSITIV_DATA_NUM		(2)
	uint8_t sens_data_master[SENSITIV_DATA_NUM];
	uint8_t sens_data_slave[SENSITIV_DATA_NUM];

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (pdata->next) {
		sctrl_pdata = container_of(pdata->next, struct mdss_dsi_ctrl_pdata, panel_data);
	}

	pr_debug("%s: in master=%p slave=%p\n", __func__, ctrl_pdata, sctrl_pdata);

	if (!ctrl_pdata) {
		pr_err("%s: ctrl_pdata is NULL.\n", __func__);
		return -ENXIO;
	}

	if (sctrl_pdata) {
		pr_err("%s: not support dual channel.\n", __func__);
		return -ENXIO;
	}

#ifndef SHDISP_DET_DSI_MIPI_ERROR
	mdss_dsi_err_intr_ctrl(ctrl_pdata, DSI_INTR_ERROR_MASK, 0);
	if (sctrl_pdata) {
		mdss_dsi_err_intr_ctrl(sctrl_pdata, DSI_INTR_ERROR_MASK, 0);
	}
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

	mdss_diag_mipi_check_exec_state = true;

	mdss_diag_dsi_cmd_bta_sw_trigger(ctrl_pdata);
	if (sctrl_pdata) {
		mdss_diag_dsi_cmd_bta_sw_trigger(sctrl_pdata);
	}

	mdss_diag_mipi_check_amp_data = 0;
	sens_data_master[0] = 0;
	sens_data_master[1] = 0;
	sens_data_slave[0] = 0;
	sens_data_slave[1] = 0;
	mdss_diag_mipi_check_get_param(
			&mdss_diag_mipi_check_amp_data,
			sens_data_master,
			sens_data_slave,
			ctrl_pdata);


	ret = mdss_diag_mipi_check_manual(
			mipi_check_param,
			ctrl_pdata,
			sctrl_pdata,
			sens_data_master,
			sens_data_slave);

	mdss_diag_mipi_check_set_param(
			mdss_diag_mipi_check_amp_data,
			sens_data_master,
			sens_data_slave,
			ctrl_pdata, sctrl_pdata);


	mdss_diag_mipi_check_exec_state = false;

	/* MMSS_DSI_0_INT_CTRL */
	isr = MIPI_INP(ctrl_pdata->ctrl_base + 0x0110);
	MIPI_OUTP(ctrl_pdata->ctrl_base + 0x0110, isr);
	if (sctrl_pdata) {
		isr = MIPI_INP(sctrl_pdata->ctrl_base + 0x0110);
		MIPI_OUTP(sctrl_pdata->ctrl_base + 0x0110, isr);
	}

#ifndef SHDISP_DET_DSI_MIPI_ERROR
	mdss_dsi_err_intr_ctrl(ctrl_pdata, DSI_INTR_ERROR_MASK, 1);
	if (sctrl_pdata) {
		mdss_dsi_err_intr_ctrl(sctrl_pdata, DSI_INTR_ERROR_MASK, 1);
	}
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

	pr_debug("%s: out\n", __func__);

	return ret;
#else  /* MDSS_DIAG_MIPI_CHECK_ENABLE */
	return 0;
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg(struct mdp_mipi_clkchg_param *mipi_clkchg_param)
{
#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
	int ret = 0;
	struct mdss_mdp_ctl *pctl;
	struct mdss_mdp_ctl *sctl;
	struct mdss_panel_data *pdata;
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
	struct shdisp_freq_params freq;
#endif  /* CONFIG_SHDISP_PANEL_HAYABUSA */

	pr_debug("%s: called\n", __func__);
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] mdpctrl is NULL.\n", __func__);
		return -EIO;
	}
	MDSS_XLOG(mipi_clkchg_param->host.clock_rate, XLOG_FUNC_ENTRY);
	pdata = pctl->panel_data;

	mdss_shdisp_lock_recovery();

	if (pctl->ops.wait_pingpong) {
		pr_debug("%s: wait_pingpong called\n", __func__);
		ret = pctl->ops.wait_pingpong(pctl, NULL);
		if(ret){
			pr_err("%s: failed to wait_pingpong. ret=%d\n", __func__, ret);
		}
	}
	sctl = mdss_mdp_get_split_ctl(pctl);
	if (sctl && sctl->ops.wait_pingpong) {
		ret = sctl->ops.wait_pingpong(sctl, NULL);
		if(ret){
			pr_err("%s: failed to wait_pingpong(split_ctl). ret=%d\n", __func__, ret);
		}
	}

	mdss_diag_mipi_clkchg_param_log(mipi_clkchg_param);

	mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata);

#if defined(CONFIG_SHDISP_PANEL_ANDY)
	shdisp_api_set_freq_param(&mipi_clkchg_param->panel);
#endif  /* defined(CONFIG_SHDISP_PANEL_ANDY) */
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
	freq.internal_osc = mipi_clkchg_param->internal_osc;
	shdisp_api_set_freq_param(&freq);
#endif  /* CONFIG_SHDISP_PANEL_HAYABUSA */

	if (mdss_shdisp_is_disp_on()) {
		ret = mdss_diag_mipi_clkchg_setparam(mipi_clkchg_param, pctl);
	} else {
		ret = mdss_diag_mipi_clkchg_panel_clk_data(pdata);
	}

	mdss_shdisp_unlock_recovery();

	pr_debug("%s: end ret(%d)\n", __func__, ret);
	MDSS_XLOG(XLOG_FUNC_EXIT);

	return ret;
#else  /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */
	return 0;
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */
}

#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_manual(struct mdp_mipi_check_param *mipi_check_param,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl,
			uint8_t *sensitiv_master, uint8_t *sensitiv_slave)
{
	int ret = 0;
	uint8_t result[2] = {MDSS_MIPICHK_RESULT_OK, MDSS_MIPICHK_RESULT_OK};
	uint8_t dummy[2] = {MDSS_MIPICHK_RESULT_OK, MDSS_MIPICHK_RESULT_OK};

	pr_debug("%s: in\n", __func__);

	if ((mipi_check_param->amp & ~(MDSS_MIPICHK_AMP_NUM - 1)) != 0) {
		pr_err("%s: out of range. amp=0x%02X\n", __func__, mipi_check_param->amp);
		return -EINVAL;
	}

	if ((mipi_check_param->sensitiv & ~(MDSS_MIPICHK_SENSITIV_NUM-1)) != 0) {
		pr_err("%s: out of range. sensitiv=0x%02X\n", __func__, mipi_check_param->sensitiv);
		return -EINVAL;
	}

	ret = mdss_diag_mipi_check_exec(result,
			mipi_check_param->frame_cnt, mipi_check_param->amp, mipi_check_param->sensitiv,
			ctrl, sctrl, sensitiv_master, sensitiv_slave);
	if (ret) {
		result[0] = MDSS_MIPICHK_RESULT_NG;
		result[1] = MDSS_MIPICHK_RESULT_NG;
	}

	if ((result[0] != MDSS_MIPICHK_RESULT_OK) || (result[1] != MDSS_MIPICHK_RESULT_OK)) {
		pr_debug("%s: recovery display.\n", __func__);
		mdss_diag_mipi_check_exec(dummy,
				1, MDSS_MIPICHK_AMP_NUM - 1, MDSS_MIPICHK_SENSITIV_NUM - 1,
				ctrl, sctrl, sensitiv_master, sensitiv_slave);
	}

	mipi_check_param->result_master = result[0];
	mipi_check_param->result_slave  = result[1];

	pr_debug("%s: out master=%d slave=%d\n", __func__,
				mipi_check_param->result_master, mipi_check_param->result_slave);

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_exec(uint8_t *result, uint8_t frame_cnt, uint8_t amp, uint8_t sensitiv,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl,
			uint8_t *sensitiv_master, uint8_t *sensitiv_slave)
{
	int ret = 0;
	uint8_t set_amp;
	uint8_t set_sensitiv_master[2] = {0, 0};
	uint8_t set_sensitiv_slave[2] = {0, 0};

	static const uint8_t amp_tbl[MDSS_MIPICHK_AMP_NUM] = {
		0x03,
		0x02,
		0x00,
		0x01,
		0x04,
		0x05,
		0x06,
		0x07
	};

	pr_debug("%s: in frame_cnt=0x%02X amp=0x%02X sensitiv=0x%02X\n", __func__, frame_cnt, amp, sensitiv);

	set_amp = (amp_tbl[amp] << 1) | 1;

#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
	set_sensitiv_master[0]  = sensitiv << 4;
	set_sensitiv_master[0] |= *(sensitiv_master) & 0x0F;
	if (sctrl) {
		set_sensitiv_slave[0]   = sensitiv << 4;
		set_sensitiv_slave[0]  |= *(sensitiv_slave) & 0x0F;
	}
#elif defined(CONFIG_SHDISP_PANEL_SAZABI)
	set_sensitiv_master[0]  = *(sensitiv_master);
	set_sensitiv_master[1]  = (sensitiv << 4) & 0x70;
	set_sensitiv_master[1] |= *(sensitiv_master + 1) & 0x8F;
	if (sctrl) {
		set_sensitiv_slave[0]  = *(sensitiv_slave);
		set_sensitiv_slave[1]  = (sensitiv << 4) & 0x70;
		set_sensitiv_slave[1] |= *(sensitiv_slave + 1) & 0x8F;
	}
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */

	mdss_diag_mipi_check_set_param(set_amp, set_sensitiv_master, set_sensitiv_slave, ctrl, sctrl);

	ret = mdss_diag_mipi_check_test(result, frame_cnt, ctrl, sctrl);

	pr_debug("%s: out ret=%d\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	pr_debug("%s: amp=0x%02X sensitiv_master=0x%02X,0x%02X sensitiv_slave=0x%02X,0x%02X\n", __func__,
			amp, sensitiv_master[0], sensitiv_master[1], sensitiv_slave[0], sensitiv_slave[1]);

	/* MMSS_DSI_0_PHY_REG_DSIPHY_REGULATOR_CTRL_0 */
	MIPI_OUTP((ctrl->phy_regulator_io.base) + MDSS_DSI_DSIPHY_REGULATOR_CTRL_0, amp);
	if (sctrl) {
		MIPI_OUTP((sctrl->phy_regulator_io.base) + MDSS_DSI_DSIPHY_REGULATOR_CTRL_0, amp);
	}
	wmb();

	mdss_diag_write_sensitiv(sensitiv_master, sensitiv_slave);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave,
			struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;

	/* MMSS_DSI_0_PHY_REG_DSIPHY_REGULATOR_CTRL_0 */
	*amp = MIPI_INP((ctrl->phy_regulator_io.base)+ MDSS_DSI_DSIPHY_REGULATOR_CTRL_0);

	ret = mdss_diag_read_sensitiv(sensitiv_master, sensitiv_slave);

	pr_debug("%s: amp=0x%02X sensitiv_master=0x%02X,0x%02X ret=%d\n", __func__, *amp,
			sensitiv_master[0], sensitiv_master[1], ret);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test(uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	int ret = 0;
	char mode;

	mode = ctrl->panel_mode;

	if (mode == DSI_VIDEO_MODE) {
		result[0] = mdss_diag_mipi_check_test_video(frame_cnt, ctrl);
	} else if (mode == DSI_CMD_MODE) {
		ret = mdss_diag_mipi_check_test_cmd(result, frame_cnt, ctrl, sctrl);
	} else {
		pr_err("%s: invalid panel_mode=%d\n", __func__, mode);
		ret = -EINVAL;
	}

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_video(uint8_t frame_cnt, struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;
	uint32_t sleep;

	sleep = frame_cnt * MDSS_DIAG_WAIT_1FRAME_US;
	pr_debug("%s: frame_cnt=%d sleep=%d\n", __func__, frame_cnt, sleep);


	pr_debug("%s: sleep start.\n",__func__);
	usleep(sleep);
	pr_debug("%s: sleep finish.\n",__func__);

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(ctrl);
	if (ret) {
		ret = MDSS_MIPICHK_RESULT_NG;
	} else {
		ret = MDSS_MIPICHK_RESULT_OK;
	}


	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_cmd(uint8_t *result, uint8_t frame_cnt,
			struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	int ret = 0;
	int ret2 = 0;
	int i;
	struct mdss_mdp_ctl *pctl;
	struct mdss_mdp_ctl *sctl;

	pr_debug("%s: in\n", __func__);

	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("%s: pctl is NULL.\n", __func__);
		return -EINVAL;
	}

	if (!pctl->ops.display_fnc) {
		pr_err("%s: display_fnc is NULL.\n", __func__);
		return -EINVAL;
	}

	sctl = mdss_mdp_get_split_ctl(pctl);

	for (i = 0; i < frame_cnt; i++) {
		pr_debug("%s: frame=%d\n", __func__, i);

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
			pr_err("%s: failed to display_fnc. ret=%d\n", __func__, ret);
			mutex_unlock(&pctl->lock);
			return ret;
		}

		mutex_unlock(&pctl->lock);
	}

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

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(ctrl);
	if (ret) {
		result[0] = MDSS_MIPICHK_RESULT_NG;
	} else {
		result[0] = MDSS_MIPICHK_RESULT_OK;
	}

	if (sctrl) {
		ret = mdss_diag_dsi_cmd_bta_sw_trigger(sctrl);
		if (ret) {
			result[1] = MDSS_MIPICHK_RESULT_NG;
		} else {
			result[1] = MDSS_MIPICHK_RESULT_OK;
		}
	}

	pr_debug("%s: out\n", __func__);

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_read_sensitiv(uint8_t *sensitiv_master, uint8_t *sensitiv_slave)
{
	int ret = 0;
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
	char payload_sensitiv[2][2] = {
		{0xFF, 0xE0},
		{0x7E, 0x00},
	};
	struct shdisp_dsi_cmd_desc sensitiv_read_cmd[] = {
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv[0]},
		{SHDISP_DTYPE_DCS_READ,   1, payload_sensitiv[1]},
	};

	mdss_shdisp_host_dsi_tx(1, &sensitiv_read_cmd[0], 1);

	ret = mdss_shdisp_host_dsi_rx(&sensitiv_read_cmd[1], sensitiv_master, 1);
#elif defined(CONFIG_SHDISP_PANEL_SAZABI)
	unsigned char read_buf[2] = {0x00, 0x00};

	char switchtopage0_payloads[] = {0xF0, 0x46, 0x23, 0x11, 0x01, 0x00};
	struct shdisp_dsi_cmd_desc switchtopage0_cmd[] = {
		{ SHDISP_DTYPE_GEN_LWRITE,  sizeof(switchtopage0_payloads), switchtopage0_payloads}
	};
	char sensitiv_payload[] = {0xB4};
	struct shdisp_dsi_cmd_desc sensitiv_read_cmd[] = {
		{SHDISP_DTYPE_GEN_READ2, sizeof(sensitiv_payload), sensitiv_payload}
	};

	mdss_shdisp_host_dsi_tx(1, switchtopage0_cmd, ARRAY_SIZE(switchtopage0_cmd));

	ret = mdss_shdisp_host_dsi_rx(sensitiv_read_cmd, read_buf, 2);
	memcpy(sensitiv_master, read_buf, sizeof(read_buf));
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_write_sensitiv(uint8_t *sensitiv_master, uint8_t *sensitiv_slave)
{
	int ret = 0;

#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
	char payload_sensitiv[2][2] = {
		{0xFF, 0xE0},
		{0x7E, 0x00},
	};

	struct shdisp_dsi_cmd_desc sensitiv_write_cmd[] = {
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv[0]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv[1]},
	};

	payload_sensitiv[1][1] = *(sensitiv_master);

	ret = mdss_shdisp_host_dsi_tx(1, sensitiv_write_cmd, ARRAY_SIZE(sensitiv_write_cmd));
#elif defined(CONFIG_SHDISP_PANEL_SAZABI)
	char switchtopage0_payloads[] = {0xF0, 0x46, 0x23, 0x11, 0x01, 0x00};
	struct shdisp_dsi_cmd_desc switchtopage0_cmd[] = {
		{ SHDISP_DTYPE_GEN_LWRITE,  sizeof(switchtopage0_payloads), switchtopage0_payloads}
	};
	char sensitiv_payload[] = {0xB4, 0x00, 0x00};
	struct shdisp_dsi_cmd_desc sensitiv_write_cmd[] = {
		{SHDISP_DTYPE_GEN_LWRITE, sizeof(sensitiv_payload), sensitiv_payload}
	};

	mdss_shdisp_host_dsi_tx(1, switchtopage0_cmd, ARRAY_SIZE(switchtopage0_cmd));

	sensitiv_payload[1] = *(sensitiv_master);
	sensitiv_payload[2] = *(sensitiv_master + 1);
    ret = mdss_shdisp_host_dsi_tx(1, sensitiv_write_cmd, ARRAY_SIZE(sensitiv_write_cmd));
#endif /* CONFIG_SHDISP_PANEL_HAYABUSA */
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;
	u32 status;
	int timeout_us = 35000;

	pr_debug("%s: in\n", __func__);

	if (ctrl == NULL) {
		pr_err("%s: ctrl is NULL.\n", __func__);
		return -EINVAL;
	}

	/* CMD_MODE_BTA_SW_TRIGGER */
	MIPI_OUTP((ctrl->ctrl_base) + 0x098, 0x01);	/* trigger */
	wmb();

	/* Check for CMD_MODE_DMA_BUSY */
	if (readl_poll_timeout(((ctrl->ctrl_base) + 0x0008),
				status, ((status & 0x0010) == 0),
				0, timeout_us)) {
		pr_info("%s: timeout. status=0x%08x\n", __func__, status);
		return -EIO;
	}

	ret = mdss_diag_dsi_ack_err_status(ctrl);

	pr_debug("%s: out status=0x%08x ret=%d\n", __func__, status, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 status;
	unsigned char *base;
	u32 ack = 0x10000000;

	base = ctrl->ctrl_base;

	status = MIPI_INP(base + 0x0068);/* DSI_ACK_ERR_STATUS */
	if (status) {
		MIPI_OUTP(base + 0x0068, status);
		/* Writing of an extra 0 needed to clear error bits */
		MIPI_OUTP(base + 0x0068, 0);

		status &= ~(ack);
		if(status){
			pr_err("%s: status=0x%08x\n", __func__, status);
			return -EIO;
		}
	}

	return 0;
}
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */

#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_setparam(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);
	if (pctl->is_video_mode) {
#ifndef SHDISP_DISABLE_HR_VIDEO
		ret |= mdss_mdp_hr_video_suspend(pctl, false);
#else  /* SHDISP_DISABLE_HR_VIDEO */
		mdss_shdisp_video_transfer_ctrl(false, false);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	} else {
		mdss_shdisp_clk_ctrl(true);
	}

	ret |= mdss_diag_mipi_clkchg_panel(mipi_clkchg_param, pctl);

	ret |= mdss_diag_mipi_clkchg_host(pctl);

	if (pctl->is_video_mode) {
#ifndef SHDISP_DISABLE_HR_VIDEO
		ret |= mdss_mdp_hr_video_resume(pctl, false);
#else  /* SHDISP_DISABLE_HR_VIDEO */
		mdss_shdisp_video_transfer_ctrl(true, false);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	} else {
		mdss_shdisp_clk_ctrl(false);
	}

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void __mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_panel_data *pdata)
{
	int i;
	struct mdss_panel_info *pinfo = &(pdata->panel_info);

	pr_debug("%s: called panel name = %s\n", __func__, pdata->panel_info.panel_name);

	pinfo->clk_rate = mipi_clkchg_param->host.clock_rate;
	pinfo->xres = mipi_clkchg_param->host.display_width;
	pinfo->yres = mipi_clkchg_param->host.display_height;
	pinfo->lcdc.h_pulse_width = mipi_clkchg_param->host.hsync_pulse_width;
	pinfo->lcdc.h_back_porch = mipi_clkchg_param->host.h_back_porch;
	pinfo->lcdc.h_front_porch = mipi_clkchg_param->host.h_front_porch;
	pinfo->lcdc.v_pulse_width = mipi_clkchg_param->host.vsync_pulse_width;
	pinfo->lcdc.v_back_porch = mipi_clkchg_param->host.v_back_porch;
	pinfo->lcdc.v_front_porch = mipi_clkchg_param->host.v_front_porch;
	pinfo->mipi.t_clk_post = mipi_clkchg_param->host.t_clk_post;
	pinfo->mipi.t_clk_pre = mipi_clkchg_param->host.t_clk_pre;
	for ( i=0; i<12; i++ ) {
		pinfo->mipi.dsi_phy_db.timing[i] = mipi_clkchg_param->host.timing_ctrl[i];
	}

	pr_debug("%s: end\n", __func__);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_panel_data *pdata)
{
	pr_debug("%s: called\n", __func__);

	__mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata);
	if (pdata->next) {
		__mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata->next);
	}

	pr_debug("%s: end\n", __func__);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int __mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl, struct mdss_panel_data *pdata)
{
	int ret = 0;

	pr_debug("%s: called panel name = %s\n", __func__, pdata->panel_info.panel_name);
	if (pctl->is_video_mode) {
#ifndef SHDISP_DISABLE_HR_VIDEO
		ret |= mdss_mdp_hr_video_clkchg_mdp_update(pctl);
#else  /* SHDISP_DISABLE_HR_VIDEO */
		ret |= mdss_mdp_video_clkchg_mdp_update(pctl);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	}
	ret |= mdss_diag_mipi_clkchg_panel_porch_update(pdata);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_stop_clkln_hs(struct mdss_panel_data *pdata) 
{
	int cnt = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	cnt = ctrl_pdata->clk_lane_cnt;
	if (cnt) {
		pr_debug("%s: ctrl_pdata(0x%p), clk_lane_cnt = %d\n", __func__, 
				ctrl_pdata, cnt);
		mdss_dsi_hs_clk_lane_enable(false);
	}
	return cnt;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_start_clkln_hs(struct mdss_panel_data *pdata, int cnt)
{
	if (cnt) {
		pr_debug("%s: clk_lane_cnt = %d\n", __func__,  cnt);
		mdss_dsi_hs_clk_lane_enable(true);
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl)
{
	int ret = 0;
	u32 ctl_flush;
	struct mdss_panel_data *pdata = pctl->panel_data;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int mpdata_clk_ln_hs_cnt = 0;

	pr_debug("%s: called\n", __func__);

	ret |= mdss_diag_mipi_clkchg_panel_clk_update(pdata);
	if (pdata->next) {
		ret |= mdss_diag_mipi_clkchg_panel_clk_update(pdata->next);
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mdss_dsi_pll_relock(ctrl_pdata);

	mpdata_clk_ln_hs_cnt = mdss_diag_mipi_stop_clkln_hs(pdata);

	ret |= __mdss_diag_mipi_clkchg_host(pctl, pdata);
	if (ret) {
		pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_host err panel name = %s.\n"
			, __func__, pdata->panel_info.panel_name);
	}
	if (pdata->next) {
		ret |= __mdss_diag_mipi_clkchg_host(pctl, pdata->next);
		if (ret) {
			pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_host err panel name = %s.\n"
				, __func__, pdata->next->panel_info.panel_name);
		}
	}

	mdss_diag_mipi_start_clkln_hs(pdata, mpdata_clk_ln_hs_cnt);

	ctl_flush = (BIT(31) >> (pctl->intf_num - MDSS_MDP_INTF0));
	ctl_flush |= (BIT(31) >> ((pctl->intf_num + 1) - MDSS_MDP_INTF0));
	mdss_mdp_ctl_write(pctl, MDSS_MDP_REG_CTL_FLUSH, ctl_flush);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_panel_clk_update(struct mdss_panel_data *pdata)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);

	ret = __mdss_diag_mipi_clkchg_panel_clk_data(pdata);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int __mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_panel_info *pinfo = &(pdata->panel_info);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pr_debug("%s: called\n", __func__);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	ret = mdss_dsi_clk_div_config(pinfo, pinfo->mipi.frame_rate);
	if (ret) {
		pr_err("LCDERR:[%s] mdss_dsi_clk_div_config err.\n", __func__);
		return ret;
	}
	pr_debug("%s: clk_rate = %d\n", __func__, pinfo->mipi.dsi_pclk_rate);
	ctrl_pdata->pclk_rate =
		pinfo->mipi.dsi_pclk_rate;
	ctrl_pdata->byte_clk_rate =
		pinfo->clk_rate / 8;

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);

	ret = __mdss_diag_mipi_clkchg_panel_clk_data(pdata);
	if (ret) {
		pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_panel_clk_data err panel name = %s.\n"
			, __func__, pdata->panel_info.panel_name);
	}
	if (pdata->next) {
		ret |= __mdss_diag_mipi_clkchg_panel_clk_data(pdata->next);
		if (ret) {
			pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_panel_clk_data err panel name = %s.\n"
				, __func__, pdata->next->panel_info.panel_name);
		}
	}

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_panel_porch_update(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = &(pdata->panel_info);
	struct mipi_panel_info *mipi;
	struct dsc_desc *dsc = NULL;
	u32 hbp, hfp, vbp, vfp, hspw, vspw, width, height;
	u32 ystride, bpp, data, dst_bpp;
	u32 stream_ctrl, stream_total;
	u32 dummy_xres, dummy_yres;
	u32 hsync_period, vsync_period;

	pr_debug("%s: called\n", __func__);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (pinfo->compression_mode == COMPRESSION_DSC)
		dsc = &pinfo->dsc;

	mdss_dsi_clkchg_phy_config(ctrl_pdata);

	dst_bpp = pdata->panel_info.fbc.enabled ?
		(pdata->panel_info.fbc.target_bpp) : (pinfo->bpp);

	hbp = pdata->panel_info.lcdc.h_back_porch;
	hfp = pdata->panel_info.lcdc.h_front_porch;
	vbp = pdata->panel_info.lcdc.v_back_porch;
	vfp = pdata->panel_info.lcdc.v_front_porch;
	hspw = pdata->panel_info.lcdc.h_pulse_width;
	vspw = pdata->panel_info.lcdc.v_pulse_width;
	width = mult_frac(pdata->panel_info.xres, dst_bpp,
			pdata->panel_info.bpp);
	height = pdata->panel_info.yres;
	pr_debug("%s: fbc=%d width=%d height=%d dst_bpp=%d\n", __func__,
			pdata->panel_info.fbc.enabled, width, height, dst_bpp);

	if (dsc)	/* compressed */
		width = dsc->pclk_per_line;

	if (pdata->panel_info.type == MIPI_VIDEO_PANEL) {
		dummy_xres = mult_frac((pdata->panel_info.lcdc.border_left +
				pdata->panel_info.lcdc.border_right),
				dst_bpp, pdata->panel_info.bpp);
		dummy_yres = pdata->panel_info.lcdc.border_top +
				pdata->panel_info.lcdc.border_bottom;
	}

	vsync_period = vspw + vbp + height + dummy_yres + vfp;
	hsync_period = hspw + hbp + width + dummy_xres + hfp;

	mipi  = &pdata->panel_info.mipi;
	if (pdata->panel_info.type == MIPI_VIDEO_PANEL) {
		if (ctrl_pdata->shared_data->timing_db_mode)
			MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x1e8, 0x1);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x24,
			((hspw + hbp + width + dummy_xres) << 16 |
			(hspw + hbp)));
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x28,
			((vspw + vbp + height + dummy_yres) << 16 |
			(vspw + vbp)));
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x2C,
				((vsync_period - 1) << 16)
				| (hsync_period - 1));

		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x30, (hspw << 16));
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x34, 0);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x38, (vspw << 16));
		if (ctrl_pdata->shared_data->timing_db_mode)
			MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x1e4, 0x1);

	} else {		/* command mode */
		if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB888)
			bpp = 3;
		else if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB666)
			bpp = 3;
		else if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB565)
			bpp = 2;
		else
			bpp = 3;	/* Default format set to RGB888 */

		ystride = width * bpp + 1;

		if (dsc) {
			stream_ctrl = ((dsc->bytes_in_slice + 1) << 16) |
					(mipi->vc << 8) | DTYPE_DCS_LWRITE;
			stream_total = dsc->pic_height << 16 |
							dsc->pclk_per_line;
		} else if (pinfo->partial_update_enabled &&
			mdss_dsi_is_panel_on(pdata) && pinfo->roi.w &&
			pinfo->roi.h) {
			stream_ctrl = (((pinfo->roi.w * bpp) + 1) << 16) |
					(mipi->vc << 8) | DTYPE_DCS_LWRITE;
			stream_total = pinfo->roi.h << 16 | pinfo->roi.w;
		} else {
			stream_ctrl = (ystride << 16) | (mipi->vc << 8) |
					DTYPE_DCS_LWRITE;
			stream_total = height << 16 | width;
		}

		/* DSI_COMMAND_MODE_NULL_INSERTION_CTRL */
		if (ctrl_pdata->shared_data->hw_rev >= MDSS_DSI_HW_REV_104_2) {
			data = (mipi->vc << 1); /* Virtual channel ID */
			data |= 0 << 16; /* Word count of the NULL packet */
			data |= 0x1; /* Enable Null insertion */
			MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x2b4, data);
		}

		/* Enable frame transfer in burst mode */
		if (ctrl_pdata->shared_data->hw_rev >= MDSS_DSI_HW_REV_103) {
			data = MIPI_INP(ctrl_pdata->ctrl_base + 0x1b8);
			data = data | BIT(16);
			MIPI_OUTP((ctrl_pdata->ctrl_base + 0x1b8), data);
			ctrl_pdata->burst_mode_enabled = 1;
		}

		/* DSI_COMMAND_MODE_MDP_STREAM_CTRL */
		data = (ystride << 16) | (mipi->vc << 8) | DTYPE_DCS_LWRITE;
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x60, data);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x58, data);

		/* DSI_COMMAND_MODE_MDP_STREAM_TOTAL */
		data = height << 16 | width;
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x64, data);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x5C, data);
	}

	if (dsc)	/* compressed */
		mdss_dsi_dsc_config(ctrl_pdata, dsc);

	mdss_dsi_sw_reset(ctrl_pdata, false);
	mdss_dsi_host_init(pdata);
	mdss_dsi_op_mode_config(mipi->mode, pdata);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_panel(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl)
{
	int ret = 0;
#if defined(CONFIG_SHDISP_PANEL_ANDY)
	static char mipi_sh_hayabusa_cmds_clkchgSetting[6][2] = {
		{0xFF, 0x05 },
		{0x90, 0x00 },
		{0x9B, 0x00 },
		{0xFF, 0x00 },
		{0xD3, 0x00 },
		{0xD4, 0x00 }
	};
	static struct shdisp_dsi_cmd_desc mipi_sh_hayabusa_cmds_clkchg[] = {
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[0]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[1]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[2]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[3]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[4]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[5]},
	};

	pr_debug("%s: called\n", __func__);

	mipi_sh_hayabusa_cmds_clkchgSetting[1][1] = mipi_clkchg_param->panel.hayabusa.rtn;

	mipi_sh_hayabusa_cmds_clkchgSetting[2][1] = mipi_clkchg_param->panel.hayabusa.gip;

	mipi_sh_hayabusa_cmds_clkchgSetting[4][1] = mipi_clkchg_param->panel.hayabusa.vbp;

	mipi_sh_hayabusa_cmds_clkchgSetting[5][1] = mipi_clkchg_param->panel.hayabusa.vfp;

	ret = mdss_shdisp_host_dsi_tx(1, mipi_sh_hayabusa_cmds_clkchg, ARRAY_SIZE(mipi_sh_hayabusa_cmds_clkchg));

	pr_debug("%s: end ret(%d)\n", __func__, ret);

#endif	/* CONFIG_SHDISP_PANEL_ANDY */
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_clkchg_param_log(struct mdp_mipi_clkchg_param *mdp_mipi_clkchg_param)
{
	pr_debug("[%s]param->host.clock_rate         = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.clock_rate          );
	pr_debug("[%s]param->host.display_width      = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.display_width       );
	pr_debug("[%s]param->host.display_height     = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.display_height      );
	pr_debug("[%s]param->host.hsync_pulse_width  = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.hsync_pulse_width   );
	pr_debug("[%s]param->host.h_back_porch       = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.h_back_porch        );
	pr_debug("[%s]param->host.h_front_porch      = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.h_front_porch       );
	pr_debug("[%s]param->host.vsync_pulse_width  = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.vsync_pulse_width   );
	pr_debug("[%s]param->host.v_back_porch       = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.v_back_porch        );
	pr_debug("[%s]param->host.v_front_porch      = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.v_front_porch       );
	pr_debug("[%s]param->host.t_clk_post         = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.t_clk_post          );
	pr_debug("[%s]param->host.t_clk_pre          = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.t_clk_pre           );
	pr_debug("[%s]param->host.timing_ctrl[ 0]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 0]     );
	pr_debug("[%s]param->host.timing_ctrl[ 1]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 1]     );
	pr_debug("[%s]param->host.timing_ctrl[ 2]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 2]     );
	pr_debug("[%s]param->host.timing_ctrl[ 3]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 3]     );
	pr_debug("[%s]param->host.timing_ctrl[ 4]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 4]     );
	pr_debug("[%s]param->host.timing_ctrl[ 5]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 5]     );
	pr_debug("[%s]param->host.timing_ctrl[ 6]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 6]     );
	pr_debug("[%s]param->host.timing_ctrl[ 7]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 7]     );
	pr_debug("[%s]param->host.timing_ctrl[ 8]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 8]     );
	pr_debug("[%s]param->host.timing_ctrl[ 9]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[ 9]     );
	pr_debug("[%s]param->host.timing_ctrl[10]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[10]     );
	pr_debug("[%s]param->host.timing_ctrl[11]    = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.timing_ctrl[11]     );

#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
	pr_debug("[%s]param->internal_osc            = %10d\n"  , __func__, mdp_mipi_clkchg_param->internal_osc             );
#endif  /* CONFIG_SHDISP_PANEL_HAYABUSA */

	return;
}
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

