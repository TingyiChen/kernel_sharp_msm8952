/* drivers/sharp/shdisp/shdisp_panel.c  (Display Driver)
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
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#ifdef CONFIG_TOUCHSCREEN_SHTPS
#include <sharp/shtps_dev.h>
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/wakelock.h>

#ifdef CONFIG_SHTERM
#include <sharp/shterm_k.h>
#endif /* CONFIG_SHTERM */
#include <sharp/sh_smem.h>
#include "shdisp_system.h"
#include "shdisp_dbg.h"
#include "shdisp_io.h"
#include "shdisp_kerl_priv.h"

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/delay.h>
#include <asm/param.h>


#include "shdisp_panel.h"
#if defined(CONFIG_SHDISP_PANEL_CARIN)
#include "shdisp_carin.h"
#elif defined(CONFIG_SHDISP_PANEL_ANDY)
#include "shdisp_andy.h"
#elif defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#include "shdisp_hayabusa.h"
#elif defined(CONFIG_SHDISP_PANEL_SAZABI)
#include "shdisp_sazabi.h"
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
#include "shdisp_aria.h"
#else   /* defined(CONFIG_SHDISP_PANEL_CARIN) */
#include "shdisp_null_panel.h"
#endif  /* defined(CONFIG_SHDISP_PANEL_CARIN) */

#define MIPI_DSI_SHORT_PACKET_LEN            (8)
#define MIPI_DSI_READ_RESPONSE_LEN           (8)
#define SHDISP_INVALID_GPIO                  (-1)

static struct shdisp_panel_operations *shdisp_panel_fops = NULL;

static struct shdisp_panel_operations shdisp_def_fops = {
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
extern int mdss_shdisp_host_dsi_tx(int commit, struct shdisp_dsi_cmd_desc *shdisp_cmds, int size);
extern int mdss_shdisp_host_dsi_rx(struct shdisp_dsi_cmd_desc *cmds, unsigned char *rx_data, int rx_size);

static void shdisp_panel_dsi_wlog(struct shdisp_dsi_cmd_desc *cmds, int cmdslen);
static void shdisp_panel_dsi_rlog(char addr, char *rbuf, int len);
static int shdisp_panel_dsi_mask_write_reg(struct shdisp_panel_regsetting *req/*, unsinged int size*/);
static int shdisp_panel_mipi_dsi_cmds_tx(int commit, struct shdisp_dsi_cmd_desc *cmds, int cnt);
static int shdisp_panel_mipi_dsi_cmds_rx(unsigned char *rbuf, struct shdisp_dsi_cmd_desc *cmds, unsigned char size);
static void shdisp_panel_workqueue_handler(struct work_struct *work);
static irqreturn_t shdisp_panel_det_isr(int irq, void * data);
static int shdisp_panel_set_irq(int enable);
static int shdisp_panel_det_probe(struct platform_device *pdev);
static int shdisp_panel_det_remove(struct platform_device *pdev);
static int shdisp_panel_det_register_driver(void);

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct workqueue_struct    *shdisp_wq_panel = NULL;
static struct work_struct         shdisp_wq_panel_wk;
static struct wake_lock shdisp_panel_wakelock;
static int shdisp_panel_det_gpio = 0;
static int shdisp_panel_det_irq = SHDISP_INVALID_GPIO;

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define MIPI_SHARP_RW_MAX_SIZE          SHDISP_LCDDR_BUF_MAX
#define MIPI_SHARP_R_SIZE               (10)

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_panel_dsi_wlog                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_panel_dsi_wlog(struct shdisp_dsi_cmd_desc *cmds, int cmdslen)
{
#ifdef SHDISP_LOG_ENABLE
    char buf[128];
    char *pbuf;
    int cmdcnt;
    int arylen;
    int writelen;

    for (cmdcnt = 0; cmdcnt != cmdslen; cmdcnt++) {
        int i;
        char dtype = cmds[cmdcnt].dtype;
        short payloadlen = cmds[cmdcnt].dlen;
        unsigned char *payload = (unsigned char *)cmds[cmdcnt].payload;

        pbuf = buf;
        arylen = sizeof(buf) / sizeof(*buf);

        writelen = snprintf(pbuf, arylen, "dtype= %02X, ", dtype);
        arylen -= writelen;
        pbuf += writelen;

        writelen = snprintf(pbuf, arylen, "payload= %02X ", payload[0]);
        arylen -= writelen;
        pbuf += writelen;

        for (i = 1; i != payloadlen; ++i) {
            if ((!((i - 1) % 16)) && (i != 1)) {
                int spacecnt = 23;
                *pbuf = '\0';
                SHDISP_DSI_LOG("%s", buf);

                arylen = sizeof(buf) / sizeof(*buf);
                pbuf = buf;
                memset(pbuf, ' ', spacecnt);
                arylen -= spacecnt;
                pbuf += spacecnt;
            }
            writelen = snprintf(pbuf, arylen, "%02X ", payload[i]);
            arylen -= writelen;
            pbuf += writelen;
        }

        *pbuf = '\0';
        SHDISP_DSI_LOG("%s", buf);
    }
#endif /* SHDISP_LOG_ENABLE */
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_dsi_rlog                                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_panel_dsi_rlog(char addr, char *rbuf, int len)
{
#ifdef SHDISP_LOG_ENABLE
    char buf[128];
    char *pbuf;
    unsigned char *prbuf = (unsigned char *)rbuf;
    int arylen;
    int writelen;
    int i = 0;

    arylen = sizeof(buf) / sizeof(*buf);
    pbuf = buf;

    writelen = snprintf(pbuf, arylen, "addr = %02X, val = ", (unsigned char)addr);
    arylen -= writelen;
    pbuf += writelen;

    for (i = 0; i != len; ++i) {
        if ((!(i % 16)) && (i)) {
            int spacecnt = 17;
            *pbuf = '\0';
            SHDISP_DEBUG("%s", buf);

            arylen = sizeof(buf) / sizeof(*buf);
            pbuf = buf;
            memset(pbuf, ' ', spacecnt);
            arylen -= spacecnt;
            pbuf += spacecnt;
        }
        writelen = snprintf(pbuf, arylen, "%02X ", *prbuf);
        arylen -= writelen;
        pbuf += writelen;
        prbuf++;
    }

    *pbuf = '\0';
    SHDISP_DSI_LOG("%s", buf);
#endif /* SHDISP_LOG_ENABLE */
}

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_create                                                   */
/* ------------------------------------------------------------------------- */
void shdisp_panel_API_create(void)
{
    shdisp_panel_fops = &shdisp_def_fops;

#if defined(CONFIG_SHDISP_PANEL_CARIN)
    shdisp_panel_fops = shdisp_carin_API_create();
#elif defined(CONFIG_SHDISP_PANEL_ANDY)
    shdisp_panel_fops = shdisp_andy_API_create();
#elif defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    shdisp_panel_fops = shdisp_hayabusa_API_create();
#elif defined(CONFIG_SHDISP_PANEL_SAZABI)
    shdisp_panel_fops = shdisp_sazabi_API_create();
#elif defined(CONFIG_SHDISP_PANEL_ARIA)
    shdisp_panel_fops = shdisp_aria_API_create();
#else   /* defined(CONFIG_SHDISP_PANEL_CARIN) */
    shdisp_panel_fops = shdisp_null_panel_API_create();
#endif  /* defined(CONFIG_SHDISP_PANEL_CARIN) */
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_init_io                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_init_io(struct shdisp_panel_context *panel_ctx)
{
    shdisp_wq_panel = create_singlethread_workqueue("shdisp_panel_queue");
    if (!shdisp_wq_panel) {
        SHDISP_ERR("failed to create_singlethread_workqueue().");
        destroy_workqueue(shdisp_wq_panel);
        shdisp_wq_panel = NULL;
    }

    INIT_WORK(&shdisp_wq_panel_wk, shdisp_panel_workqueue_handler);
    wake_lock_init(&shdisp_panel_wakelock, WAKE_LOCK_SUSPEND, "panel_wake_lock");
    shdisp_panel_det_register_driver();
    if (shdisp_panel_fops->init_io) {
        return shdisp_panel_fops->init_io(panel_ctx);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_exit_io                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_exit_io(void)
{
    if (shdisp_panel_fops->exit_io) {
        return shdisp_panel_fops->exit_io();
    }
    if (shdisp_wq_panel) {
        flush_workqueue(shdisp_wq_panel);
        destroy_workqueue(shdisp_wq_panel);
        shdisp_wq_panel = NULL;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_power_on                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_power_on(int mode)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->power_on) {
        SHDISP_DEBUG("out1");
        return shdisp_panel_fops->power_on(mode);
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_power_off                                                */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_power_off(int mode)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->power_off) {
        SHDISP_DEBUG("out1");
        return shdisp_panel_fops->power_off(mode);
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_disp_on                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_disp_on(void)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->disp_on) {
        SHDISP_DEBUG("out1");
        return shdisp_panel_fops->disp_on();
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_disp_off                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_disp_off(void)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->disp_off) {
        SHDISP_DEBUG("out1");
        return shdisp_panel_fops->disp_off();
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_write_reg                                           */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    if (shdisp_panel_fops->write_reg) {
        return shdisp_panel_fops->write_reg(addr, write_data, size);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_read_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    if (shdisp_panel_fops->read_reg) {
        return shdisp_panel_fops->read_reg(addr, read_data, size);
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_set_flicker_param                                   */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_set_flicker_param(struct shdisp_diag_flicker_param vcom)
{
    if (shdisp_panel_fops->set_flicker) {
        return shdisp_panel_fops->set_flicker(vcom);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_get_flicker_param                                   */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *vcom)
{
    if (shdisp_panel_fops->get_flicker) {
        return shdisp_panel_fops->get_flicker(vcom);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_get_flicker_low_param                               */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom)
{
    if (shdisp_panel_fops->get_flicker_low) {
        return shdisp_panel_fops->get_flicker_low(vcom);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_mipi_diag_write_reg                                      */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_mipi_diag_write_reg(char dtype, unsigned char addr, char *write_data, unsigned char size)
{
    int retry = 5;
    int ret;
    struct shdisp_dsi_cmd_desc cmd[1];
    char cmd_buf[MIPI_SHARP_RW_MAX_SIZE + 1];

    if (size > MIPI_SHARP_RW_MAX_SIZE) {
        SHDISP_ERR("size over, -EINVAL");
        return -EINVAL;
    }

    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;
    memcpy(&cmd_buf[1], write_data, size);

    cmd[0].dtype = dtype;
    cmd[0].mode = 0;
    cmd[0].wait = 0x00;
    cmd[0].dlen = size + 1;
    cmd[0].payload = cmd_buf;

    for (; retry >= 0; retry--) {
        ret = shdisp_panel_mipi_dsi_cmds_tx(1, cmd, ARRAY_SIZE(cmd));
        if (ret == SHDISP_RESULT_SUCCESS) {
            break;
        } else {
            SHDISP_WARN("shdisp_panel_mipi_dsi_cmds_tx() failure. ret=%d retry=%d", ret, retry);
        }
    }
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error");
        return -ENODEV;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_mipi_diag_read_reg                                       */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_mipi_diag_read_reg(char dtype, unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int retry = 5;
    int ret;
#ifdef SHDISP_LOG_ENABLE
    int i;
    unsigned char rlen;
#endif  /* SHDISP_LOG_ENABLE */
    struct shdisp_dsi_cmd_desc cmd[1];
    char cmd_buf[2 + 1];

    SHDISP_TRACE("in address:%02X, buf:%p, size:%d", addr, read_data, size);
    if ((size > MIPI_SHARP_RW_MAX_SIZE) || (size == 0)) {
        SHDISP_ERR("size over, -EINVAL");
        return -EINVAL;
    }

    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;

    cmd[0].dtype = dtype;
    cmd[0].wait     = 0x00;
    cmd[0].mode = 0;
    cmd[0].dlen     = 1;
    cmd[0].payload  = cmd_buf;

    memset(read_data, 0, size);

    for (; retry >= 0; retry--) {
        ret = shdisp_panel_mipi_dsi_cmds_rx(read_data, cmd, size);
        if (ret == SHDISP_RESULT_SUCCESS) {
            break;
        } else {
            SHDISP_WARN("shdisp_panel_mipi_dsi_cmds_rx() failure. ret=%d retry=%d", ret, retry);
        }
    }
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
        return -ENODEV;
    }

#ifdef SHDISP_LOG_ENABLE
    rlen = size;
    for (i = 0; i < rlen; i++) {
        if ((i % MIPI_DSI_SHORT_PACKET_LEN) == 0) {
            SHDISP_DEBUG(" ");
            SHDISP_PRINTK(SHDISP_LOG_LV_DEBUG, "[SHDISP_DEBUG][%s] Data    : ", __func__);
        }
        SHDISP_PRINTK(SHDISP_LOG_LV_DEBUG, "%02X ", read_data[i]);
    }
    SHDISP_DEBUG(" ");
#endif  /* SHDISP_LOG_ENABLE */

    SHDISP_TRACE("out SHDISP_RESULT_SUCCESS");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_dsi_write_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_dsi_write_reg(struct shdisp_dsi_cmd_req *req)
{
    int retry = 5;
    int ret;
    char dtype;
    struct shdisp_dsi_cmd_desc cmd[1];
    char cmd_buf[MIPI_SHARP_RW_MAX_SIZE + 1];

    if (req->size > MIPI_SHARP_RW_MAX_SIZE) {
        SHDISP_ERR("size over, -EINVAL");
        return SHDISP_RESULT_FAILURE;
    }

    memset(cmd_buf, 0x00, sizeof(cmd_buf));
    cmd_buf[0] = req->addr;
    cmd_buf[1] = 0x00;
    memcpy(&cmd_buf[1], req->data, req->size);

    if (req->dtype != SHDISP_DTYPE_GEN_WRITE) {
        if (req->size == 0) { /* 0 parameters */
            dtype = SHDISP_DTYPE_DCS_WRITE;
        } else if (req->size == 1) { /* 1 parameter */
            dtype = SHDISP_DTYPE_DCS_WRITE1;
        } else { /* Many parameters */
            dtype = SHDISP_DTYPE_DCS_LWRITE;
        }
    } else {
        if (req->size == 0) { /* 0 parameters */
            dtype = SHDISP_DTYPE_GEN_WRITE;
        } else if (req->size == 1) { /* 1 parameter */
            dtype = SHDISP_DTYPE_GEN_WRITE1;
        } else if (req->size == 2) { /* 2 parameters */
            dtype = SHDISP_DTYPE_GEN_WRITE2;
        } else { /* Many parameters */
            dtype = SHDISP_DTYPE_GEN_LWRITE;
        }
    }

    cmd[0].dtype = dtype;
    cmd[0].mode = req->mode;
    cmd[0].wait = 0x00;
    cmd[0].dlen = req->size + 1;
    cmd[0].payload = cmd_buf;

    for (; retry >= 0; retry--) {
        ret = shdisp_panel_mipi_dsi_cmds_tx(1, cmd, ARRAY_SIZE(cmd));
        if (ret == SHDISP_RESULT_SUCCESS) {
            break;
        } else {
            SHDISP_WARN("shdisp_panel_mipi_dsi_cmds_tx() failure. ret=%d retry=%d", ret, retry);
        }
    }

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error");
        return SHDISP_RESULT_FAILURE; 
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_dsi_read_reg                                             */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_dsi_read_reg(struct shdisp_dsi_cmd_req *req)
{
    int retry = 5;

    int ret;
    char dtype;
    struct shdisp_dsi_cmd_desc cmd[1];
    char cmd_buf[2 + 1];

    SHDISP_TRACE("in address:%02X, buf:%p, size:%d", req->addr, req->data, req->size);
    if ((req->size > MIPI_SHARP_RW_MAX_SIZE) || (req->size == 0)) {
        SHDISP_ERR("size over, -EINVAL");
        return SHDISP_RESULT_FAILURE;
    }

    cmd_buf[0] = req->addr;
    cmd_buf[1] = 0x00;

    if (req->dtype != SHDISP_DTYPE_GEN_READ) {
        dtype = SHDISP_DTYPE_DCS_READ;
    } else {
        if (req->size == 1) { /* 0 parameters */
            dtype = SHDISP_DTYPE_GEN_READ;
        } else if (req->size == 2) { /* 1 parameter */
            dtype = SHDISP_DTYPE_GEN_READ1;
        } else { /* 2 paramters */
            dtype = SHDISP_DTYPE_GEN_READ2;
        }
    }
    cmd[0].dtype    = dtype;
    cmd[0].mode     = req->mode;
    cmd[0].wait     = 0x00;
    cmd[0].dlen     = 1;
    cmd[0].payload  = cmd_buf;

    for (; retry >= 0; retry--) {
        ret = shdisp_panel_mipi_dsi_cmds_rx((unsigned char *)req->data, cmd, req->size);
        if (ret == SHDISP_RESULT_SUCCESS) {
            break;
        } else {
            SHDISP_WARN("shdisp_panel_mipi_dsi_cmds_rx() failure. ret=%d retry=%d", ret, retry);
        }
    }

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out SHDISP_RESULT_SUCCESS");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_dsi_mask_write_reg                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_dsi_mask_write_reg(struct shdisp_panel_regsetting *req/*, unsinged int size*/)
{
    struct shdisp_dsi_cmd_desc cmd;
    char payload[1 + SHDISP_LCDDR_BUF_MAX];
    int i;
    int ret = SHDISP_RESULT_SUCCESS;

    if (!req || !(req->data) || !(req->mask) || ((req->size)>SHDISP_LCDDR_BUF_MAX)) {
    	SHDISP_ERR("req is null.");
    	return SHDISP_RESULT_FAILURE;
    }

    memset(&cmd, 0, sizeof(cmd));
    memset(payload, 0, ARRAY_SIZE(payload));
    
    payload[0]  = req->addr;
    cmd.dtype   = req->r_dtype;
    cmd.dlen    = 1;
    cmd.payload = payload;
    cmd.wait    = 0x00;
    cmd.mode    = 0;

    if (shdisp_panel_mipi_dsi_cmds_rx((unsigned char *)payload+1, &cmd, req->size) != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
        ret = SHDISP_RESULT_FAILURE;
    } else {
        for (i = 0; i != req->size; ++i) {
                unsigned char rdata = payload[i+1];
                unsigned char mask  = req->mask[i];
                payload[i+1] = (rdata & ~(mask)) | (req->data[i] & mask);
                SHDISP_DEBUG("param[%d], read=0x%02x, mask=0x%02x, write = 0x%02x", i, rdata, mask, payload[i+1]);
        }
        cmd.dtype   = req->w_dtype;
        cmd.dlen    = req->size + 1;
        ret = shdisp_panel_mipi_dsi_cmds_tx(1, &cmd, 1);
    }
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_dsi_mask_write_reg                                       */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_dsi_mask_write_reg(struct shdisp_panel_regsetting *req/*, unsinged int size*/)
{
	return shdisp_panel_dsi_mask_write_reg(req);
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_start_display                                            */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_start_display(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    if (shdisp_panel_fops->start_display) {
        return shdisp_panel_fops->start_display();
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_post_video_start                                         */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_post_video_start(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");
    if (shdisp_panel_fops->post_video_start) {
        return shdisp_panel_fops->post_video_start();
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_check_recovery                                           */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_check_recovery(void)
{
    if (shdisp_panel_fops->check_recovery) {
        return shdisp_panel_fops->check_recovery();
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_set_gmmtable_and_voltage                            */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    if (shdisp_panel_fops->set_gmmtable_and_voltage) {
        return shdisp_panel_fops->set_gmmtable_and_voltage(gmm_info);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_get_gmmtable_and_voltage                            */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    if (shdisp_panel_fops->get_gmmtable_and_voltage) {
        return shdisp_panel_fops->get_gmmtable_and_voltage(gmm_info);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_set_gmm                                             */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_diag_set_gmm(struct shdisp_diag_gamma *gmm)
{
    if (shdisp_panel_fops->set_gmm) {
        return shdisp_panel_fops->set_gmm(gmm);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_shutdown                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_shutdown(void)
{
    int ret = 0;
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->shutdown) {
        ret = shdisp_panel_fops->shutdown();
        SHDISP_DEBUG("out ret=%d", ret);
        return ret;
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_dump_reg                                                 */
/* ------------------------------------------------------------------------- */
void shdisp_panel_API_dump(int type)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->dump) {
        shdisp_panel_fops->dump(type);
    }
    SHDISP_TRACE("out");
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_set_irq                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_set_irq(int enable)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_panel_set_irq(enable);

    if (shdisp_panel_fops->set_irq) {
        ret = shdisp_panel_fops->set_irq(enable);
        SHDISP_DEBUG("out ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_mfr                                                      */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_mfr(int mfr)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->set_mfr) {
        return shdisp_panel_fops->set_mfr(mfr);
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_chg_mode                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_chg_mode(struct shdisp_panel_mode p_mode)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->chg_mode) {
        return shdisp_panel_fops->chg_mode(p_mode);
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_set_freq_param                                           */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_set_freq_param(struct shdisp_freq_params *freq)
{
    SHDISP_TRACE("in");
    if (shdisp_panel_fops->set_freq_param) {
        return shdisp_panel_fops->set_freq_param(freq);
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_mipi_dsi_cmds_tx                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_mipi_dsi_cmds_tx(int commit, struct shdisp_dsi_cmd_desc *cmds, int cnt)
{
    int ret;
    SHDISP_TRACE("in cnt=%d", cnt);
    shdisp_panel_dsi_wlog(cmds, cnt);
    ret = mdss_shdisp_host_dsi_tx(commit, cmds, cnt );
    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_mipi_dsi_cmds_rx                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_mipi_dsi_cmds_rx(unsigned char *rbuf, struct shdisp_dsi_cmd_desc *cmds, unsigned char size)
{
    int ret;
    SHDISP_TRACE("in size:%d", size);
    shdisp_panel_dsi_wlog(cmds, 1);
    ret = mdss_shdisp_host_dsi_rx(cmds, rbuf, size);
    shdisp_panel_dsi_rlog(cmds->payload[0], (char *)rbuf, size);
    SHDISP_TRACE("out ret:%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_mipi_dsi_cmds_tx                                         */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_mipi_dsi_cmds_tx(int commit, struct shdisp_dsi_cmd_desc *cmds, int cnt)
{
    return shdisp_panel_mipi_dsi_cmds_tx(commit, cmds, cnt);
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_mipi_dsi_cmds_rx                                         */
/* ------------------------------------------------------------------------- */
int shdisp_panel_API_mipi_dsi_cmds_rx(unsigned char *rbuf, struct shdisp_dsi_cmd_desc *cmds, unsigned char size)
{
    return shdisp_panel_mipi_dsi_cmds_rx(rbuf, cmds, size);
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_workqueue_handler                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_panel_workqueue_handler(struct work_struct *work)
{
    int ret;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in");

    if (shdisp_panel_check_det() != SHDISP_RESULT_SUCCESS) {

#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_ESD_DETIN;
        shdisp_dbg_API_err_output(&err_code, 0);
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_ESD_DETIN);
#endif /* SHDISP_RESET_LOG */

        ret = shdisp_API_do_lcd_det_recovery();
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("recovery request error!! ret=%d", ret);
        }
    } else {
        enable_irq(shdisp_panel_det_irq);
    }

    wake_unlock(&shdisp_panel_wakelock);

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_det_isr                                                      */
/* ------------------------------------------------------------------------- */
static irqreturn_t shdisp_panel_det_isr(int irq, void * data)
{
    int ret;

    SHDISP_TRACE("in irq=%d", irq);

    if (shdisp_API_is_lcd_det_recovering()) {
        SHDISP_WARN("now recovering...");
        goto exit;
    }

    disable_irq_nosync(shdisp_panel_det_irq);

    if (!shdisp_wq_panel) {
        SHDISP_ERR("invalid work queue. wq=%p", shdisp_wq_panel);
        goto exit;
    }

    ret = shdisp_api_get_main_disp_status();
    if (ret == SHDISP_MAIN_DISP_OFF) {
        SHDISP_DEBUG("display OFF, will be exited.");
        goto exit;
    }

    wake_lock(&shdisp_panel_wakelock);

    ret = queue_work(shdisp_wq_panel, &shdisp_wq_panel_wk);
    if (ret == 0) {
        wake_unlock(&shdisp_panel_wakelock);
        SHDISP_DEBUG("failed to queue_work(). ret=%d", ret);
    }

exit:
    SHDISP_TRACE("out");
    return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_set_irq                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_set_irq(int enable)
{
    int ret = SHDISP_RESULT_SUCCESS;

    if (shdisp_panel_det_irq == SHDISP_INVALID_GPIO) {
        SHDISP_DEBUG("invalid gpio.");
        return SHDISP_RESULT_SUCCESS;
    }

    switch (enable) {
    case SHDISP_IRQ_ENABLE:
        ret = request_irq(shdisp_panel_det_irq, shdisp_panel_det_isr,
                        IRQF_TRIGGER_FALLING, "shdisp_paneldet", NULL);
        if (ret) {
            ret = SHDISP_RESULT_FAILURE;
            SHDISP_ERR("failed to request_irq(). (ret=%d irq=%d)", ret, shdisp_panel_det_irq);
        }
        break;
    case SHDISP_IRQ_DISABLE:
        disable_irq(shdisp_panel_det_irq);
        free_irq(shdisp_panel_det_irq, NULL);
        break;
    default:
        ret = SHDISP_RESULT_FAILURE;
        SHDISP_ERR("invalid argument. (enable=%d)", enable);
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_det_probe                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_det_probe(struct platform_device *pdev)
{
    struct resource * pr;

    SHDISP_TRACE("in pdev=0x%p.", pdev);

    if (pdev == NULL) {
        SHDISP_ERR("pdev is NULL");
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_panel_det_gpio = of_get_named_gpio(pdev->dev.of_node, "lcd_det_gpio", 0);
    if (!gpio_is_valid(shdisp_panel_det_gpio)) {
        SHDISP_ERR("lcd det gpio not specified");
    } else {
        SHDISP_DEBUG("lcd det gpio succusess!");
    }

    pr = platform_get_resource_byname(pdev, IORESOURCE_IRQ, 
                                  "shdisp_panel_det");
    if (pr == NULL) {
        SHDISP_ERR("irq resouce err!!");
        return SHDISP_RESULT_FAILURE;
    } else {
        shdisp_panel_det_irq = pr->start;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_det_remove                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_det_remove(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_panel_det_dt_match[] = {
    { .compatible = "sharp,shdisp_panel_det", },
    {}
};
#else
#define shdisp_panel_det_dt_match NULL;
#endif /* CONFIG_OF */

static struct platform_driver shdisp_panel_det_driver = {
    .probe = shdisp_panel_det_probe,
    .remove = shdisp_panel_det_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_panel_det",
        .of_match_table = shdisp_panel_det_dt_match,
    },
};

/* ------------------------------------------------------------------------- */
/* shdisp_panel_det_register_driver                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_det_register_driver(void)
{
    int ret;
    SHDISP_TRACE("in.");
    ret = platform_driver_register(&shdisp_panel_det_driver);
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_check_det                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_panel_check_det(void)
{
    SHDISP_TRACE("in");

    if (!gpio_is_valid(shdisp_panel_det_gpio)) {
        SHDISP_DEBUG("lcd det gpio not specified");
        return SHDISP_RESULT_SUCCESS;
    }

    if (!shdisp_IO_API_get_Host_gpio(shdisp_panel_det_gpio)) {
        SHDISP_DEBUG("out DET is LOW.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
