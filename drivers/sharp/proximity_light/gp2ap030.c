/* drivers/input/misc/gp2ap030.c - GP2AP030*00F v1.0.2ambient light sensor and proximity sensor driver
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
 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/input/gp2ap030.h>
#include <sharp/sh_smem.h>
#include <sharp/sh_boot_manager.h>

#define DEVICE_NAME 			"GP2AP03000F"
#define CLASS_NAME				"gp2ap030"
#define LIGHT_SENSOR_NAME		"light_sensor"
#define PROXIMITY_SENSOR_NAME	"proximity_sensor"

#define PSALS_RECOVERY_ENABLE

#define SENSOR_DEFAULT_DELAY	( 200 )			/* 200 ms */
#define SENSOR_MAX_DELAY		( 5000 )		/* 5000 ms */

#define ALS_AVG_READ_COUNT		( 10 )			/* sensor read count */
#define ALS_AVG_READ_INTERVAL	( 20 * 1000 )	/* 20 ms */

#define SENSOR_REGL_OFF_DELAY	( 1  * 1000 )	/* 1 ms */
#define SENSOR_REGL_ON_DELAY	( 5  * 1000 )	/* 5 ms */
#define SENSOR_ALS_PALS_DELAY	( 35 * 1000 )	/* 35 ms */
#define SENSOR_PS_PALS_DELAY	( 35 * 1000 )	/* 35 ms */
#define REG_ADR00_READ_COUNT	( 4 )			/* REG_ADR00 read count */
#define RATIO_OF_ALS0			( 64 / 4 )
#define RATIO_OF_ALS1			( 60 / 4 )
#define ADO_MAX					( 65535 )
#define GAMM_ADJUST_MAX			( 32 )
#define ALPH_ADJ0_VAL			( 0x3F5C )
#define ALPH_ADJ1_VAL			( 0x0CAC )
#define BETA_ADJ0_VAL			( 0x3F5C )
#define BETA_ADJ1_VAL			( 0x0CAC )
#define GAMM_ADJ0_VAL			( 0x03 )
#define GAMM_ADJ1_VAL			( 0x03 )
#define AVE_ADO_READ_TIMES		( 5 )

#define LOW_LUX_MODE			( 0 )
#define HIGH_LUX_MODE			( 1 )

#define CALIBRATION_ONOFF		( 1 )
#define SET_TH					( 20 )

#define SENSOR_ALS_RANGE_DELAY_MS		( 40 )		/* 40  ms */
#define SENSOR_ALS_RANGE_NO_DELAY		( 0 )		/* 0   ms */
#define SENSOR_PS_RANGE_DELAY_MS		( 100 )		/* 100 ms */
#define SENSOR_PALS_UNIT_SHFT			( 1000 )
#define SENSOR_ALS_SMEM_ALS_ADJ_MIN		( 0x0000 )
#define SENSOR_ALS_SMEM_ALS_ADJ_MAX		( 0xFFFF )
#define SENSOR_ALS_SMEM_ALS_SHFT_MIN	( 0x00 )
#define SENSOR_ALS_SMEM_ALS_SHFT_MAX	( 0xFF )
#define SENSOR_ALS_SMEM_ALS_SHFT_LTD	( 0x1F )
#define SENSOR_ALS_SMEM_ALS_SHFT_OUT	( 0x10 )
#define SENSOR_PSALS_ROUND_SHFT			( 1000 )
#define SENSOR_PSALS_ROUND_ADD			( 5 )
#define SENSOR_PSALS_ROUND_DIV			( 10 )
#define SENSOR_ALS_MODPRM_ADO_DEFVAL	( 0 )
#define SENSOR_ALS_DATA_ALS0_VAL		( 64 )
#define SENSOR_ALS_DATA_ALS1_VAL		( 60 )
#define SENSOR_ALS_IR_ZERO_DIVISION_VAL	( 0 )
#define SENSOR_ALS_UPTO_NEXT_RANGE		( 0xFF37 )
#define SENSOR_ALS_DOWN_NEXT_RANGE		( 0x2EE0 )
#define SENSOR_ALS_DOWN_ZERO_RANGE		( 0x0BB8 )
#define SENSOR_ALS_RANGE_ZERO			( 0 )
#define SENSOR_ALS_RANGE_FOUR			( 4 )
#define SENSOR_ALS_RANGE_SIX			( 6 )
#define SENSOR_ALS_BIT_RES_SHFT_VAL		( 3 )
#define SENSOR_ALS_BIT_RES_MASK_VAL		( 0x38 )
#define SENSOR_ALS_RANGE_BIT_MASK_VAL	( 0x07 )
#define SENSOR_ALS_BIT_ALS_SHFT_VAL		( 8 )
#define SENSOR_ALS_BIT_ADO_SHFT_VAL		( 15 )
#define SENSOR_ALS_BIT_ADO_SHFT_MAX		( 32 )
#define SENSOR_ALS_BIT_RNG_SHFT_VAL		( 4 )

// Reg. 00H
#define	OP_SHUTDOWN		0x00	// OP3:0
#define	OP_RUN			0x80	// OP3:1
#define	OP_CONTINUOUS	0x40	// OP2:1
#define	OP_PS_ALS		0x00	// OP01:00
#define	OP_ALS			0x10	// OP01:01
#define	OP_PS			0x20	// OP01:10
#define	OP_COUNT		0x30	// OP01:11
#define	INT_ALCLEAR		0x0C	// PROX:1 FLAG_P:1 FLAG_A:0
#define	INT_NOCLEAR		0x0E	// PROX:1 FLAG_P:1 FLAG_A:1

// Reg. 01H
#define	PRST_1			0x00	// PRST:00
#define	PRST_4			0x40	// PRST:01
#define	PRST_8			0x80	// PRST:10
#define	PRST_16			0xC0	// PRST:11
#define	RES_A_14		0x20	// RES_A:100
#define	RES_A_16		0x18	// RES_A:011
#define	RANGE_A_0		0x00	// RANGE_A:000
#define	RANGE_A_8		0x03	// RANGE_A:011
#define	RANGE_A_16		0x04	// RANGE_A:100
#define	RANGE_A_32		0x05	// RANGE_A:101
#define	RANGE_A_64		0x06	// RANGE_A:110
#define	RANGE_A_128		0x07	// RANGE_A:111

// Reg. 02H
#define	INTTYPE_L		0x00	// INTTYPE:0
#define	INTTYPE_P		0x40	// INTTYPE:1
#define	RES_P_14		0x08	// RES_P:001
#define	RES_P_12		0x10	// RES_P:010
#define	RES_P_10		0x18	// RES_P:011
#define	RANGE_P_4		0x02	// RANGE_A:010
#define	RANGE_P_8		0x03	// RANGE_A:011

// Reg. 03H
#define	INTVAL_0		0x00	// INTVAL:00
#define	INTVAL_4		0x40	// INTVAL:01
#define	INTVAL_8		0x80	// INTVAL:10
#define	INTVAL_16		0xC0	// INTVAL:11
#define	IS_65			0x20	// IS:10
#define	IS_130			0x30	// IS:11
#define	PIN_INT			0x00	// PIN:00
#define	PIN_INT_ALS		0x04	// PIN:01
#define	PIN_INT_PS		0x08	// PIN:10
#define	PIN_DETECT		0x0C	// PIN:11
#define	FREQ_327_5		0x00	// FREQ:0
#define	FREQ_81_8		0x02	// FREQ:1
#define	RST				0x01	// RST:1

#define GP2AP030_REGUSE

#define PSALS_ADDRESS_MAX	0x12	// ADDRESS_MAX
#define ALS_LUX_TABLE_ARRAY_SIZE    (ARRAY_SIZE(gp2ap030_ado_tbl))

#define ERR_PSALS_STATE_DISABLED -1 // PS, ALS Disabled state Error Code

#define PROX_USE_SMEM
#define ALS_USE_SMEM

struct gp2ap_data
{
	struct mutex			mutex ;
	struct i2c_client	   	*client ;
	u8				regData[12] ;
	struct class		   	*dev_class ;

	struct input_dev	   	*als_input_dev ;
	struct delayed_work		als_work ; 
	int				als_enabled ;
	int				als_mode ;
	int				als_delay ;
	int				als_lux_prev ;
	u8				als_range_prev ;
	int 			als_delay_gap ;
	int				als_enabled_prev ;
	struct timeval	als_tv;

	int				ps_gpio ;
	int				ps_irq ;
	int				ps_irq_enabled ;
	spinlock_t		ps_lock ;
	struct input_dev	   	*ps_input_dev ;
	struct work_struct		ps_int_work ;
	int				ps_enabled ;
	int				ps_enabled_prev ;
	int				ps_distance ;
	int				ps_calibration ;
#if defined(PSALS_RECOVERY_ENABLE)
	struct delayed_work		ps_polling_work;
	struct workqueue_struct	*work_queue_recovery;
	struct work_struct		psals_recovery_work;
#endif /* PSALS_RECOVERY_ENABLE */
} ;

static u8 gp2ap_init_data[12] = {
	/* Reg0 shutdown */
	0x00,
	/* Reg1 PRST:01 RES_A:100 RANGE_A:011 */
	0x00,
	/* Reg2 INTTYPE:0 RES_P:011 RANGE_P:010 */
	0x00,
	/* Reg3 INTVAL:0 IS:11 PIN:11 FREQ:0 */
	0x00,
	/* Reg.4 TL[7:0]:0x00 */
	0x00,
	/* Reg.5 TL[15:8]:0x00 */
	0x00,
	/* Reg.6 TH[7:0]:0x00 */
	0xff,
	/* Reg.7 TH[15:8]:0x00 */
	0xff,
	/* Reg.8 PL[7:0]:0x0B */
	0x00,
	/* Reg.9 PL[15:8]:0x00 */
	0x00,
	/* Reg.A PH[7:0]:0x0C */
	0xff,
	/* Reg.B PH[15:8]:0x00 */
	0xff
} ;

static struct platform_device *gp2ap_pdev = NULL ;
#ifdef GP2AP030_REGUSE
static struct regulator *gp2ap_vdda = NULL;
#endif

static int gp2ap_restart_device( struct gp2ap_data *data ) ;
static struct wake_lock		gp2ap_ps_timeout_wake_lock;
static struct wake_lock		gp2ap_ps_wake_lock ;
static u32					gp2ap_als_ave_als0 = 0 ;
static u32					gp2ap_als_ave_als1 = 0 ;
static u32					gp2ap_als_ave_ado = 0 ;
static unsigned long		bootmode = SH_BOOT_NORMAL ;
static u8 reg_address_data = 0x00 ;
static u8 reg_count_data = 0x01 ;

#ifndef ALS_USE_SMEM
int gp2ap_modp_ado_vala = 0 ;
int gp2ap_modp_ado_valb = 0 ;
int gp2ap_modp_ado_valc = 0 ;

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(gp2ap_modp_ado_vala,  int, 0600) ;
module_param(gp2ap_modp_ado_valb,  int, 0600) ;
module_param(gp2ap_modp_ado_valc,  int, 0600) ;
#endif /* CONFIG_ANDROID_ENGINEERING */

#endif /* ALS_USE_SMEM */


#if defined(CONFIG_ANDROID_ENGINEERING)
	#define PSALS_PARAM_DEF(name, val) \
		static int name = val; \
		module_param(name, int, S_IRUGO | S_IWUSR);
#else
	#define PSALS_PARAM_DEF(name, val) \
		static const int name = val;
#endif /* CONFIG_ANDROID_ENGINEERING */

#if defined(PSALS_RECOVERY_ENABLE)

#define PSALS_RECOVERY_DELAY			(10 * 1000)	/* 10 ms */
#define RECOVERY_RETRY_COUNT        3 // recovery retry count

PSALS_PARAM_DEF(PSALS_RECOVERY, 1);
PSALS_PARAM_DEF(PSALS_RECOVERY_POLLING_DELAY, 5000);

static struct wake_lock gp2ap_recovery_wake_lock;
static unsigned char psals_recovery_flg 	= 0;

#endif /* PSALS_RECOVERY_ENABLE */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
#if defined(PSALS_RECOVERY_ENABLE)
static void psals_recovery_func( struct work_struct *work );
#endif /* PSALS_RECOVERY_ENABLE */


/* *****************************************************************************
		Common
***************************************************************************** */
static char *strtok( char *s1, char *s2 )
{
	static char	   *str ;
	char		   *start ;

	if( s1 != NULL )
	{
		str = s1 ;
	}
	start = str ;

	if( str == NULL )
	{
		return NULL ;
	}

	if( s2 != NULL )
	{
		str = strstr( str, s2 ) ;
		if( str != NULL )
		{
			*str = 0x00 ;
			str += strlen( s2 ) ;
		}
	}
	else
	{
		str = NULL ;
	}
	return start ;
}

/* *****************************************************************************
		I2C
***************************************************************************** */
static int gp2ap_i2c_read( u8 reg, unsigned char *rbuf, int len, struct i2c_client *client )
{

	int					err = -1 ;
	struct i2c_msg		i2cMsg[2] ;
	uint8_t				buff ;

	if( client == NULL )
	{
		return -ENODEV ;
	}

	i2cMsg[0].addr = client->addr ;
	i2cMsg[0].flags = 0 ;
	i2cMsg[0].len = 1 ;
	i2cMsg[0].buf = &buff ;
	buff = reg ;
	i2cMsg[1].addr = client->addr ;
	i2cMsg[1].flags = I2C_M_RD ;
	i2cMsg[1].len = len ;
	i2cMsg[1].buf = rbuf ;

	err = i2c_transfer( client->adapter, &i2cMsg[0], 2 ) ;
/*
	if( err >= 0 )
	{
		i2cMsg.flags = I2C_M_RD ;
		i2cMsg.len = len ;
		i2cMsg.buf = rbuf ;
		err = i2c_transfer( client->adapter, &i2cMsg, 1 ) ;
	}
*/
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]i2c transfer error(%d)!!\n", __func__, __LINE__, err ) ;
	}

    return err ;
}

static int gp2ap_i2c_write( u8 reg, u8 *wbuf, struct i2c_client *client ,struct gp2ap_data *data)
{
	int					err = 0 ;
	struct i2c_msg		i2cMsg ;
	unsigned char		buff[2] ;
	int					retry = 10 ;

	if( client == NULL )
	{
		return -ENODEV ;
	}

	while( retry-- )
	{
		buff[0] = reg ;
		buff[1] = *wbuf ;

		i2cMsg.addr = client->addr ;
		i2cMsg.flags = 0 ;
		i2cMsg.len = 2 ;
		i2cMsg.buf = buff ;

		err = i2c_transfer( client->adapter, &i2cMsg, 1 ) ;
		pr_debug( "gp2ap_i2c_write : 0x%x, 0x%x\n", reg, *wbuf ) ;

		if( err >= 0 )
		{
			data->regData[reg] = *wbuf;
			return 0 ;
		}
	}
	printk( KERN_ERR "[%s][%d]i2c transfer error(%d)!!\n", __func__, __LINE__, err ) ;

    return err ;
}

static int gp2ap_i2c_probe( struct i2c_client *client, const struct i2c_device_id *id )
{
	struct gp2ap_data			   *data ;
	struct gp2ap030_platform_data  *pdata ;

	pr_debug( "gp2ap_i2c_probe \n" ) ;

	data = platform_get_drvdata( gp2ap_pdev ) ;
	data->client = client ;
	pdata = client->dev.platform_data ;
	data->ps_gpio = of_get_named_gpio(client->dev.of_node,"qcom,prox-int-gpio",0);
	data->ps_irq = gpio_to_irq( data->ps_gpio ) ;

	pr_debug( "GP2AP i2c attach success!!!\n" ) ;
	return 0 ;
}

static int gp2ap_i2c_remove( struct i2c_client *client )
{
	struct gp2ap_data  *data ;

	pr_debug( "gp2ap_i2c_remove \n" ) ;

	data = platform_get_drvdata( gp2ap_pdev ) ;
	data->client = NULL ;

	return 0 ;
}

static const struct i2c_device_id gp2ap_device_id[] =
{
	{ "sharp,gp2ap030", 0},
	{ }
} ;
MODULE_DEVICE_TABLE( i2c, gp2ap_device_id ) ;

static struct of_device_id gp2ap_match_table[] = {
	{ .compatible = "sharp,gp2ap030",},
	{ }
};
MODULE_DEVICE_TABLE(of, gp2ap_match_table);

static struct i2c_driver gp2ap_i2c_driver =
{
	.driver = {
		.name = "sharp,gp2ap030",
		.owner= THIS_MODULE,
		.of_match_table = gp2ap_match_table,
	},
	.probe		= gp2ap_i2c_probe,
	.remove		= gp2ap_i2c_remove,
	.id_table	= gp2ap_device_id,
} ;

static int gp2ap_i2c_init( void )
{
	if( i2c_add_driver( &gp2ap_i2c_driver ) )
	{
		printk( KERN_ERR "[%s][%d]i2c_add_driver failed \n", __func__, __LINE__ ) ;
		return -ENODEV ;
	}
	return 0 ;
}


/* *****************************************************************************
		Light Sensor
***************************************************************************** */
static ssize_t
als_delay_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	int					delay ;

	pr_debug( "als_delay_show \n" ) ;

	mutex_lock( &data->mutex ) ;
	delay = data->als_delay ;
	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", delay ) ;
}

static ssize_t
als_delay_store( struct device *dev,
					struct device_attribute *attr,
						const char *buf,
							size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	char			   *endp ;
	int					delay = simple_strtoul( buf, &endp, 10 ) ;
	int					enable ;
	int					old_delay ;

	pr_debug( "als_delay_store data=%s\n", buf ) ;

	if( !( endp != buf &&
			( endp == ( buf + strlen( buf ) )
				|| ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid delay (%s)\n", __func__, __LINE__, buf ) ;
		return count ;
	}
	if( delay < 0 || SENSOR_MAX_DELAY < delay )
	{
		printk( KERN_ERR "[%s][%d]invalid delay (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}

	mutex_lock( &data->mutex ) ;

	if( data->als_enabled )
	{
		cancel_delayed_work( &data->als_work ) ;
		schedule_delayed_work( &data->als_work, msecs_to_jiffies( delay ) ) ;
	}

	old_delay = data->als_delay ;
	data->als_delay = delay ;
	enable = data->als_enabled ;

	mutex_unlock( &data->mutex ) ;

	if( old_delay != delay )
	{
		input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enable << 16) | delay ) ;
	}

	return count ;
}

static int als_onoff( u8 onoff, struct gp2ap_data *data )
{
	u8		value ;
	u8		rdata ;
	int 	i;
	int err = 0;

	pr_debug( "light_sensor onoff = %d\n", onoff ) ;

	if( onoff )
	{
		if( !data->ps_enabled )
		{
#ifdef GP2AP030_REGUSE
			err = regulator_enable(gp2ap_vdda);
			if (err) {
				printk(KERN_ERR "[%s][%d]regulator_enable failure. (%d)\n", __func__, __LINE__, err);
				return -1;
			}
#endif		
			usleep( SENSOR_REGL_ON_DELAY ) ; // S0 -> S1
			
			for( i = 0 ; i < REG_ADR00_READ_COUNT ; i++ ) // S1 -> S2
			{
				value = OP_SHUTDOWN ; // shutdown
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( PRST_1 | RES_A_16 ) ;
				err = gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( INTTYPE_L | RES_P_12 ) ;
				err = gp2ap_i2c_write( REG_ADR_02, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_02 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( INTVAL_0 | PIN_INT_PS ) ;
				err = gp2ap_i2c_write( REG_ADR_03, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_03 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( OP_RUN | OP_CONTINUOUS | OP_ALS ) ;				// ALS mode
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				err = gp2ap_i2c_read( REG_ADR_00, &rdata, sizeof( rdata ), data->client ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 read err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				if ( rdata == OP_SHUTDOWN ) 
				{
					if ( i > 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_00 unregisted. !! Retry:%d\n", __func__, __LINE__, i ) ;
					}
					if ( i == 3 )
					{
						return -1;
					}
				}
				else
				{
					i = REG_ADR00_READ_COUNT ;
				}
			}
		}
		else
		{ // S3 -> S4
			value = ( OP_SHUTDOWN | INT_ALCLEAR ) ; 
			err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			value = ( RES_A_14 | (data->regData[REG_ADR_01] & RANGE_A_128)) ;
			err = gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			value = ( INTTYPE_L | RES_P_12 ) ;
			err = gp2ap_i2c_write( REG_ADR_02, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_02 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			value = ( INTVAL_16 | IS_65 | PIN_DETECT | FREQ_327_5 ) ;
			err = gp2ap_i2c_write( REG_ADR_03, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_03 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			value = ( OP_RUN | OP_CONTINUOUS | OP_PS_ALS | INT_ALCLEAR ) ; // ALS & PS mode
			err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			usleep( SENSOR_PS_PALS_DELAY ) ;
		}
		do_gettimeofday( &data->als_tv ) ;
	}
	else
	{
		memset( &data->als_tv, 0, sizeof( struct timeval )) ;

		if( !data->ps_enabled ) // 1:als OFF 0:als ON
		{ // S2,S3,S4 -> S0
#if defined(PSALS_RECOVERY_ENABLE)
			if(PSALS_RECOVERY) {

				value = OP_RUN;
				err = gp2ap_i2c_write(REG_ADR_00, &value, data->client, data);
				if(err < 0) {
					printk(KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__);
				}
				usleep(SENSOR_REGL_OFF_DELAY);

				value = OP_SHUTDOWN; // shutdown
				err = gp2ap_i2c_write(REG_ADR_00, &value, data->client, data);
				if(err < 0) {
					printk(KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__);
				}

#ifdef GP2AP030_REGUSE
				err = regulator_disable(gp2ap_vdda);
				if(err) {
					printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, err);
					return -1;
				}
				for(i = 0; i < sizeof(gp2ap_init_data); i++) {
					data->regData[i] = gp2ap_init_data[i];
				}
#endif
			}
			else {
				value = OP_RUN ;
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				usleep ( SENSOR_REGL_OFF_DELAY ) ;
				value = OP_SHUTDOWN ; // shutdown
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
#ifdef GP2AP030_REGUSE
				err = regulator_disable(gp2ap_vdda);
				if (err) {
					printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, err);
					return -1;
				}
				for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
				{
					data->regData[i] = gp2ap_init_data[i] ;
				}
#endif
			}
#else
			value = OP_RUN ;
			err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			usleep ( SENSOR_REGL_OFF_DELAY ) ;
			value = OP_SHUTDOWN ; // shutdown
			err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
#ifdef GP2AP030_REGUSE
			err = regulator_disable(gp2ap_vdda);
			if (err) {
				printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, err);
				return -1;
			}
			for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
			{
				data->regData[i] = gp2ap_init_data[i] ;
			}
#endif
#endif /* PSALS_RECOVERY_ENABLE */
		}
		else
		{
			for( i = 0 ; i < REG_ADR00_READ_COUNT ; i++ ) // S4 -> S3
			{
				value = ( OP_SHUTDOWN | INT_ALCLEAR ) ;
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( PRST_1 | (data->regData[REG_ADR_01] & RANGE_A_128)) ;
				err = gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( INTTYPE_L | RES_P_12 ) ;
				err = gp2ap_i2c_write( REG_ADR_02, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_02 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( INTVAL_16 | IS_65 | PIN_DETECT | FREQ_327_5 ) ;
				err = gp2ap_i2c_write( REG_ADR_03, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_03 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( OP_RUN | OP_CONTINUOUS | OP_PS | INT_ALCLEAR ) ;	// ALS mode
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				err = gp2ap_i2c_read( REG_ADR_00, &rdata, sizeof( rdata ), data->client ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 read err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				if ( rdata == OP_SHUTDOWN ) 
				{
					if ( i > 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_00 unregisted. !! Retry:%d\n", __func__, __LINE__, i ) ;
					}
					if ( i == 3 )
					{
						return -1;
					}
				}
				else
				{
					i = REG_ADR00_READ_COUNT ;
				}
			}
		}
	}
	return 0 ;
}

static ssize_t
als_enable_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	int					enabled ;

	pr_debug( "als_enable_show \n" ) ;

	mutex_lock( &data->mutex ) ;
	enabled = data->als_enabled ;
	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", enabled ) ;
}

static ssize_t
als_enable_store( struct device *dev,
					struct device_attribute *attr,
						const char *buf,
							size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	char			   *endp ;
	int					enabled = simple_strtoul( buf, &endp, 10 ) ;
	int					err = 0;
#if defined(PSALS_RECOVERY_ENABLE)
	int					i;
#endif /* PSALS_RECOVERY_ENABLE */

	pr_debug( "[%s]als_enable_store data=%s\n", __func__, buf);

	if( !( endp != buf &&
			( endp == ( buf + strlen( buf ) )
				|| ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}
	if( enabled != 0 && enabled != 1 )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}

	if( !enabled )
	{
		mutex_lock( &data->mutex );
	    if(!data->als_enabled) {
	        printk( KERN_ERR "[%s][%d]als is state disabled (%d)\n", __func__, __LINE__, data->als_enabled);
	    }
		else
		{
			cancel_delayed_work( &data->als_work );
			data->als_mode = LOW_LUX_MODE;
			err = als_onoff( 0, data );
			if(err < 0) {
				printk(KERN_ERR "[%s][%d]als power off failed.\n", __func__, __LINE__);
				mutex_unlock(&data->mutex);
				return -1;
			}
			data->als_enabled_prev = data->als_enabled ;
			data->als_enabled = enabled ;
		}
		mutex_unlock( &data->mutex ) ;
		pr_debug( "light_sensor disable!! \n" ) ;
		input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) | data->als_delay ) ;
	}
	else
	{
		mutex_lock( &data->mutex );
	    if(data->als_enabled) {
	        printk( KERN_ERR "[%s][%d]als is state enabled (%d)\n", __func__, __LINE__, data->als_enabled);
	    }
		else {
#if defined(PSALS_RECOVERY_ENABLE)
			if(PSALS_RECOVERY) {
				for(i = 0; i < RECOVERY_RETRY_COUNT; i++) {
					err = als_onoff(1, data);
					if(err == 0) {
						printk(KERN_ERR "[%s][%d]als on success. (%d)\n", __func__, __LINE__, err);
						break;
					}
				}
			}
			else {
				err = als_onoff(1, data);
			}
#else
			err = als_onoff(1, data);
#endif /* PSALS_RECOVERY_ENABLE */
			if(err < 0) {
				printk(KERN_ERR "[%s][%d]als power on failed.\n", __func__, __LINE__);
				mutex_unlock(&data->mutex);
				return -1;
			}
			data->als_enabled_prev = data->als_enabled ;
			data->als_enabled = enabled;
			if (( bootmode != SH_BOOT_D ) && ( bootmode != SH_BOOT_F_F ))
			{
				schedule_delayed_work( &data->als_work, 0 ) ;
			}
		}
		mutex_unlock( &data->mutex ) ;
		pr_debug( "light_sensor enable!! \n" ) ;
		input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) | data->als_delay ) ;
		input_report_abs( input_dev, ABS_LUX_REPORT, -1 ) ;
	}

	return count ;
}

static void
als_wait_after_shift_range( struct gp2ap_data *data, u8 range0 )
{
	int				delay ;
	unsigned long	pass ;
	struct timeval	tv ;

	if (( range0 != data->als_range_prev ) ||
		( data->ps_enabled != data->ps_enabled_prev ) ||
		( data->als_enabled != data->als_enabled_prev ))
	{
		if( data->als_tv.tv_usec )
		{
			do_gettimeofday( &tv ) ;
			pass = ((tv.tv_sec -  data->als_tv.tv_sec) * SENSOR_PALS_UNIT_SHFT)
				 + ((tv.tv_usec - data->als_tv.tv_usec) / SENSOR_PALS_UNIT_SHFT );
		}
		else
		{
			pass = SENSOR_ALS_RANGE_NO_DELAY ;
		}
		
		if( data->ps_enabled )
		{
			delay = ( SENSOR_ALS_RANGE_DELAY_MS - pass ) ;
		}
		else
		{
			delay = ( SENSOR_PS_RANGE_DELAY_MS - pass ) ;
		}
		
		if ( delay > SENSOR_ALS_RANGE_NO_DELAY )
		{
			usleep( delay * SENSOR_PSALS_ROUND_SHFT ) ;
			data->als_delay_gap = delay ;
		}
		else
		{
			data->als_delay_gap = SENSOR_ALS_RANGE_NO_DELAY ;
		}
		
		data->als_range_prev = range0 ;
		data->als_enabled_prev = data->als_enabled ;
		data->ps_enabled_prev = data->ps_enabled ;
	}
	else
	{
		data->als_delay_gap = SENSOR_ALS_RANGE_NO_DELAY ;
	}
	
	memset( &data->als_tv, 0, sizeof( struct timeval )) ;
	
	return ;
}

static void
als_adj_smem( struct gp2ap_data *data, u16 *alph, u16 *beta, u8 *gamm, u32 ir, u32 ir_val )
{
#ifdef ALS_USE_SMEM
	sharp_smem_common_type *p_sh_smem_common_type = NULL ;
	u16				smem_alph[2] ;
	u16				smem_beta[2] ;
	u8				smem_gamm[2] ;

	p_sh_smem_common_type = sh_smem_get_common_address() ;
	if( p_sh_smem_common_type == NULL )
	{
		printk( KERN_ERR "[%s][%d]sh_smem_get_common_address failed. !! \n", __func__, __LINE__ ) ;
		if ( ir > ir_val )
		{
			*alph = ( u16 )ALPH_ADJ1_VAL ;
			*beta = ( u16 )BETA_ADJ1_VAL ;
			*gamm = ( u8  )GAMM_ADJ1_VAL ;
		}
		else
		{
			*alph = ( u16 )ALPH_ADJ0_VAL ;
			*beta = ( u16 )BETA_ADJ0_VAL ;
			*gamm = ( u8  )GAMM_ADJ0_VAL ;
		}
	}
	else
	{
		if ((( p_sh_smem_common_type->sh_als_adj0[0]	== SENSOR_ALS_SMEM_ALS_ADJ_MIN  ) &&
		     ( p_sh_smem_common_type->sh_als_adj1[0]	== SENSOR_ALS_SMEM_ALS_ADJ_MIN  ) &&
			 ( p_sh_smem_common_type->sh_als_shift[0]	== SENSOR_ALS_SMEM_ALS_SHFT_MIN ) &&
			 ( p_sh_smem_common_type->sh_als_adj0[1]	== SENSOR_ALS_SMEM_ALS_ADJ_MIN  ) &&
			 ( p_sh_smem_common_type->sh_als_adj1[1]	== SENSOR_ALS_SMEM_ALS_ADJ_MIN  ) &&
			 ( p_sh_smem_common_type->sh_als_shift[1]	== SENSOR_ALS_SMEM_ALS_SHFT_MIN )) ||
			
			(( p_sh_smem_common_type->sh_als_adj0[0]	== SENSOR_ALS_SMEM_ALS_ADJ_MAX  ) &&
			 ( p_sh_smem_common_type->sh_als_adj1[0]	== SENSOR_ALS_SMEM_ALS_ADJ_MAX  ) &&
			 ( p_sh_smem_common_type->sh_als_shift[0]	== SENSOR_ALS_SMEM_ALS_SHFT_MAX ) &&
			 ( p_sh_smem_common_type->sh_als_adj0[1]	== SENSOR_ALS_SMEM_ALS_ADJ_MAX  ) &&
			 ( p_sh_smem_common_type->sh_als_adj1[1]	== SENSOR_ALS_SMEM_ALS_ADJ_MAX  ) &&
			 ( p_sh_smem_common_type->sh_als_shift[1]	== SENSOR_ALS_SMEM_ALS_SHFT_MAX )) ||
			
			(  p_sh_smem_common_type->sh_als_shift[0]	<  SENSOR_ALS_SMEM_ALS_SHFT_MIN    ) ||
			(  p_sh_smem_common_type->sh_als_shift[0]	>  SENSOR_ALS_SMEM_ALS_SHFT_LTD    ) ||
			(  p_sh_smem_common_type->sh_als_shift[0]	== SENSOR_ALS_SMEM_ALS_SHFT_OUT    ) ||
			(  p_sh_smem_common_type->sh_als_shift[1]	<  SENSOR_ALS_SMEM_ALS_SHFT_MIN    ) ||
			(  p_sh_smem_common_type->sh_als_shift[1]	>  SENSOR_ALS_SMEM_ALS_SHFT_LTD    ) ||
			(  p_sh_smem_common_type->sh_als_shift[1]	== SENSOR_ALS_SMEM_ALS_SHFT_OUT    ))
		{
			printk( KERN_ERR "[%s][%d]sh_smem Alpha, Beta, Gamma check err. !! \n", __func__, __LINE__ ) ;
			if ( ir > ir_val )
			{
				*alph = ( u16 )ALPH_ADJ1_VAL ;
				*beta = ( u16 )BETA_ADJ1_VAL ;
				*gamm = ( u8  )GAMM_ADJ1_VAL ;
			}
			else
			{
				*alph = ( u16 )ALPH_ADJ0_VAL ;
				*beta = ( u16 )BETA_ADJ0_VAL ;
				*gamm = ( u8  )GAMM_ADJ0_VAL ;
			}
		}
		else
		{
			memcpy( smem_alph, p_sh_smem_common_type->sh_als_adj0, sizeof( smem_alph ) ) ;
			memcpy( smem_beta, p_sh_smem_common_type->sh_als_adj1, sizeof( smem_beta ) ) ;
			memcpy( smem_gamm, p_sh_smem_common_type->sh_als_shift, sizeof( smem_gamm ) ) ;
			
			if ( ir > ir_val )
			{
				*alph = smem_alph[1] ;
				*beta = smem_beta[1] ;
				*gamm = smem_gamm[1] ;
			}
			else
			{
				*alph = smem_alph[0] ;
				*beta = smem_beta[0] ;
				*gamm = smem_gamm[0] ;
			}
		}
	}
#else
	if (( gp2ap_modp_ado_vala == SENSOR_ALS_MODPRM_ADO_DEFVAL ) &&
		( gp2ap_modp_ado_valb == SENSOR_ALS_MODPRM_ADO_DEFVAL ) &&
		( gp2ap_modp_ado_valc == SENSOR_ALS_MODPRM_ADO_DEFVAL ))
	{
		if ( ir > ir_val )
		{
			*alph = ( u16 )ALPH_ADJ1_VAL ;
			*beta = ( u16 )BETA_ADJ1_VAL ;
			*gamm = ( u8  )GAMM_ADJ1_VAL ;
		}
		else
		{
			*alph = ( u16 )ALPH_ADJ0_VAL ;
			*beta = ( u16 )BETA_ADJ0_VAL ;
			*gamm = ( u8  )GAMM_ADJ0_VAL ;
		}
	}
	else
	{
		*alph = gp2ap_modp_ado_vala ;
		*beta = gp2ap_modp_ado_valb ;
		*gamm = gp2ap_modp_ado_valc ;
	}
#endif  /* ALS_USE_SMEM */
	
	return ;
}

static int
als_adjust_ado( struct gp2ap_data *data, u32 *data_als0, u32 *data_als1, u32 ir, u32 ir_val, u8 res, u8 range0 )
{
	u16				alph ;
	u16				beta ;
	u8				gamm ;
	u32				ado0 ;
	int				ado1 ;
	int				shift_tmp = 0 ;
	u64				ado_tmp ;
	
	als_adj_smem( data, &alph, &beta, &gamm, ir, ir_val ) ;
	
	ado0 = (( *data_als0 * alph ) - ( *data_als1 * beta )) ;
	shift_tmp = gamm ;
	
	if ( ado0 < 0 )
	{
		ado0 = 0 ;
	}
	
	if ( shift_tmp > SENSOR_ALS_BIT_ADO_SHFT_VAL )
	{
		shift_tmp = SENSOR_ALS_BIT_ADO_SHFT_MAX - shift_tmp ;
	}
	
	if ( res < SENSOR_ALS_BIT_RES_SHFT_VAL )
	{
		shift_tmp += ( res - SENSOR_ALS_BIT_RES_SHFT_VAL ) 
					+ ( range0 - SENSOR_ALS_BIT_RNG_SHFT_VAL - SENSOR_ALS_BIT_ADO_SHFT_VAL ) ;
	}
	else
	{
		shift_tmp += ( res - SENSOR_ALS_BIT_RES_SHFT_VAL ) * 2 
					+ ( range0 - SENSOR_ALS_BIT_RNG_SHFT_VAL - SENSOR_ALS_BIT_ADO_SHFT_VAL ) ;
	}
	
	if ( shift_tmp  < 0 )
	{
		shift_tmp = shift_tmp * -1 ;
		ado_tmp = ( uint64_t )ado0 >> ( shift_tmp ) ;
	}
	else
	{
		ado_tmp = ( uint64_t )ado0 << ( shift_tmp ) ;
	}
	
	if ( ado_tmp > ADO_MAX )
	{
		ado1 = ADO_MAX ;
	}
	else 
	{
		ado1 = ( int )ado_tmp ;
	}
	
	return ado1 ;
}

static int
als_shift_range( struct gp2ap_data *data ,u8 range_next )
{
	u8				value ;
	int 			err = 0 ;
	
	value = ((data->regData[REG_ADR_00] & OP_COUNT) | OP_SHUTDOWN | OP_CONTINUOUS | INT_NOCLEAR);
	err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
		return -1;
	}
	
	if ( range_next == SENSOR_ALS_RANGE_FOUR )
	{
		value = ((data->regData[REG_ADR_01] & ~RANGE_A_128) | RANGE_A_16);
	}
	else
	{
		if ( range_next == SENSOR_ALS_RANGE_SIX )
		{
			value = ((data->regData[REG_ADR_01] & ~RANGE_A_128) | RANGE_A_64);
		}
		else 
		{
			value = ((data->regData[REG_ADR_01] & ~RANGE_A_128) | RANGE_A_0);
		}
	}
	
	err = gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
		return -1;
	}
	
	value = ((data->regData[REG_ADR_00] & OP_COUNT) | OP_RUN | OP_CONTINUOUS);
	err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
		return -1;
	}
	
	return 0 ;
}

static int
als_get_ado( struct gp2ap_data *data )
{
	u8				rdata[4] ;
	u8				range0 ;
	u8 				res ;
	u32				data_als0 ;
	u32				data_als1 ;
	int				ado1 ;
	u32				ir ;
	u32				ir_val ;
	int 			err = 0 ;
	u8				range_next = 0 ;
	
	err = gp2ap_i2c_read( REG_ADR_01, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_01 read err. !! \n", __func__, __LINE__ ) ;
		return -1;
	}
	range0 = ( rdata[0] & SENSOR_ALS_RANGE_BIT_MASK_VAL ) ;
	res    = ( rdata[0] & SENSOR_ALS_BIT_RES_MASK_VAL ) >> SENSOR_ALS_BIT_RES_SHFT_VAL ;
	
	als_wait_after_shift_range( data, range0 ) ;
	
	err = gp2ap_i2c_read( REG_ADR_0C, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_0C read err. !! \n", __func__, __LINE__ ) ;
		return -1;
	}
	data_als0 = ( rdata[1] << SENSOR_ALS_BIT_ALS_SHFT_VAL | rdata[0] ) ;
	data_als1 = ( rdata[3] << SENSOR_ALS_BIT_ALS_SHFT_VAL | rdata[2] ) ;
	
	if ( !data_als0 )
	{
		printk( KERN_ERR "[%s][%d]zero division by als0:%d\n", __func__, __LINE__, data_als0 ) ;
		ir = SENSOR_ALS_IR_ZERO_DIVISION_VAL ;
	}
	else 
	{
		ir = ((( data_als1 * SENSOR_PSALS_ROUND_SHFT / data_als0 )
			   + SENSOR_PSALS_ROUND_ADD ) / SENSOR_PSALS_ROUND_DIV ) ;
	}
	ir_val = ((( SENSOR_ALS_DATA_ALS1_VAL * SENSOR_PSALS_ROUND_SHFT / SENSOR_ALS_DATA_ALS0_VAL ) 
			   + SENSOR_PSALS_ROUND_ADD ) / SENSOR_PSALS_ROUND_DIV ) ;
	
	ado1 = als_adjust_ado( data, &data_als0, &data_als1, ir, ir_val, res, range0 ) ;
	
	data_als0 = data_als0 << ( ( res - SENSOR_ALS_BIT_RES_SHFT_VAL ) * 2 ) ;
	
	if ( range0 == 0 )
	{
		if ( data_als0 >= SENSOR_ALS_UPTO_NEXT_RANGE )
		{
			range_next = SENSOR_ALS_RANGE_FOUR ;
		}
		else
		{
			goto without_change_range ;
		}
	}
	else
	{
		if ( range0 == SENSOR_ALS_RANGE_FOUR )
		{
			if ( data_als0 >= SENSOR_ALS_UPTO_NEXT_RANGE )
			{
				range_next = SENSOR_ALS_RANGE_SIX ;
			}
			else
			{
				if ( data_als0 <= SENSOR_ALS_DOWN_ZERO_RANGE )
				{
					range_next = SENSOR_ALS_RANGE_ZERO ;
				}
				else
				{
					goto without_change_range ;
				}
			}
		}
		else
		{
			if ( data_als0 <= SENSOR_ALS_DOWN_NEXT_RANGE )
			{
				range_next = SENSOR_ALS_RANGE_FOUR ;
			}
			else
			{
				goto without_change_range ;
			}
		}
	}	
	err = als_shift_range( data, range_next ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]als_shift_range err. !! \n", __func__, __LINE__ ) ;
		return -1;
	}
	pr_debug( "als range changed!! from:%d to:%d\n", data->als_range_prev, range_next ) ;
	
    do_gettimeofday( &data->als_tv ) ;
	
	return ado1 ;

without_change_range:

	return ado1 ;
}

static int
als_get_lux( struct gp2ap_data *data )
{
	int ado, lux ;
	int i ;
	
	ado = als_get_ado( data ) ;
	
	if(ado == ERR_PSALS_STATE_DISABLED) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		return ERR_PSALS_STATE_DISABLED;
	}

	if ( ado == 0 )
	{
		lux = 0 ;
	}
	else
	{
		for ( i = 0; i < ALS_LUX_TABLE_ARRAY_SIZE; i++ )
		{
			if (( ado >= gp2ap030_ado_tbl[i].range_low ) &&
				( ado < gp2ap030_ado_tbl[i].range_high ))
			{
				lux = ((( ado * gp2ap030_ado_tbl[i].param_a ) + gp2ap030_ado_tbl[i].param_b + 50 ) / 100 ) ;
				i = ALS_LUX_TABLE_ARRAY_SIZE ;
				break;
			}
		}
	}
	pr_debug( "als_lux_show lux:%d ado: %d\n", lux, ado ) ;

	return lux ;
}

static ssize_t
als_lux_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	int lux ;

	pr_debug( "als_lux_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
	    return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	lux = als_get_lux( data ) ;
	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", lux ) ;
}

static ssize_t
als_ado_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	int		ado ;

	pr_debug( "als_ado_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
	    return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	ado = als_get_ado( data ) ;
	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", ado ) ;
}

static ssize_t
als_ave_als0_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	unsigned int		als0 ;

	pr_debug( "als_ave_als0_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}
	
	als0 = gp2ap_als_ave_als0 ;
	
	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", als0 ) ;
}

static ssize_t
als_ave_als1_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	unsigned int		als1 ;

	pr_debug( "als_ave_als1_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}
	
	als1 = gp2ap_als_ave_als1 ;
	
	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", als1) ;
}

static ssize_t
als_ave_ado_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	unsigned int		ado ;

	pr_debug( "als_ave_ado_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}
	
	ado = gp2ap_als_ave_ado ;
	
	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", ado ) ;
}

static ssize_t
als_wake_store( struct device *dev,
					struct device_attribute *attr,
						const char *buf,
							size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata(input_dev);
	static int			cnt = 1 ;

	pr_debug( "als_wake_store data=%s\n", buf ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		return ERR_PSALS_STATE_DISABLED;
	}

	input_report_abs( input_dev, ABS_WAKE, cnt++ ) ;

	return count ;
}

static ssize_t
als_raw_data_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u32					data_clear ;
	u32					data_ir ;
	u8					rdata[2] ;
	u32					ratio ;
	int					mode ;
	int 				err = 0 ;

	pr_debug( "als_raw_data_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	err = gp2ap_i2c_read( REG_ADR_0C, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_0C read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	data_clear = ( rdata[1] << 8 ) | rdata[0] ;
	err = gp2ap_i2c_read( REG_ADR_0E, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_0E read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	data_ir = ( rdata[1] << 8 ) | rdata[0] ;

	if( data_clear == 0 )
	{
		ratio = 100 ;
	}
	else
	{
		ratio = ( data_ir * 100 ) / data_clear ;
	}

	mode = data->als_mode ;

	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d,%d,%d.%02d,%d", data_clear, data_ir, ( ratio / 100 ), ( ratio % 100 ), mode ) ;
}

static ssize_t
als_avg_show( struct device *dev,
				struct device_attribute *attr,
					char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	int					min = 0 ;
	int					max = 0 ;
	int					avg = 0 ;
	int					i = 0 ;
	int					lux ;

	pr_debug( "als_avg_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	min = 0x7fffffff ;
	for( i = 0 ; i < ALS_AVG_READ_COUNT ; i++ )
	{
		lux = als_get_lux( data ) ;
		usleep( ALS_AVG_READ_INTERVAL ) ;
		avg += lux ;
		if( lux < min )
		{
			min = lux ;
		}
		if( lux > max )
		{
			max = lux ;
		}
	}
	avg /= ALS_AVG_READ_COUNT ;

	mutex_unlock( &data->mutex ) ;

	return sprintf( buf,"%d,%d,%d", min, max, avg ) ;
}

#if 0
static ssize_t
als_resolution_show( struct device *dev,
						struct device_attribute *attr,
							char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					rdata ;
	u32					res_a ;

	pr_debug( "als_resolution_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	gp2ap_i2c_read( REG_ADR_01, &rdata, sizeof( rdata ), data->client ) ;
	res_a = ( rdata & 0x38 ) >> 3 ;

	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", res_a ) ;
}

static ssize_t
als_resolution_store( struct device *dev,
					struct device_attribute *attr,
						const char *buf,
							size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					value ;
	u8					rdata ;
	u32					res_a = simple_strtoul( buf, NULL, 10 ) ;

	pr_debug( "als_resolution_store res_a=%d\n", res_a ) ;

	if( res_a < 0 || res_a > 7 )
	{
		return count ;
	}

	mutex_lock( &data->mutex ) ;

	value = ( OP_SHUTDOWN | INT_NOCLEAR ) ;	// shutdown
	gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;

	gp2ap_i2c_read( REG_ADR_01, &rdata, sizeof( rdata ), data->client ) ;

	value = ( rdata & 0xC7 ) | ( res_a << 3 ) ;
	gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;

	gp2ap_restart_device( data ) ;

	mutex_unlock( &data->mutex ) ;

	return count ;
}
#endif

static ssize_t
als_range_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					rdata ;
	u32					range_a ;
	int 			err = 0 ;

	pr_debug( "als_range_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	err = gp2ap_i2c_read( REG_ADR_01, &rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_01 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	range_a = rdata & 0x07 ;

	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", range_a ) ;
}

static ssize_t
als_range_store( struct device *dev,
					struct device_attribute *attr,
						const char *buf,
							size_t count )
{
#ifdef ALS_USE_SMEM
	sharp_smem_common_type *p_sh_smem_common_type = NULL ;
	u16					smem_alph[2] ;
	u16					smem_beta[2] ;
	u8					smem_gamm[2] ;
#endif  /* ALS_USE_SMEM */
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					value ;
	u8					rval[4] ;
	u8					range_a = simple_strtoul( buf, NULL, 10 ) ;
	u8					range_b ;
	u16					alph ;
	u16					beta ;
	u8					gamm ;
	u32					tmp_als0 ;
	u32					tmp_als1 ;
	u32					ir ;
	u32					ir_val ;
	int 				err = 0 ;
	int 				i ;

	pr_debug( "als_range_store range_a=%d\n", range_a ) ;

	if( range_a < 0 || range_a > 7 )
	{
		return -1 ;
	}
	
	mutex_lock( &data->mutex ) ;

	if(!data->als_enabled) {
		printk(KERN_ERR "[%s][%d]als is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return ERR_PSALS_STATE_DISABLED;
	}

	value = ( OP_SHUTDOWN | 0x50 ) ; // shutdown
	err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	err = gp2ap_i2c_read( REG_ADR_01, &range_b, sizeof( range_b ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_01 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	if ( range_a != ( range_b & 0x07 ) )
	{
		value = ( range_b & 0xF8 ) | range_a ;
		err = gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;
		if( err < 0 )
		{
			printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
		err = gp2ap_restart_device( data ) ;
		if( err < 0 )
		{
			printk( KERN_ERR "[%s][%d]gp2ap_restart_device failed. !! \n", __func__, __LINE__ ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
		usleep( 100 * 1000 ) ;
	}
	
	tmp_als0 = tmp_als1 = 0 ;
	
	for ( i = 0 ; i < AVE_ADO_READ_TIMES ; i++ )
	{
		err = gp2ap_i2c_read( REG_ADR_0C, rval, sizeof( rval ), data->client ) ;
		if( err < 0 )
		{
			printk( KERN_ERR "[%s][%d]REG_ADR_0C read err. !! \n", __func__, __LINE__ ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
		tmp_als0 = tmp_als0 + ( rval[1] << 8 | rval[0] ) ;
		tmp_als1 = tmp_als1 + ( rval[3] << 8 | rval[2] ) ;
	}
	
	gp2ap_als_ave_als0 = ( tmp_als0 / AVE_ADO_READ_TIMES ) ;
	gp2ap_als_ave_als1 = ( tmp_als1 / AVE_ADO_READ_TIMES ) ;
	
	ir     = ((( gp2ap_als_ave_als1 * 1000 / gp2ap_als_ave_als0 ) + 5 ) / 10 ) ;
	ir_val = ((( 60 * 1000 / 64 ) + 5 ) / 10 ) ;

#ifdef ALS_USE_SMEM
	p_sh_smem_common_type = sh_smem_get_common_address() ;
	if( p_sh_smem_common_type == NULL )
	{
		printk( KERN_ERR "[%s][%d]sh_smem_get_common_address failed. !! \n", __func__, __LINE__ ) ;
		if ( ir > ir_val )
		{
			alph = ALPH_ADJ1_VAL ;
			beta = BETA_ADJ1_VAL ;
			gamm = GAMM_ADJ1_VAL ;
		}
		else
		{
			alph = ALPH_ADJ0_VAL ;
			beta = BETA_ADJ0_VAL ;
			gamm = GAMM_ADJ0_VAL ;
		}
	}
	else
	{
		if ((( p_sh_smem_common_type->sh_als_adj0[0]	== SENSOR_ALS_SMEM_ALS_ADJ_MIN  ) &&
			 ( p_sh_smem_common_type->sh_als_adj1[0]	== SENSOR_ALS_SMEM_ALS_ADJ_MIN  ) &&
			 ( p_sh_smem_common_type->sh_als_shift[0]	== SENSOR_ALS_SMEM_ALS_SHFT_MIN ) &&
			 ( p_sh_smem_common_type->sh_als_adj0[1]	== SENSOR_ALS_SMEM_ALS_ADJ_MIN  ) &&
			 ( p_sh_smem_common_type->sh_als_adj1[1]	== SENSOR_ALS_SMEM_ALS_ADJ_MIN  ) &&
			 ( p_sh_smem_common_type->sh_als_shift[1]	== SENSOR_ALS_SMEM_ALS_SHFT_MIN )) ||

			(( p_sh_smem_common_type->sh_als_adj0[0]	== SENSOR_ALS_SMEM_ALS_ADJ_MAX  ) &&
			 ( p_sh_smem_common_type->sh_als_adj1[0]	== SENSOR_ALS_SMEM_ALS_ADJ_MAX  ) &&
			 ( p_sh_smem_common_type->sh_als_shift[0]	== SENSOR_ALS_SMEM_ALS_SHFT_MAX ) &&
			 ( p_sh_smem_common_type->sh_als_adj0[1]	== SENSOR_ALS_SMEM_ALS_ADJ_MAX  ) &&
			 ( p_sh_smem_common_type->sh_als_adj1[1]	== SENSOR_ALS_SMEM_ALS_ADJ_MAX  ) &&
			 ( p_sh_smem_common_type->sh_als_shift[1]	== SENSOR_ALS_SMEM_ALS_SHFT_MAX )) ||

			(  p_sh_smem_common_type->sh_als_shift[0]	<  SENSOR_ALS_SMEM_ALS_SHFT_MIN ) ||
			(  p_sh_smem_common_type->sh_als_shift[0]	>  SENSOR_ALS_SMEM_ALS_SHFT_LTD ) ||
			(  p_sh_smem_common_type->sh_als_shift[0]	== SENSOR_ALS_SMEM_ALS_SHFT_OUT ) ||
			(  p_sh_smem_common_type->sh_als_shift[1]	<  SENSOR_ALS_SMEM_ALS_SHFT_MIN ) ||
			(  p_sh_smem_common_type->sh_als_shift[1]	>  SENSOR_ALS_SMEM_ALS_SHFT_LTD ) ||
			(  p_sh_smem_common_type->sh_als_shift[1]	== SENSOR_ALS_SMEM_ALS_SHFT_OUT ))
		{
			printk( KERN_ERR "[%s][%d]sh_smem Alpha, Beta, Gamma check err. !! \n", __func__, __LINE__ ) ;
			if ( ir > ir_val )
			{
				alph = ALPH_ADJ1_VAL ;
				beta = BETA_ADJ1_VAL ;
				gamm = GAMM_ADJ1_VAL ;
			}
			else
			{
				alph = ALPH_ADJ0_VAL ;
				beta = BETA_ADJ0_VAL ;
				gamm = GAMM_ADJ0_VAL ;
			}
		}
		else
		{
        	memcpy( smem_alph, p_sh_smem_common_type->sh_als_adj0, sizeof( smem_alph ) ) ;
    	    memcpy( smem_beta, p_sh_smem_common_type->sh_als_adj1, sizeof( smem_beta ) ) ;
	        memcpy( smem_gamm, p_sh_smem_common_type->sh_als_shift, sizeof( smem_gamm ) ) ;
		
			if ( ir > ir_val )
			{
				alph = smem_alph[1] ;
				beta = smem_beta[1] ;
				gamm = smem_gamm[1] ;
			}
			else
			{
				alph = smem_alph[0] ;
				beta = smem_beta[0] ;
				gamm = smem_gamm[0] ;
			}
		}
	}
#else
	if (( gp2ap_modp_ado_vala == 0 ) &&
		( gp2ap_modp_ado_valb == 0 ) &&
		( gp2ap_modp_ado_valc == 0 ))
	{
		if ( ir > ir_val )
		{
			alph = ALPH_ADJ1_VAL ;
			beta = BETA_ADJ1_VAL ;
			gamm = GAMM_ADJ1_VAL ;
		}
		else
		{
			alph = ALPH_ADJ0_VAL ;
			beta = BETA_ADJ0_VAL ;
			gamm = GAMM_ADJ0_VAL ;
		}
	}
	else
	{
		alph = gp2ap_modp_ado_vala ;
		beta = gp2ap_modp_ado_valb ;
		gamm = gp2ap_modp_ado_valc ;
	}
#endif  /* ALS_USE_SMEM */
	
	if (gamm < 16)
	{
		gp2ap_als_ave_ado = ((( gp2ap_als_ave_als0 * alph ) - ( gp2ap_als_ave_als1 * beta )) << gamm) >> 15 ;
	} 
	else
	{
		gp2ap_als_ave_ado = ((( gp2ap_als_ave_als0 * alph ) - ( gp2ap_als_ave_als1 * beta )) >> ( GAMM_ADJUST_MAX - gamm )) >> 15 ;
	}

	if ( gp2ap_als_ave_ado > ADO_MAX )
	{
		gp2ap_als_ave_ado = ADO_MAX ;
	}

	if ( range_a != ( range_b & 0x07 ) )
	{
		value = range_b ;
		err = gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;
		if( err < 0 )
		{
			printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
		err = gp2ap_restart_device( data ) ;
		if( err < 0 )
		{
			printk( KERN_ERR "[%s][%d]gp2ap_restart_device failed. !! \n", __func__, __LINE__ ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
	}
	
	mutex_unlock( &data->mutex ) ;
	
	return count ;
}


/*Using S_IWUGO is not permitted because of kernel-3.18 specification*/
//static DEVICE_ATTR( als_delay, S_IRUGO|S_IWUGO, als_delay_show, als_delay_store ) ;
//static DEVICE_ATTR( als_enable, S_IRUGO|S_IWUGO, als_enable_show, als_enable_store ) ;
static DEVICE_ATTR( als_delay, S_IRUGO|S_IWUSR|S_IWGRP, als_delay_show, als_delay_store ) ;
static DEVICE_ATTR( als_enable, S_IRUGO|S_IWUSR|S_IWGRP, als_enable_show, als_enable_store ) ;
static DEVICE_ATTR( als_lux, S_IRUGO, als_lux_show, NULL ) ;
static DEVICE_ATTR( als_ado, S_IRUGO, als_ado_show, NULL ) ;
static DEVICE_ATTR( als_ave_als0, S_IRUGO, als_ave_als0_show, NULL ) ;
static DEVICE_ATTR( als_ave_als1, S_IRUGO, als_ave_als1_show, NULL ) ;
static DEVICE_ATTR( als_ave_ado, S_IRUGO, als_ave_ado_show, NULL ) ;
//static DEVICE_ATTR( als_wake, S_IWUGO, NULL, als_wake_store ) ;
static DEVICE_ATTR( als_wake, S_IWUSR|S_IWGRP, NULL, als_wake_store ) ;
static DEVICE_ATTR( als_raw_data, S_IRUGO, als_raw_data_show, NULL ) ;
static DEVICE_ATTR( als_avg, S_IRUGO, als_avg_show, NULL ) ;
//static DEVICE_ATTR( als_resolution, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, als_resolution_show, als_resolution_store ) ;
//static DEVICE_ATTR( als_range, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, als_range_show, als_range_store ) ;
static DEVICE_ATTR( als_range, S_IRUGO|S_IWUSR|S_IWGRP, als_range_show, als_range_store ) ;

static struct attribute *als_attributes[] =
{
	&dev_attr_als_delay.attr,
	&dev_attr_als_enable.attr,
	&dev_attr_als_lux.attr,
	&dev_attr_als_ado.attr,
	&dev_attr_als_ave_als0.attr,
	&dev_attr_als_ave_als1.attr,
	&dev_attr_als_ave_ado.attr,
	&dev_attr_als_wake.attr,
	&dev_attr_als_raw_data.attr,
	&dev_attr_als_avg.attr,
//	&dev_attr_als_resolution.attr,
	&dev_attr_als_range.attr,
	NULL
} ;

static struct attribute_group als_attribute_group =
{
	.attrs = als_attributes
} ;

static int als_input_init( struct gp2ap_data *data )
{
	struct input_dev   *dev ;
	int					err = 0 ;

	dev = input_allocate_device( ) ;
	if( !dev )
	{
		printk( KERN_ERR "[%s][%d]input_allocate_device error(%d)!!\n", __func__, __LINE__, err ) ;
		return -ENOMEM ;
	}

	set_bit( EV_ABS, dev->evbit ) ;
	input_set_capability( dev, EV_ABS, ABS_LUX_REPORT ) ;
	input_set_abs_params( dev, ABS_LUX_REPORT, 0, 0x7fffffff, 0, 0 ) ;
	input_set_capability( dev, EV_ABS, ABS_RUDDER ) ;
	input_set_abs_params( dev, ABS_RUDDER, 0, 0x1, 0, 0 ) ;
	input_set_capability( dev, EV_ABS, ABS_WAKE ) ;
	input_set_abs_params( dev, ABS_WAKE, 0, 0x7fffffff, 0, 0 ) ;
	input_set_capability( dev, EV_ABS, ABS_CONTROL_REPORT ) ;
	input_set_abs_params( dev, ABS_CONTROL_REPORT, 0, 0x1ffff, 0, 0 ) ;

	dev->name = LIGHT_SENSOR_NAME ;

	err = input_register_device( dev ) ;
	if( err < 0 )
	{
		input_free_device( dev ) ;
		printk( KERN_ERR "[%s][%d]input_register_device error(%d)!!\n", __func__, __LINE__, err ) ;
		return err ;
	}
	input_set_drvdata( dev, data ) ;

	data->als_input_dev = dev ;

	return 0 ;
}

static void als_data_polling( struct work_struct *work )
{
	struct gp2ap_data  *data = container_of( ( struct delayed_work * )work,
														struct gp2ap_data, als_work ) ;
	int		lux ;
#if defined(PSALS_RECOVERY_ENABLE)
	u8  val = 0;
	int err = 0;
#endif /* PSALS_RECOVERY_ENABLE */

	if( data != NULL )
	{
		mutex_lock( &data->mutex ) ;
		if(!data->als_enabled) {
			pr_debug("[%s][%d]polling als disabled.\n", __func__, __LINE__);
			mutex_unlock(&data->mutex);
			return;
		}
#if defined(PSALS_RECOVERY_ENABLE)
		if(PSALS_RECOVERY) {
			err = gp2ap_i2c_read(REG_ADR_00, &val, sizeof(val), data->client);
			if(err < 0 || ((err >= 0) && ((val & OP_RUN) == OP_SHUTDOWN))) {
				if(err < 0) {
					printk(KERN_ERR "[%s][%d]i2c read error. (%d)\n", __func__, __LINE__, lux);
				}
				if((err >= 0) && ((val & OP_RUN) == OP_SHUTDOWN)) {
					printk(KERN_ERR "[%s][%d]REG_ADR_00 shutdown bit is 0. (%d)\n", __func__, __LINE__, val);
				}
				psals_recovery_flg = 1;
				wake_lock(&gp2ap_recovery_wake_lock);
				queue_work(data->work_queue_recovery, &data->psals_recovery_work);
				mutex_unlock(&data->mutex);
				return;
			}
		}
#endif /* PSALS_RECOVERY_ENABLE */
		lux = als_get_lux( data ) ;
		mutex_unlock( &data->mutex ) ;

		if(lux >= 0) {
			input_report_abs( data->als_input_dev, ABS_LUX_REPORT, lux ) ;
			input_sync( data->als_input_dev ) ;
		}

		if (( data->als_delay - data->als_delay_gap ) > 0 )
		{
			schedule_delayed_work( &data->als_work, msecs_to_jiffies( data->als_delay - data->als_delay_gap ) ) ;
		}
		else
		{
			schedule_delayed_work( &data->als_work, 0 ) ;
		}
	}
}

/* *****************************************************************************
		Proximity Sensor
***************************************************************************** */
static char ps_distance_get_value(struct gp2ap_data *data)
{
	char value ;
	value = gpio_get_value_cansleep(data->ps_gpio) ;
	if (value) {
		value = 7; // 1 -> 7: away
	}
	return value;
}

static int ps_onoff( u8 onoff, struct gp2ap_data *data )
{
	u8			value ;
	u8			rdata ;
	int			i;
	int err = 0;
#ifdef PROX_USE_SMEM
    unsigned short proxadj[2];
    sharp_smem_common_type *p_sh_smem_common_type = NULL ;
#endif  /* PROX_USE_SMEM */

	pr_debug( "proximity_sensor onoff = %d\n", onoff ) ;

	if( onoff ) //1:PS-ON 0:PS-OFF
	{
#ifdef PROX_USE_SMEM
	    memset((void*)proxadj, 0x00, sizeof(proxadj)) ;
	    p_sh_smem_common_type = sh_smem_get_common_address() ;
#endif  /* PROX_USE_SMEM */

		if( !data->als_enabled ) // 1:als OFF 0:als ON 
		{ //als off PS ON

#ifdef GP2AP030_REGUSE
			err = regulator_enable(gp2ap_vdda);
			if (err) {
				printk(KERN_ERR "[%s][%d]regulator_enable failure. (%d)\n", __func__, __LINE__, err);
				return -1;
			}
#endif
			memset( &data->als_tv, 0, sizeof( struct timeval )) ;
			usleep( SENSOR_REGL_ON_DELAY ) ; // S0 -> S1
			
			for( i = 0 ; i < REG_ADR00_READ_COUNT ; i++ ) // S1 -> S3
			{
				value = OP_SHUTDOWN ;
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = PRST_1 ;
				err = gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( INTTYPE_L | RES_P_12 ) ;
				err = gp2ap_i2c_write( REG_ADR_02, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_02 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( INTVAL_16 | PIN_DETECT | IS_65 ) ;
				err = gp2ap_i2c_write( REG_ADR_03, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_03 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
#ifdef PROX_USE_SMEM
			    if ( p_sh_smem_common_type != NULL )
				{
			        if (( p_sh_smem_common_type->shdiag_proxadj[0] == 0      ) ||
						( p_sh_smem_common_type->shdiag_proxadj[0] == 0xFFFF ) ||
        				( p_sh_smem_common_type->shdiag_proxadj[1] == 0      ) ||
						( p_sh_smem_common_type->shdiag_proxadj[1] == 0xFFFF ))
					{
        				value = 0xFF ;
        				err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = 0xFF ;
        			    err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = 0xFF ;
        			    err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = 0xFF ;
        			    err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			}else {
        				memcpy( proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof( proxadj ) ) ;
        				value = (u8)( proxadj[0] & 0x000000ff ) ;
        				err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = (u8)(( proxadj[0] & 0x0000ff00 ) >> 8 ) ;
        			    err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = (u8)( proxadj[1] & 0x000000ff ) ;
        			    err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = (u8)(( proxadj[1] & 0x0000ff00) >> 8 ) ;
        			    err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			}
			    }else {
    				value = data->regData[REG_ADR_08] ;
    				err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
					if( err < 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
						return -1 ;
					}
    				value = data->regData[REG_ADR_09] ;
    				err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
					if( err < 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
						return -1 ;
					}
    				value = data->regData[REG_ADR_0A] ;
    				err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
					if( err < 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
						return -1 ;
					}
    				value = data->regData[REG_ADR_0B] ;
    				err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
					if( err < 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
						return -1 ;
					}
			    }
#else
				value = data->regData[REG_ADR_08] ;
				err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = data->regData[REG_ADR_09] ;
				err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = data->regData[REG_ADR_0A] ;
				err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = data->regData[REG_ADR_0B] ;
				err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
#endif  /* PROX_USE_SMEM */
				value = ( OP_RUN | OP_CONTINUOUS | OP_PS ) ; // PS mode
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				err = gp2ap_i2c_read( REG_ADR_00, &rdata, sizeof( rdata ), data->client ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 read err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				if ( rdata == OP_SHUTDOWN ) 
				{
					if ( i > 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_00 unregisted. !! Retry:%d\n", __func__, __LINE__ ,i ) ;
					}
					if ( i == 3 )
					{
						return -1;
					}
				}
				else
				{
					i = REG_ADR00_READ_COUNT ;
				}
			}
			usleep( SENSOR_ALS_PALS_DELAY ) ;
		}
		else
		{ //als ON ps ON
			for( i = 0 ; i < REG_ADR00_READ_COUNT ; i++ ) // S2 -> S4
			{
				value = OP_SHUTDOWN ; // shutdown
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( RES_A_14 | (data->regData[REG_ADR_01] & RANGE_A_128)) ;
				err = gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( INTTYPE_L | RES_P_12 ) ;
				err = gp2ap_i2c_write( REG_ADR_02, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_02 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = ( INTVAL_16 | PIN_DETECT | IS_65 ) ;
				err = gp2ap_i2c_write( REG_ADR_03, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_03 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
#ifdef PROX_USE_SMEM
			    if ( p_sh_smem_common_type != NULL )
				{
			        if (( p_sh_smem_common_type->shdiag_proxadj[0] == 0      ) ||
						( p_sh_smem_common_type->shdiag_proxadj[0] == 0xFFFF ) ||
						( p_sh_smem_common_type->shdiag_proxadj[1] == 0      ) ||
						( p_sh_smem_common_type->shdiag_proxadj[1] == 0xFFFF )) 
					{
        				value = 0xFF;
        				err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = 0xFF;
        			    err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = 0xFF;
        			    err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = 0xFF;
        			    err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			}else {
        				memcpy( proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof(proxadj) ) ;
        				value = (u8)( proxadj[0] & 0x000000ff ) ;
        				err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = (u8)(( proxadj[0] & 0x0000ff00) >> 8 ) ;
        			    err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = (u8)( proxadj[1] & 0x000000ff ) ;
        			    err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			    value = (u8)(( proxadj[1] & 0x0000ff00) >> 8 ) ;
        			    err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
						if( err < 0 )
						{
							printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
							return -1 ;
						}
        			}
			    }else {
    				value = data->regData[REG_ADR_08] ;
    				err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
					if( err < 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
						return -1 ;
					}
    				value = data->regData[REG_ADR_09] ;
    				err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
					if( err < 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
						return -1 ;
					}
    				value = data->regData[REG_ADR_0A] ;
    				err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
					if( err < 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
						return -1 ;
					}
    				value = data->regData[REG_ADR_0B] ;
    				err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
					if( err < 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
						return -1 ;
					}
			    }
#else
				value = data->regData[REG_ADR_08] ;
				err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = data->regData[REG_ADR_09] ;
				err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = data->regData[REG_ADR_0A] ;
				err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				value = data->regData[REG_ADR_0B] ;
				err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
#endif  /* PROX_USE_SMEM */
				
				value = ( OP_RUN | OP_CONTINUOUS | INT_ALCLEAR ) ; // PS mode
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				err = gp2ap_i2c_read( REG_ADR_00, &rdata, sizeof( rdata ), data->client ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 read err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				if ( rdata == OP_SHUTDOWN ) 
				{
					if ( i > 0 )
					{
						printk( KERN_ERR "[%s][%d]REG_ADR_00 unregisted. !! Retry:%d\n", __func__, __LINE__ ,i ) ;
					}
					if ( i == 3 )
					{
						return -1;
					}
				}
				else
				{
					i = REG_ADR00_READ_COUNT ;
				}
			}
			do_gettimeofday( &data->als_tv ) ;
			usleep( SENSOR_ALS_PALS_DELAY ) ;
		}
	}
	else
	{
		memset( &data->als_tv, 0, sizeof( struct timeval )) ;
		
		if( !data->als_enabled ) // 1:als OFF 0:als ON
		{//als off PS OFF
#if defined(PSALS_RECOVERY_ENABLE)
			if(PSALS_RECOVERY) {

				value = OP_RUN;
				err = gp2ap_i2c_write(REG_ADR_00, &value, data->client, data);
				if(err < 0) {
					printk(KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__);
				}
				usleep(SENSOR_REGL_OFF_DELAY);

				value = OP_SHUTDOWN; // shutdown
				err = gp2ap_i2c_write(REG_ADR_00, &value, data->client, data);
				if(err < 0) {
					printk(KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__);
				}

#ifdef GP2AP030_REGUSE
				err = regulator_disable(gp2ap_vdda);
				if(err) {
					printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, err);
					return -1;
				}
				for(i = 0; i < sizeof(gp2ap_init_data); i++) {
					data->regData[i] = gp2ap_init_data[i];
				}
#endif
			}
			else {
				value = OP_RUN ;
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
				usleep ( SENSOR_REGL_OFF_DELAY ) ;
				value = OP_SHUTDOWN ; // shutdown
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
					return -1;
				}
#ifdef GP2AP030_REGUSE
				err = regulator_disable(gp2ap_vdda);
				if (err) {
					printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, err);
					return -1;
				}
				for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
				{
					data->regData[i] = gp2ap_init_data[i] ;
				}
#endif
			}
#else
			value = OP_RUN ;
			err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			usleep ( SENSOR_REGL_OFF_DELAY ) ;
			value = OP_SHUTDOWN ; // shutdown
			err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
#ifdef GP2AP030_REGUSE
			err = regulator_disable(gp2ap_vdda);
			if (err) {
				printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, err);
				return -1;
			}
			for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
			{
				data->regData[i] = gp2ap_init_data[i] ;
			}
#endif
#endif /* PSALS_RECOVERY_ENABLE */
		}
		else
		{//als ON PS OFF S4 -> S2
			value = OP_SHUTDOWN ;
			err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			value = ( PRST_1 | RES_A_16 | (data->regData[REG_ADR_01] & RANGE_A_128)) ;
			err = gp2ap_i2c_write( REG_ADR_01, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_01 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			value = ( INTTYPE_L | RES_P_12 ) ;
			err = gp2ap_i2c_write( REG_ADR_02, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_02 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			value = ( INTVAL_0 | PIN_INT_PS ) ;
			err = gp2ap_i2c_write( REG_ADR_03, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_03 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
			value = ( OP_RUN | OP_CONTINUOUS | OP_ALS ) ; // ALS mode
			err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
			if( err < 0 )
			{
				printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
				return -1;
			}
		}
	}

	return 0 ;
}

static int ps_enable_irq( struct gp2ap_data *data )
{
	unsigned long  flags ;
	
	spin_lock_irqsave( &( data->ps_lock ), flags ) ;

	if ( !data->ps_irq_enabled )
	{
		enable_irq( data->ps_irq ) ;
	}
	data->ps_irq_enabled = 1 ;
	
	spin_unlock_irqrestore( &( data->ps_lock ), flags ) ;

	return 0 ;
}

static int ps_disable_irq( struct gp2ap_data *data )
{
	unsigned long  flags ;
	
	spin_lock_irqsave( &( data->ps_lock ), flags ) ;

	if ( data->ps_irq_enabled )
	{
		disable_irq_nosync( data->ps_irq ) ;
	}
	data->ps_irq_enabled = 0 ;
	
	spin_unlock_irqrestore( &( data->ps_lock ), flags ) ;

	return 0 ;
}

static ssize_t
ps_enable_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	int					enabled ;

	pr_debug( "ps_enable_show \n" ) ;

	mutex_lock( &data->mutex ) ;
	enabled = data->ps_enabled ;
	mutex_unlock( &data->mutex ) ;

    return sprintf( buf, "%d", enabled ) ;
}

static ssize_t
ps_enable_store( struct device *dev,
					struct device_attribute *attr,
						const char *buf,
							size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	char			   *endp ;
	int					enabled = simple_strtoul( buf, &endp, 10 ) ;
	char				value ;
	int					err = 0;
#if defined(PSALS_RECOVERY_ENABLE)
	int					i;
#endif /* PSALS_RECOVERY_ENABLE */

	pr_debug( "ps_enable_store data=%s\n", buf ) ;

	if( !( endp != buf &&
			( endp == ( buf + strlen( buf ) )
				|| ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}
	if( enabled != 0 && enabled != 1 )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}

	mutex_lock( &data->mutex ) ;

	if( data->ps_enabled && !enabled )
	{
#if defined(PSALS_RECOVERY_ENABLE)
		if(PSALS_RECOVERY) {
			cancel_delayed_work(&data->ps_polling_work);
		}
#endif /* PSALS_RECOVERY_ENABLE */
		ps_disable_irq( data ) ;
		err = ps_onoff( 0, data ) ;
		if(err < 0) {
			printk(KERN_ERR "[%s][%d]ps power off failed.\n", __func__, __LINE__);
			mutex_unlock(&data->mutex);
			return -1;
		}
		data->ps_enabled_prev = data->ps_enabled ;
		data->ps_enabled = enabled ;
		data->ps_distance = 7 ;
		pr_debug( "proximity_sensor disable!! \n" ) ;
		input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) ) ;
	}
	else if( !data->ps_enabled && enabled )
	{
#if defined(PSALS_RECOVERY_ENABLE)
		if(PSALS_RECOVERY) {
			for(i = 0; i < RECOVERY_RETRY_COUNT; i++) {
				err = ps_onoff(1, data);
				if(err == 0) {
					printk(KERN_ERR "[%s][%d]ps on success. (%d)\n", __func__, __LINE__, err);
					break;
				}
			}
		}
		else {
			err = ps_onoff(1, data);
		}
#else
		err = ps_onoff( 1, data ) ;
#endif /* PSALS_RECOVERY_ENABLE */
		if(err < 0) {
			printk(KERN_ERR "[%s][%d]ps power on failed.\n", __func__, __LINE__);
			mutex_unlock(&data->mutex);
			return -1;
		}
		data->ps_enabled_prev = data->ps_enabled ;
		data->ps_enabled = enabled ;
		value = ps_distance_get_value(data) ;
		data->ps_distance = value;
		ps_enable_irq( data ) ;
#if defined(PSALS_RECOVERY_ENABLE)
		if(PSALS_RECOVERY) {
			if((bootmode != SH_BOOT_D) && (bootmode != SH_BOOT_F_F)) {
				schedule_delayed_work(&data->ps_polling_work, 0);
			}
		}
#endif /* PSALS_RECOVERY_ENABLE */
		pr_debug( "proximity_sensor enable!! \n" ) ;
		input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) ) ;
		input_report_abs( input_dev, ABS_DISTANCE_REPORT, -1 ) ;
		input_report_abs( input_dev, ABS_DISTANCE_REPORT, value ) ;
		input_sync( input_dev ) ;
	}

	mutex_unlock( &data->mutex ) ;

	return count ;
}

static ssize_t
ps_raw_data_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	int					D2_data ;
	u8					rdata[2] ;
	int err = 0 ;

	pr_debug( "ps_raw_data_show\n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	err = gp2ap_i2c_read( REG_ADR_10, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_10 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	mutex_unlock( &data->mutex ) ;

	D2_data = ( rdata[1] << 8 ) | rdata[0] ;

	return sprintf( buf, "%d", D2_data ) ;
}

static ssize_t
ps_distance_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	int					distance ;

	pr_debug( "ps_distance_show\n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	distance = data->ps_distance ;
	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", distance) ;
}

static ssize_t
ps_wake_store( struct device *dev,
					struct device_attribute *attr,
						const char *buf,
							size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata(input_dev);
	static int			cnt = 1 ;

	pr_debug( "ps_wake_store data=%s\n", buf ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		return ERR_PSALS_STATE_DISABLED;
	}

	input_report_abs( input_dev, ABS_WAKE, cnt++ ) ;

	return count ;
}

static ssize_t
ps_threshold_show( struct device *dev,
						struct device_attribute *attr,
							char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					rdata[2] ;
	u32					th_l ;
	u32					th_h ;
	int err = 0 ;

	pr_debug( "ps_threshold_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	err = gp2ap_i2c_read( REG_ADR_08, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_08 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	th_l = ( rdata[1] << 8 ) | rdata[0] ;
	err = gp2ap_i2c_read( REG_ADR_0A, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_0A read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	th_h = ( rdata[1] << 8 ) | rdata[0] ;

	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d,%d", th_l, th_h ) ;
}

static ssize_t
ps_threshold_store( struct device *dev,
					struct device_attribute *attr,
						const char *buf,
							size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					value ;
	char			   *th_lp ;
	char			   *th_hp ;
	u32					th_l ;
	u32					th_h ;
	char			   *endp ;
	char				tmpbuf[16] ;
	int err = 0 ;

	pr_debug( "ps_threshold_store data=%s\n", buf ) ;

	if( strlen( buf ) > 15 )
	{
		printk( KERN_ERR "[%s][%d]invalid threshold (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return ERR_PSALS_STATE_DISABLED;
	}

	memset( tmpbuf, 0x00, sizeof( tmpbuf ) ) ;
	memcpy( tmpbuf, buf, strlen( buf ) ) ;

	th_lp = strtok( tmpbuf, "," ) ;
	th_hp = strtok( NULL, NULL ) ;

	if( th_lp == NULL || th_hp == NULL )
	{
		printk( KERN_ERR "[%s][%d]invalid threshold (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	th_l = simple_strtoul( th_lp, &endp, 10 ) ;
	if( !( endp != th_lp &&
			( endp == ( th_lp + strlen( th_lp ) )
				|| ( endp == ( th_lp + strlen( th_lp ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid threshold (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	if( th_l < 0 || th_l > 0xffff )
	{
		printk( KERN_ERR "[%s][%d]invalid threshold (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	th_h = simple_strtoul( th_hp, &endp, 10 ) ;
	if( !( endp != th_hp &&
			( endp == ( th_hp + strlen( th_hp ) )
				|| ( endp == ( th_hp + strlen( th_hp ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid threshold (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	if( th_h < 0 || th_h > 0xffff )
	{
		printk( KERN_ERR "[%s][%d]invalid threshold (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	pr_debug( "  th_l=%d,th_h=%d\n", th_l, th_h ) ;

	value = ( OP_SHUTDOWN | INT_NOCLEAR ) ;	// shutdown
	err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	value = ( u8 )( th_l & 0x000000ff ) ;
	err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	value = ( u8 )( ( th_l  & 0x0000ff00 ) >> 8 ) ;
	err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	value = ( u8 )( th_h & 0x000000ff ) ;
	err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	value = ( u8 )( ( th_h  & 0x0000ff00 ) >> 8 ) ;
	err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	err = gp2ap_restart_device( data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]gp2ap_restart_device failed. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	mutex_unlock( &data->mutex ) ;

	return count ;
}

static ssize_t
ps_resolution_show( struct device *dev,
						struct device_attribute *attr,
							char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					rdata ;
	u32					res_p ;
	int err = 0 ;

	pr_debug( "ps_resolution_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	err = gp2ap_i2c_read( REG_ADR_02, &rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_02 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	res_p = ( rdata & 0x38 ) >> 3 ;

	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", res_p ) ;
}

static ssize_t
ps_resolution_store( struct device *dev,
						struct device_attribute *attr,
							const char *buf,
								size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					value ;
	u8					rdata ;
	char			   *endp ;
	u32					res_p = simple_strtoul( buf, &endp, 10 ) ;
	int err = 0 ;

	pr_debug( "ps_resolution_store data=%s\n", buf ) ;

	if( !( endp != buf &&
			( endp == ( buf + strlen( buf ) )
				|| ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid resolution (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}
	if( res_p < 0 || res_p > 7 )
	{
		printk( KERN_ERR "[%s][%d]invalid resolution (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return ERR_PSALS_STATE_DISABLED;
	}

	value = ( OP_SHUTDOWN | INT_NOCLEAR ) ;	// shutdown
	err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	err = gp2ap_i2c_read( REG_ADR_02, &rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_02 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	value = ( rdata & 0xC7 ) | ( res_p << 3 ) ;
	err = gp2ap_i2c_write( REG_ADR_02, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_02 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	err = gp2ap_restart_device( data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]gp2ap_restart_device failed. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	mutex_unlock( &data->mutex ) ;

	return count ;
}

static ssize_t
ps_range_show( struct device *dev,
					struct device_attribute *attr,
						char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					rdata ;
	u32					range_p ;
	int err = 0 ;

	pr_debug( "ps_range_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	err = gp2ap_i2c_read( REG_ADR_02, &rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_02 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	range_p = rdata & 0x07 ;

	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%d", range_p ) ;
}

static ssize_t
ps_range_store( struct device *dev,
					struct device_attribute *attr,
						const char *buf,
							size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					value ;
	u8					rdata ;
	char			   *endp ;
	u32					range_p = simple_strtoul( buf, &endp, 10 ) ;
	int err = 0 ;

	pr_debug( "ps_range_store data=%s\n", buf ) ;

	if( !( endp != buf &&
			( endp == ( buf + strlen( buf ) )
				|| ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid range (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}
	if( range_p < 0 || range_p > 7 )
	{
		printk( KERN_ERR "[%s][%d]invalid range (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return ERR_PSALS_STATE_DISABLED;
	}

	value = ( OP_SHUTDOWN | INT_NOCLEAR ) ;	// shutdown
	err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	err = gp2ap_i2c_read( REG_ADR_02, &rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_02 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	value = ( rdata & 0xF8 ) | range_p ;
	err = gp2ap_i2c_write( REG_ADR_02, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_02 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	err = gp2ap_restart_device( data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]gp2ap_restart_device failed. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	mutex_unlock( &data->mutex ) ;

	return count ;
}

static ssize_t
ps_calibration_show( struct device *dev,
						struct device_attribute *attr,
							char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					rdata[2] ;
	int err = 0 ;

	pr_debug( "ps_calibration_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	err = gp2ap_i2c_read( REG_ADR_10, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_10 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	data->ps_calibration = ( rdata[1] << 8 ) | rdata[0] ;

	mutex_unlock( &data->mutex ) ;

	pr_debug( "  calibration=%d\n", data->ps_calibration ) ;

	return 0 ;
}

static ssize_t
ps_calibration_store( struct device *dev,
						struct device_attribute *attr,
							const char *buf,
								size_t count )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	u8					value ;
	char			   *cmdp ;
	char			   *offsetp ;
	u32					cmd ;
	u32					offset ;
	u8					rdata[2] ;
	u32					th_l ;
	u32					th_h ;
	char				tmpbuf[16] ;
	char			   *endp ;
	int err = 0 ;

	pr_debug( "ps_calibration_store data=%s\n", buf ) ;

	if( strlen( buf ) > 15 )
	{
		return count ;
	}

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled) {
		printk(KERN_ERR "[%s][%d]ps is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return ERR_PSALS_STATE_DISABLED;
	}

	memset( tmpbuf, 0x00, sizeof( tmpbuf ) ) ;
	memcpy( tmpbuf, buf, strlen( buf ) ) ;

	cmdp = strtok( tmpbuf, "," ) ;
	if( cmdp == NULL )
	{
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	cmd = simple_strtoul( cmdp, &endp, 10 ) ;
	if( !( endp != cmdp &&
			( endp == ( cmdp + strlen( cmdp ) )
				|| ( endp == ( cmdp + strlen( cmdp ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	if( cmd == 1 )
	{
		offsetp = strtok( NULL, NULL ) ;
		if( offsetp == NULL )
		{
			printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
		offset = simple_strtoul( offsetp, &endp, 10 ) ;
		if( !( endp != offsetp &&
				( endp == ( offsetp + strlen( offsetp ) )
					|| ( endp == ( offsetp + strlen( offsetp ) - 1 ) && *endp == '\n' ) ) ) )
		{
			printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
		if( offset < 0 || offset > 0xffff )
		{
			printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
	}
	else if( cmd == 0 )
	{
		offsetp = strtok( NULL, NULL ) ;
		if( offsetp != NULL )
		{
			printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
		offset = data->ps_calibration ;
	}
	else
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	pr_debug( "  cmd=%d,offset=%d\n", cmd, offset ) ;

	err = gp2ap_i2c_read( REG_ADR_08, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_08 read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	th_l = ( rdata[1] << 8 ) | rdata[0] ;
	err = gp2ap_i2c_read( REG_ADR_0A, rdata, sizeof( rdata ), data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_0A read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	th_h = ( rdata[1] << 8 ) | rdata[0] ;

	th_l += offset ;
	th_h += offset ;
	if( th_l > 0xffff )
	{
		th_l = 0xffff ;
	}
	if( th_h > 0xffff )
	{
		th_h = 0xffff ;
	}

	value = ( OP_SHUTDOWN | INT_NOCLEAR ) ;	// shutdown
	err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	value = ( u8 )( th_l & 0x000000ff ) ;
	err = gp2ap_i2c_write( REG_ADR_08, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_08 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	value = ( u8 )( ( th_l  & 0x0000ff00 ) >> 8 ) ;
	err = gp2ap_i2c_write( REG_ADR_09, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_09 write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	value = ( u8 )( th_h & 0x000000ff ) ;
	err = gp2ap_i2c_write( REG_ADR_0A, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_0A write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	value = ( u8 )( ( th_h  & 0x0000ff00 ) >> 8 ) ;
	err = gp2ap_i2c_write( REG_ADR_0B, &value, data->client, data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]REG_ADR_0B write err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	err = gp2ap_restart_device( data ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]gp2ap_restart_device failed. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	mutex_unlock( &data->mutex ) ;

	return count ;
}

static ssize_t
ps_dataread_show( struct device *dev,
						struct device_attribute *attr,
							char *buf )
{
	struct input_dev   *input_dev = to_input_dev( dev ) ;
	struct gp2ap_data  *data = input_get_drvdata( input_dev ) ;
	int value = 7 ;
	int nRetry = 0 ;
	int ret = 0 ;
	int prox_data = 0 ;
	
	pr_debug( "ps_dataread_show\n" ) ;
	
	mutex_lock( &data->mutex ) ;

	if( !data->ps_enabled ){
		ret = ps_onoff( 1, data ) ;
		if(ret < 0){
			printk(KERN_ERR "[%s][%d] ps_power_on_filed \n", __func__, __LINE__);
			mutex_unlock( &data->mutex ) ;
			return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
		}
	}
	value = ps_distance_get_value(data) ;
	if (value == 0) {	/*Near*/
		prox_data = 0;
	}else {				/*Far*/
		prox_data = 7;
	}
	
	if( !data->ps_enabled ){
#if defined(PSALS_RECOVERY_ENABLE)
		if(PSALS_RECOVERY) {
			ret = ps_onoff(0, data);
		}
		else {
			for(nRetry=0;nRetry<20;nRetry++) {
				ret = ps_onoff( 0, data ) ;
				if(ret == 0){
					pr_debug( "ps_power_off_sucess \n" ) ;
					break;
				}
				if (nRetry < 19) usleep(100 * 1000);
			}
		}
#else
		for(nRetry=0;nRetry<20;nRetry++) {
			ret = ps_onoff( 0, data ) ;
			if(ret == 0){
				pr_debug( "ps_power_off_sucess \n" ) ;
				break;
			}
			if (nRetry < 19) usleep(100 * 1000);
		}
#endif /* PSALS_RECOVERY_ENABLE */
		if( ret < 0 ){
			printk(KERN_ERR "[%s][%d] ps_power_off_filed \n", __func__, __LINE__);
			mutex_unlock( &data->mutex ) ;
			return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
		}
	}
	
	mutex_unlock( &data->mutex ) ;
	return sprintf( buf, "%d", prox_data ) ;
}

static DEVICE_ATTR( ps_enable, S_IRUGO|S_IWUSR|S_IWGRP, ps_enable_show, ps_enable_store ) ;
static DEVICE_ATTR( ps_raw_data, S_IRUGO, ps_raw_data_show, NULL ) ;
static DEVICE_ATTR( ps_distance, S_IRUGO, ps_distance_show, NULL ) ;
static DEVICE_ATTR( ps_wake, S_IWUSR|S_IWGRP, NULL, ps_wake_store ) ;
static DEVICE_ATTR( ps_threshold, S_IRUGO|S_IWUSR|S_IWGRP, ps_threshold_show, ps_threshold_store ) ;
static DEVICE_ATTR( ps_resolution, S_IRUGO|S_IWUSR|S_IWGRP, ps_resolution_show, ps_resolution_store ) ;
static DEVICE_ATTR( ps_range, S_IRUGO|S_IWUSR|S_IWGRP, ps_range_show, ps_range_store ) ;
static DEVICE_ATTR( ps_calibration, S_IRUGO|S_IWUSR|S_IWGRP, ps_calibration_show, ps_calibration_store ) ;
static DEVICE_ATTR( ps_dataread, S_IRUGO, ps_dataread_show, NULL ) ;

static struct attribute *ps_attributes[] =
{
	&dev_attr_ps_enable.attr,
	&dev_attr_ps_raw_data.attr,
	&dev_attr_ps_distance.attr,
	&dev_attr_ps_wake.attr,
	&dev_attr_ps_threshold.attr,
	&dev_attr_ps_resolution.attr,
	&dev_attr_ps_range.attr,
	&dev_attr_ps_calibration.attr,
	&dev_attr_ps_dataread.attr,
	NULL
} ;

static struct attribute_group ps_attribute_group =
{
	.attrs = ps_attributes
} ;

static int ps_input_init( struct gp2ap_data *data )
{
	struct input_dev   *dev ;
	int					err = 0 ;

	dev = input_allocate_device( ) ;
	if( !dev )
	{
		printk( KERN_ERR "[%s][%d]input_allocate_device error!!\n", __func__, __LINE__ ) ;
		return -ENOMEM ;
	}

	set_bit( EV_ABS, dev->evbit ) ;
	input_set_capability( dev, EV_ABS, ABS_DISTANCE_REPORT ) ;
	input_set_abs_params( dev, ABS_DISTANCE_REPORT, 0, 7, 0, 0 ) ;
	input_set_capability( dev, EV_ABS, ABS_RUDDER ) ;
	input_set_abs_params( dev, ABS_RUDDER, 0, 0x1, 0, 0 ) ;
	input_set_capability( dev, EV_ABS, ABS_WAKE ) ;
	input_set_abs_params( dev, ABS_WAKE, 0, 0x7fffffff, 0, 0 ) ;
	input_set_capability( dev, EV_ABS, ABS_CONTROL_REPORT ) ;
	input_set_abs_params( dev, ABS_CONTROL_REPORT, 0, 0x1ffff, 0, 0 ) ;

	dev->name = PROXIMITY_SENSOR_NAME ;

	err = input_register_device( dev ) ;
	if( err < 0 )
	{
		input_free_device( dev ) ;
		printk( KERN_ERR "[%s][%d]input_register_device error(%d)!!\n", __func__, __LINE__, err ) ;
		return err ;
	}
	input_set_drvdata( dev, data ) ;

	data->ps_input_dev = dev ;

	return 0 ;
}

irqreturn_t ps_irq_handler( int irq, void *dev_id )
{
	struct gp2ap_data   *data = dev_id ;
	
	pr_debug( "ps_irq_handler\n" ) ;
	wake_lock( &gp2ap_ps_wake_lock ) ;
	
	ps_disable_irq( data ) ;
	
	schedule_work( &data->ps_int_work ) ;
	return IRQ_HANDLED ;
}

static void ps_work_int_func( struct work_struct *work )
{
	struct gp2ap_data  *data = container_of( ( struct work_struct * )work,
												struct gp2ap_data, ps_int_work ) ;
	char				distance ;

	if( data == NULL )
	{
		return ;
	}

	mutex_lock( &data->mutex ) ;

	// 0 : proximity, 7 : away
	distance = ps_distance_get_value(data) ;
	pr_debug( "proximity = %d\n", distance ) ;
	data->ps_distance = distance;

	wake_lock_timeout( &gp2ap_ps_timeout_wake_lock, 1 * HZ ) ;
	input_report_abs( data->ps_input_dev, ABS_DISTANCE_REPORT, distance ) ;
	input_sync( data->ps_input_dev ) ;

	ps_enable_irq( data ) ;
	
	wake_unlock( &gp2ap_ps_wake_lock ) ;
	mutex_unlock( &data->mutex ) ;

}

#if defined(PSALS_RECOVERY_ENABLE)
static void ps_recovery_polling(struct work_struct *work) {

	struct gp2ap_data  *data = container_of((struct delayed_work *)work, struct gp2ap_data, ps_polling_work);
	int err = 0;
	u8  val = 0;

	pr_debug("[%s]ps recovery polling start.\n", __func__);

	if(data != NULL) {
		mutex_lock(&data->mutex);
		if(!data->ps_enabled) {
			pr_debug("[%s][%d]polling ps disabled.\n", __func__, __LINE__);
			mutex_unlock(&data->mutex);
			return;
		}
		err = gp2ap_i2c_read(REG_ADR_00, &val, sizeof(val), data->client);

		if(err < 0 || ((err >= 0) && ((val & OP_RUN) == OP_SHUTDOWN))) {
			if(err < 0) {
				printk(KERN_ERR "[%s][%d]i2c read error. (%d)\n", __func__, __LINE__, err);
			}
			if((err >= 0) && ((val & OP_RUN) == OP_SHUTDOWN)) {
				printk(KERN_ERR "[%s][%d]REG_ADR_00 shutdown bit is 0. (%d)\n", __func__, __LINE__, val);
			}
			psals_recovery_flg = 1;
			wake_lock(&gp2ap_recovery_wake_lock);
			queue_work(data->work_queue_recovery, &data->psals_recovery_work);
			mutex_unlock(&data->mutex);
			return;
		}
		mutex_unlock( &data->mutex ) ;

		schedule_delayed_work(&data->ps_polling_work, msecs_to_jiffies(PSALS_RECOVERY_POLLING_DELAY));
	}
	pr_debug("[%s]ps recovery polling end.\n", __func__);
}

static void psals_recovery_func(struct work_struct *work) {

	struct gp2ap_data  *data = container_of((struct work_struct *)work, struct gp2ap_data, psals_recovery_work);
	char   value;
	int    ps_recovery_state_flg  = 0;
	int    als_recovery_state_flg = 0;
	int    ret;
	int    i;

	pr_debug("[%s]ps_recovery_func start.\n", __func__);

	mutex_lock(&data->mutex);

	if(!psals_recovery_flg) {
		pr_debug("[%s]psals was recoveried!! \n", __func__);
		mutex_unlock(&data->mutex);
		
		return;
	}

	if(data->als_enabled) {
		cancel_delayed_work(&data->als_work);
		data->als_mode = LOW_LUX_MODE;
		data->als_enabled_prev = data->als_enabled;
		data->als_enabled = 0;
		ret = als_onoff(0, data);
		if(ret < 0) {
			printk(KERN_ERR "[%s][%d]als power off failed.\n", __func__, __LINE__);
		}
		als_recovery_state_flg = 1;
		pr_debug("[%s]light sensor disable!! \n", __func__);
	}

	if(data->ps_enabled) {
		cancel_delayed_work(&data->ps_polling_work);
		ps_disable_irq(data);
		ret = ps_onoff(0, data);
		if(ret < 0) {
			printk(KERN_ERR "[%s][%d]ps power off failed.\n", __func__, __LINE__);
		}
		data->ps_enabled_prev = data->ps_enabled;
		data->ps_enabled = 0;
		data->ps_distance = 7;
		ps_recovery_state_flg = 1;
		pr_debug("[%s]proximity sensor disable!! \n", __func__);
	}

	usleep(PSALS_RECOVERY_DELAY);

	if(als_recovery_state_flg) {
		for(i = 0; i < RECOVERY_RETRY_COUNT; i++) {
			ret = als_onoff(1, data);
			if(ret == 0) {
				pr_debug("[%s][%d]als on success. (%d)\n", __func__, __LINE__, ret);
				break;
			}
			pr_debug("[%s]als power on retry.(%d) \n", __func__, i);
		}
		if(ret < 0) {
			printk(KERN_ERR "[%s][%d]als power on failed.\n", __func__, __LINE__);
			goto recovery_err;
		}
		data->als_enabled_prev = data->als_enabled;
		data->als_enabled = 1;
		if((bootmode != SH_BOOT_D) && (bootmode != SH_BOOT_F_F)) {
			schedule_delayed_work(&data->als_work, 0);
		}
		als_recovery_state_flg = 0;
		pr_debug("[%s]light sensor enable!! \n", __func__);
	}

	if(ps_recovery_state_flg) {
		for(i = 0; i < RECOVERY_RETRY_COUNT; i++) {
			ret = ps_onoff(1, data);
			if(ret == 0) {
				pr_debug("[%s][%d]ps on success. (%d)\n", __func__, __LINE__, ret);
				break;
			}
			pr_debug("[%s]ps power on retry.(%d) \n", __func__, i);
		}
		if(ret < 0) {
			printk(KERN_ERR "[%s][%d]ps power on failed.\n", __func__, __LINE__);
			goto recovery_err;
		}
		data->ps_enabled_prev = data->ps_enabled;
		data->ps_enabled = 1;
		value = ps_distance_get_value(data);
		data->ps_distance = value;
		ps_enable_irq(data);
		if((bootmode != SH_BOOT_D) && (bootmode != SH_BOOT_F_F)) {
			schedule_delayed_work(&data->ps_polling_work, 0);
		}
		ps_recovery_state_flg = 0;
		pr_debug("[%s]proximity sensor enable!! \n", __func__);
	}
	
	psals_recovery_flg = 0;

	mutex_unlock(&data->mutex);
	wake_unlock(&gp2ap_recovery_wake_lock);

	return;

recovery_err:
#ifdef GP2AP030_REGUSE
	ret = regulator_disable(gp2ap_vdda);
	if (ret)
	{
		printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, ret);
	}
	for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
	{
		data->regData[i] = gp2ap_init_data[i] ;
	}
#endif
	if(data->als_enabled) {
		cancel_delayed_work(&data->als_work);
		data->als_mode = LOW_LUX_MODE;
		data->als_enabled_prev = data->als_enabled;
		data->als_enabled = 0;
		printk(KERN_ERR "[%s]recovery err. light sensor disable!! \n", __func__);
	}

	ps_recovery_state_flg = 0;
	psals_recovery_flg = 0;

	mutex_unlock(&data->mutex);
	wake_unlock(&gp2ap_recovery_wake_lock);

	return;
}
#endif /* PSALS_RECOVERY_ENABLE */

/* *****************************************************************************
		GP2AP03000F
***************************************************************************** */
#if 0
static void gp2ap_init_device( struct gp2ap_data *data )
{
	int		i ;
	int err = 0 ;

	for( i = 1 ; i < sizeof( data->regData ) ; i++ )
	{
		gp2ap_i2c_write( i, data->regData[i], data->client, data ) ;
	}
}
#endif

static int gp2ap_restart_device( struct gp2ap_data *data )
{
	u8		value ;
	int err = 0 ;

	if( data->als_enabled && data->ps_enabled )
	{
		value = ( OP_RUN | OP_CONTINUOUS | OP_PS_ALS | INT_NOCLEAR ) ;
		err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
		if( err < 0 )
		{
			printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
			return -1 ;
		}
	}
	else if( data->als_enabled && !data->ps_enabled )
	{
		value = ( OP_RUN | OP_CONTINUOUS | OP_ALS | INT_NOCLEAR ) ;
		gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
		if( err < 0 )
		{
			printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
			return -1 ;
		}
	}
	else if( !data->als_enabled && data->ps_enabled )
	{
		value = ( OP_RUN | OP_CONTINUOUS | OP_PS | INT_NOCLEAR ) ;
		gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
		if( err < 0 )
		{
			printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
			return -1 ;
		}
	}
	return 0 ;
}

static ssize_t
gp2ap_reg_address_show( struct class *class,
                    struct class_attribute *attr,
                        char *buf )
{
	struct gp2ap_data  *data = platform_get_drvdata( gp2ap_pdev ) ;
	int					address ;

	pr_debug( "gp2ap_reg_address_show \n" ) ;

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled && !data->als_enabled) {
		printk(KERN_ERR "[%s][%d]psals is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	address = reg_address_data ;
	mutex_unlock( &data->mutex ) ;

	return sprintf( buf, "%2x", address ) ;
}

static ssize_t
gp2ap_reg_address_store( struct class *class,
                    struct class_attribute *attr,
                        const char *buf,
                            size_t count )
{
	struct gp2ap_data  *data = platform_get_drvdata( gp2ap_pdev ) ;
	char			   *endp ;
	int					address = simple_strtoul( buf, &endp, 16 ) ;

	pr_debug( "gp2ap_reg_address_store data=%s\n", buf ) ;

	if( !( endp != buf &&
			( endp == ( buf + strlen( buf ) )
				|| ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}
	if( (address < 0) || (( PSALS_ADDRESS_MAX - 1 ) < address) )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled && !data->als_enabled) {
		printk(KERN_ERR "[%s][%d]psals is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return ERR_PSALS_STATE_DISABLED;
	}

	reg_address_data = address ;
	mutex_unlock( &data->mutex ) ;

	return count ;
}

static ssize_t
gp2ap_reg_count_store( struct class *class,
                    struct class_attribute *attr,
                        const char *buf,
                            size_t count )
{
	struct gp2ap_data  *data = platform_get_drvdata( gp2ap_pdev ) ;
	char			   *endp ;
	u8					reg_count = simple_strtoul( buf, &endp, 10 ) ;

	pr_debug( "gp2ap_reg_count_store data=%s\n", buf ) ;

	if( !( endp != buf &&
			( endp == ( buf + strlen( buf ) )
				|| ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}
	if( (reg_count < 1) || ((PSALS_ADDRESS_MAX - reg_address_data) < reg_count) )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled && !data->als_enabled) {
		printk(KERN_ERR "[%s][%d]psals is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return ERR_PSALS_STATE_DISABLED;
	}

	reg_count_data = reg_count ;
	mutex_unlock( &data->mutex ) ;

	return count ;
}

static ssize_t
gp2ap_reg_show( struct class *class,
                    struct class_attribute *attr,
                        char *buf )
{
	struct gp2ap_data  *data ;
	u8					rdata[18] ;
	int i ;
	char rdata_buf[5] = {0};
	char rbuf[90] = {0};
	int err = 0 ;

	pr_debug( "gp2ap_reg_show\n" ) ;
	data = platform_get_drvdata( gp2ap_pdev ) ;

	if( reg_count_data > (PSALS_ADDRESS_MAX - reg_address_data )){
		reg_count_data = (PSALS_ADDRESS_MAX - reg_address_data );
	}
	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled && !data->als_enabled) {
		printk(KERN_ERR "[%s][%d]psals is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return sprintf(buf, "%d", ERR_PSALS_STATE_DISABLED);
	}

	err = gp2ap_i2c_read( reg_address_data, rdata, reg_count_data, data->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]reg_address_data read err. !! \n", __func__, __LINE__ ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	mutex_unlock( &data->mutex ) ;

	for(i = 0 ;i < reg_count_data ; i++){
		if(i == (reg_count_data - 1)){
			sprintf( rdata_buf, "%02x",rdata[i]);
			strcat(rbuf,rdata_buf);
			break;
		}
		sprintf(rdata_buf, "%02x,",rdata[i]);
		strcat(rbuf,rdata_buf);
	}

	return sprintf( buf,"%s",rbuf);
}

static ssize_t
gp2ap_reg_store( struct class *class,
                    struct class_attribute *attr,
                        const char *buf,
                            size_t count )
{
	struct gp2ap_data  *data ;
	char			   *reg_p ;
	char			   *value_p ;
	u32					reg ;
	u32					value ;
	char				tmpbuf[8] ;
	char			   *endp ;
	int err = 0 ;

	pr_debug( "gp2ap_reg_store data=%s\n", buf ) ;

	data = platform_get_drvdata( gp2ap_pdev ) ;

	if( strlen( buf ) > 7 )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		return -1 ;
	}

	mutex_lock( &data->mutex ) ;

	if(!data->ps_enabled && !data->als_enabled) {
		printk(KERN_ERR "[%s][%d]psals is disabled !! \n", __func__, __LINE__);
		mutex_unlock( &data->mutex ) ;
		return ERR_PSALS_STATE_DISABLED;
	}

	memset( tmpbuf, 0x00, sizeof( tmpbuf ) ) ;
	memcpy( tmpbuf, buf, strlen( buf ) ) ;

	reg_p = strtok( tmpbuf, "," ) ;
	value_p = strtok( NULL, NULL ) ;

	if( reg_p == NULL || value_p == NULL )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	reg = simple_strtoul( reg_p, &endp, 16 ) ;
	if( !( endp != reg_p &&
			( endp == ( reg_p + strlen( reg_p ) )
				|| ( endp == ( reg_p + strlen( reg_p ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}
	value = simple_strtoul( value_p, &endp, 16 ) ;
	if( !( endp != value_p &&
			( endp == ( value_p + strlen( value_p ) )
				|| ( endp == ( value_p + strlen( value_p ) - 1 ) && *endp == '\n' ) ) ) )
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
		return -1 ;
	}

	if(( reg <= 0x0b ) && ( value <= 255 ))
	{
		pr_debug( "  reg=%02x,value=%02x\n", reg, value ) ;
		data->regData[reg] = ( u8 )value ;
		err = gp2ap_i2c_write( ( u8 )reg, &( data->regData[reg] ), data->client, data ) ;
		if( err < 0 )
		{
			printk( KERN_ERR "[%s][%d]REG write err. !! \n", __func__, __LINE__ ) ;
			mutex_unlock( &data->mutex ) ;
			return -1 ;
		}
	}
	else
	{
		printk( KERN_ERR "[%s][%d]invalid value (%s)\n", __func__, __LINE__, buf ) ;
		mutex_unlock( &data->mutex ) ;
	    return -1 ;
	}

	mutex_unlock( &data->mutex ) ;

	return count ;
}

static CLASS_ATTR( reg_address, S_IRUGO|S_IWUSR|S_IWGRP, gp2ap_reg_address_show, gp2ap_reg_address_store );
static CLASS_ATTR( reg_count, S_IRUGO|S_IWUSR|S_IWGRP, NULL, gp2ap_reg_count_store );
static CLASS_ATTR( reg, S_IRUGO|S_IWUSR|S_IWGRP, gp2ap_reg_show, gp2ap_reg_store ) ;

static int gp2ap_probe( struct platform_device *pdev )
{
	struct gp2ap_data		   *gp2ap ;
#ifdef GP2AP030_REGUSE
	struct device *dev = &pdev->dev;
#endif
	u8							value = 0 ;
	int							err = 0 ;
	int							ret = 0 ;
	int							i ;

	pr_debug( "gp2ap_probe start !! \n" ) ;

	gp2ap = kzalloc( sizeof( struct gp2ap_data ), GFP_KERNEL ) ;
	if( !gp2ap )
	{
		printk( KERN_ERR "[%s][%d]kzalloc error !! \n", __func__, __LINE__ ) ;
		return -ENOMEM ;
	}

	mutex_init( &gp2ap->mutex ) ;

	gp2ap->dev_class = class_create( THIS_MODULE, CLASS_NAME ) ;
	if( IS_ERR( gp2ap->dev_class ) )
	{
		printk( KERN_ERR "[%s][%d]could not create dev_class\n", __func__, __LINE__ ) ;
		goto err_dev_class_create ;
	}

	if( class_create_file( gp2ap->dev_class, &class_attr_reg_address ) < 0 )
	{
		printk( KERN_ERR "[%s][%d]could not create class file(%s)!\n", __func__, __LINE__, class_attr_reg.attr.name ) ;
		goto err_class_create_reg_addr_file ;
	}
	if( class_create_file( gp2ap->dev_class, &class_attr_reg_count ) < 0 )
	{
		printk( KERN_ERR "[%s][%d]could not create class file(%s)!\n", __func__, __LINE__, class_attr_reg.attr.name ) ;
		goto err_class_create_reg_count_file ;
	}
	if( class_create_file( gp2ap->dev_class, &class_attr_reg ) < 0 )
	{
		printk( KERN_ERR "[%s][%d]could not create class file(%s)!\n", __func__, __LINE__, class_attr_reg.attr.name ) ;
		goto err_class_create_reg_file ;
	}

	gp2ap->als_enabled = 0 ;
	gp2ap->als_delay = SENSOR_DEFAULT_DELAY ;
	gp2ap->als_delay_gap = 0 ;
	gp2ap->als_range_prev = 0 ;
	gp2ap->als_enabled_prev = 0 ;
	memset( &gp2ap->als_tv, 0, sizeof( struct timeval )) ;
	INIT_DELAYED_WORK( &gp2ap->als_work, als_data_polling ) ;
#ifdef GP2AP030_REGUSE
	gp2ap_vdda = regulator_get(dev, "pm8950_l10");
	if(IS_ERR(gp2ap_vdda)) {
		printk(KERN_ERR "[%s][%d]gp2ap_vdda is NULL\n", __func__, __LINE__);
		goto err_regulator_control ;
	}
	err = regulator_enable(gp2ap_vdda);
	if (err != 0) {
		printk(KERN_ERR "[%s][%d]regulator_enable failure. (%d)\n", __func__, __LINE__, err);
		goto err_regulator_enable ;
	}
	usleep( SENSOR_REGL_ON_DELAY ) ;
#endif

	err = als_input_init( gp2ap ) ;
	if( err < 0 )
	{
		goto err_als_input_device ;
	}

	err = sysfs_create_group( &gp2ap->als_input_dev->dev.kobj, &als_attribute_group ) ;
	if( err )
	{
		printk( KERN_ERR
					"[%s][%d]sysfs_create_group failed[%s]\n", __func__, __LINE__,
						gp2ap->als_input_dev->name ) ;
		goto err_sysfs_create_group_als ;
	}

	gp2ap->ps_enabled = 0 ;
	gp2ap->ps_distance = 7 ;
	gp2ap->ps_enabled_prev = 0 ;

	INIT_WORK( &gp2ap->ps_int_work, ps_work_int_func ) ;

#if defined(PSALS_RECOVERY_ENABLE)
	INIT_DELAYED_WORK(&gp2ap->ps_polling_work, ps_recovery_polling);
	gp2ap->work_queue_recovery = create_singlethread_workqueue("gp2ap030_psals_recovery");
	if(gp2ap->work_queue_recovery) {
		INIT_WORK(&gp2ap->psals_recovery_work, psals_recovery_func);
	}
	wake_lock_init(&gp2ap_recovery_wake_lock, WAKE_LOCK_SUSPEND, "psals_recovery_wake_lock");
#endif /* PSALS_RECOVERY_ENABLE */

	err = ps_input_init( gp2ap ) ;
	if( err < 0 )
	{
		goto err_ps_input_device ;
	}

	err = sysfs_create_group( &gp2ap->ps_input_dev->dev.kobj, &ps_attribute_group ) ;
	if( err )
	{
		printk( KERN_ERR
					"[%s][%d]sysfs_create_group failed[%s]\n", __func__, __LINE__,
						gp2ap->ps_input_dev->name ) ;
		goto err_sysfs_create_group_ps ;
	}

	spin_lock_init( &gp2ap->ps_lock ) ;
	wake_lock_init( &gp2ap_ps_timeout_wake_lock, WAKE_LOCK_SUSPEND, "ps_timeout_wake_lock" ) ;
	wake_lock_init( &gp2ap_ps_wake_lock, WAKE_LOCK_SUSPEND, "ps_wake_lock" ) ;

	platform_set_drvdata( pdev, gp2ap ) ;

	gp2ap_i2c_init( ) ;
	value = OP_SHUTDOWN ;
	err = gp2ap_i2c_write( REG_ADR_00, &value, gp2ap->client, gp2ap ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]threre is no such device. !! \n", __func__, __LINE__ ) ;
		goto err_no_device ;
	}

	for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
	{
		gp2ap->regData[i] = gp2ap_init_data[i] ;
	}

	err = request_irq( gp2ap->ps_irq, ps_irq_handler,
									IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "proximity_int", gp2ap ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "[%s][%d]failed to request proximity_irq\n", __func__, __LINE__ ) ;
		goto err_ps_request_irq ;
	}
	gp2ap->ps_irq_enabled = 1 ;
	ps_disable_irq( gp2ap ) ;

	device_init_wakeup( &pdev->dev, 1 ) ;
#ifdef GP2AP030_REGUSE
	err = regulator_disable(gp2ap_vdda);
	if (err)
	{
		printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, err);
	}
	for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
	{
		gp2ap->regData[i] = gp2ap_init_data[i] ;
	}
#endif
	
    bootmode = sh_boot_get_bootmode() ;

	return 0 ;

err_ps_request_irq:
err_no_device:
#if defined(PSALS_RECOVERY_ENABLE)
	if(gp2ap->work_queue_recovery) {
		destroy_workqueue(gp2ap->work_queue_recovery);
		gp2ap->work_queue_recovery = NULL;
	}
	wake_lock_destroy(&gp2ap_recovery_wake_lock);
#endif /* PSALS_RECOVERY_ENABLE */
	i2c_del_driver( &gp2ap_i2c_driver ) ;
	sysfs_remove_group( &gp2ap->ps_input_dev->dev.kobj,
							&ps_attribute_group ) ;
	wake_lock_destroy( &gp2ap_ps_timeout_wake_lock ) ;
	wake_lock_destroy( &gp2ap_ps_wake_lock ) ;
err_sysfs_create_group_ps:
	input_unregister_device( gp2ap->ps_input_dev ) ;
	input_free_device( gp2ap->ps_input_dev ) ;
err_ps_input_device:
	sysfs_remove_group( &gp2ap->als_input_dev->dev.kobj,
							&als_attribute_group ) ;
err_sysfs_create_group_als:
	input_unregister_device( gp2ap->als_input_dev ) ;
	input_free_device( gp2ap->als_input_dev ) ;
err_als_input_device:
#ifdef GP2AP030_REGUSE
	ret = regulator_disable(gp2ap_vdda);
	if (ret) 
	{
		printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, ret);
	}
	for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
	{
		gp2ap->regData[i] = gp2ap_init_data[i] ;
	}
err_regulator_enable:
	regulator_put( gp2ap_vdda ) ;
	gp2ap_vdda = NULL ;
#endif
err_regulator_control:
	class_remove_file( gp2ap->dev_class, &class_attr_reg ) ;
err_class_create_reg_file:
	class_remove_file( gp2ap->dev_class, &class_attr_reg_count ) ;
err_class_create_reg_count_file:
	class_remove_file( gp2ap->dev_class, &class_attr_reg_address ) ;
err_class_create_reg_addr_file:
	class_destroy( gp2ap->dev_class ) ;
err_dev_class_create:
	mutex_destroy( &gp2ap->mutex ) ;
	kfree( gp2ap ) ;
	return err ;
}

static int
gp2ap_remove( struct platform_device *pdev )
{
	struct gp2ap_data  *data;
	u8 value ;
	int err = 0 ;
	int i = 0;

	pr_debug( "gp2ap_remove\n" ) ;

	if ( pdev )
	{
		data = platform_get_drvdata( pdev ) ;
		if( data )
		{
			if( (data->als_enabled) || (data->ps_enabled) )
			{
				value = OP_SHUTDOWN ;	// shutdown
				err = gp2ap_i2c_write( REG_ADR_00, &value, data->client, data ) ;
				if( err < 0 )
				{
					printk( KERN_ERR "[%s][%d]REG_ADR_00 write err. !! \n", __func__, __LINE__ ) ;
				}
			}
			if( data->als_enabled )
			{
				cancel_delayed_work_sync( &data->als_work ) ;
			}
			if( data->als_input_dev != NULL )
			{
				sysfs_remove_group( &data->als_input_dev->dev.kobj, &als_attribute_group ) ;
				input_unregister_device( data->als_input_dev ) ;
				input_free_device( data->als_input_dev ) ;
			}
#if defined(PSALS_RECOVERY_ENABLE)
			if(PSALS_RECOVERY) {
				if(data->ps_enabled) {
					cancel_delayed_work_sync(&data->ps_polling_work);
				}
			}
#endif /* PSALS_RECOVERY_ENABLE */
			if( data->ps_input_dev != NULL )
			{
				free_irq( data->ps_irq, data ) ;
				sysfs_remove_group( &data->ps_input_dev->dev.kobj, &ps_attribute_group ) ;
				wake_unlock( &gp2ap_ps_timeout_wake_lock ) ;
				wake_lock_destroy( &gp2ap_ps_timeout_wake_lock ) ;
				wake_unlock( &gp2ap_ps_wake_lock ) ;
				wake_lock_destroy( &gp2ap_ps_wake_lock ) ;
#if defined(PSALS_RECOVERY_ENABLE)
				if(PSALS_RECOVERY) {
					if(data->work_queue_recovery) {
						flush_workqueue(data->work_queue_recovery);
						destroy_workqueue(data->work_queue_recovery);
						data->work_queue_recovery = NULL;
					}
				}
				wake_unlock(&gp2ap_recovery_wake_lock);
				wake_lock_destroy(&gp2ap_recovery_wake_lock);
#endif /* PSALS_RECOVERY_ENABLE */
				input_unregister_device( data->ps_input_dev ) ;
				input_free_device( data->ps_input_dev ) ;
			}
			if( data->dev_class != NULL )
			{
				class_remove_file( data->dev_class, &class_attr_reg ) ;
				class_destroy( data->dev_class ) ;
			}
			i2c_del_driver( &gp2ap_i2c_driver ) ;
			device_init_wakeup( &pdev->dev, 0 ) ;
#ifdef GP2AP030_REGUSE
			if(gp2ap_vdda != NULL ){
				pr_debug("[%s][%d]regulator_disable start.\n", __func__, __LINE__ ) ;
				err = regulator_disable(gp2ap_vdda);
				if (err) 
				{
					printk(KERN_ERR "[%s][%d]regulator_disable failure. (%d)\n", __func__, __LINE__, err) ;
				}
				for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
				{
					data->regData[i] = gp2ap_init_data[i] ;
				}
				regulator_put( gp2ap_vdda ) ;
				gp2ap_vdda = NULL ;
			}
#endif
			mutex_destroy( &data->mutex ) ;
			kfree( data ) ;
		}
	}

	return 0 ;
}

static int
gp2ap_suspend( struct platform_device *pdev, pm_message_t state )
{
	struct gp2ap_data  *data;

	pr_debug( "gp2ap_suspend \n" ) ;

	if( pdev )
	{
		data = platform_get_drvdata( pdev ) ;
		if( data )
		{
			if( data->als_enabled )
			{
				cancel_delayed_work_sync( &data->als_work ) ;
			}
			if( data->ps_enabled )
			{
#if defined(PSALS_RECOVERY_ENABLE)
				if(PSALS_RECOVERY) {
					cancel_delayed_work_sync(&data->ps_polling_work);
				}
#endif /* PSALS_RECOVERY_ENABLE */
				ps_disable_irq(data);
				enable_irq_wake(data->ps_irq);
			}
		}
	}
	
	return 0 ;
}

static int
gp2ap_resume( struct platform_device *pdev )
{
	struct gp2ap_data  *data;
#if defined(PSALS_RECOVERY_ENABLE)
	u8 rdata = 0;
	int err  = 0;
#endif /* PSALS_RECOVERY_ENABLE */

	pr_debug( "gp2ap_resume \n" ) ;
	
	if( pdev ) 
	{
		data = platform_get_drvdata( pdev ) ;
		if ( data )
		{
#if defined(PSALS_RECOVERY_ENABLE)
			if(PSALS_RECOVERY) {
				if(data->ps_enabled) {
					mutex_lock(&data->mutex);
					err = gp2ap_i2c_read(REG_ADR_00, &rdata, sizeof(rdata), data->client);
					if(err < 0 || ((err >= 0) && ((rdata & OP_RUN) == OP_SHUTDOWN))) {
						if(err < 0) {
							printk(KERN_ERR "[%s][%d]resume i2c error.\n", __func__, __LINE__);
						}
						if((err >= 0) && ((rdata & OP_RUN) == OP_SHUTDOWN)) {
							printk(KERN_ERR "[%s][%d]resume REG_ADR_00 shutdown bit is 0.\n", __func__, __LINE__);
						}
						psals_recovery_flg = 1;
						wake_lock(&gp2ap_recovery_wake_lock);
						queue_work(data->work_queue_recovery, &data->psals_recovery_work);
						mutex_unlock(&data->mutex);
						return 0;
					}
					mutex_unlock(&data->mutex);
					if((bootmode != SH_BOOT_D) && (bootmode != SH_BOOT_F_F)) {
						schedule_delayed_work(&data->ps_polling_work, 0);
					}
					ps_enable_irq(data);
					disable_irq_wake(data->ps_irq);
				}
				if(data->als_enabled) {
					if((bootmode != SH_BOOT_D) && (bootmode != SH_BOOT_F_F)) {
						schedule_delayed_work(&data->als_work, 0);
					}
				}
			}
			else {
				if(data->als_enabled) {
					if((bootmode != SH_BOOT_D) && (bootmode != SH_BOOT_F_F)) {
						schedule_delayed_work(&data->als_work, 0);
					}
				}
				if(data->ps_enabled) {
					ps_enable_irq(data);
					disable_irq_wake(data->ps_irq);
				}
			}
#else
			if( data->als_enabled )
			{
				if (( bootmode != SH_BOOT_D ) && ( bootmode != SH_BOOT_F_F ))
				{
					schedule_delayed_work( &data->als_work, 0 ) ;
				}
			}
			if( data->ps_enabled )
			{
				ps_enable_irq(data);
				disable_irq_wake(data->ps_irq);
			}
#endif /* PSALS_RECOVERY_ENABLE */
		}
	}
	
	return 0 ;
}

static struct platform_driver gp2ap_driver = {
	.probe      = gp2ap_probe,
	.remove     = gp2ap_remove,
	.suspend    = gp2ap_suspend,
	.resume     = gp2ap_resume,
	.driver = {
		.name   = DEVICE_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = gp2ap_match_table,
	},
} ;

static int __init gp2ap_module_init( void )
{
	int ret;

	pr_debug( "gp2ap_module_init \n" ) ;
	gp2ap_pdev = platform_device_register_simple( DEVICE_NAME, 0, NULL, 0 ) ;
	if( IS_ERR( gp2ap_pdev ) )
	{
		printk( KERN_ERR "[%s][%d]platform_device_register_simple error!! \n", __func__, __LINE__ ) ;
		return -1 ;
	}
	ret = platform_driver_register( &gp2ap_driver ) ;
	return ret;
}

static void __exit gp2ap_module_exit( void )
{
	pr_debug( "gp2ap_module_exit \n" ) ;

	platform_driver_unregister( &gp2ap_driver ) ;
	platform_device_unregister( gp2ap_pdev ) ;
}

module_init( gp2ap_module_init ) ;
module_exit( gp2ap_module_exit ) ;

MODULE_AUTHOR( "SHARP" ) ;
MODULE_DESCRIPTION( "Optical Sensor driver for GP2AP030*00F" ) ;
MODULE_LICENSE( "GPL" ) ;
