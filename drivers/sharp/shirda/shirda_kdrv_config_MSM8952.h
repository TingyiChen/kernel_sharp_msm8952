/*
 *	default(MSM8952) IrDA kernel driver hardware configurations header
 *
 *	Copyright(C) 2014 SHARP CORPORATION. All rights reserved.
 */
/*
 * when		who			what, where, why
 * ----------	----------	-----------------------------------------
 * 2014/12/09	Katsumi.A	Device Tree
 * 2014/12/22	SC Sugimoto	Customize from base of 2014/12/22 version
 * 2015/01/27	SC Sugimoto	Added defined of BLSP serial device name
 * 2015/01/28	SC Kawakami	Move Compile Switch 
 *				"SHIRDA_TTY_WITH_ENTITY_OF_TERMIOS"
 *				from shirda_ldisc.c
 */

/* Compile Switchs */
#define	SHIRDA_DRV_USE_DEVICE_TREE	/* use DT/pinctr		*/
#define	SHIRDA_PF_LOLLIPOP_OR_LATER
#undef	SHIRDA_USE_CLK_GET_SYS
#define SHIRDA_TTY_WITH_ENTITY_OF_TERMIOS

/* The latest environment may be not supported */
#undef	SHIRDA_TEST_GPIO_DEBUG

/* BLSP serial node for tty device name */
#define	SHIRDA_BLSP_NODE	"7aef000.serial"

/*
 * CPU Registers
 */
#define	GSBIREG_BASE		(0x7AEF000)	/* BLSP#5 base address */
#define	UART_DM_REGISTER	(0x0)		/* UART offset address */
#define	UART_DM_IRDA		(0x00B8)	/* offset address */
