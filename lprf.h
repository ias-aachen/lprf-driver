/*
 * IAS LPRF driver
 *
 * Copyright (C) 2015 IAS RWTH Aachen
 * Adapted from  AT86RF230 driver, Copyright (C) 2009-2012 Siemens AG
 * Also based on scull driver from "Linux Device Drivers"
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details
 *
 * Written by:
 * Moritz Schrey <mschrey@ias.rwth-aachen.de>
 * Dmitry Eremin-Solenikov <dbaryshkov@gmail.com>
 * Alexander Smirnov <alex.bluesman.smirnov@gmail.com>
 * Alexander Aring <aar@pengutronix.de>
 * Alessandro Rubini
 * Jonathan Corbet
 */
//<scull>
#ifndef _SCULL_H_
#define _SCULL_H_

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */

/*
 * Macros to help debugging
 */
#undef PDEBUG             /* undef it, just in case */
#ifdef SCULL_DEBUG
#  ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "scull: " fmt, ## args)
#  else
     /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */

#ifndef SCULL_MAJOR
#define SCULL_MAJOR 0   /* dynamic major by default */
#endif

#ifndef SCULL_NR_DEVS
#define SCULL_NR_DEVS 1    /* lprf0 (was scull0 through scull3) */
#endif

/*
 * The bare device is a variable-length region of memory.
 * Use a linked list of indirect blocks.
 *
 * "scull_dev->data" points to an array of pointers, each
 * pointer refers to a memory area of SCULL_QUANTUM bytes.
 *
 * The array (quantum-set) is SCULL_QSET long.
 */
#ifndef SCULL_QUANTUM
#define SCULL_QUANTUM 1
#endif

#ifndef SCULL_QSET
#define SCULL_QSET    1
#endif

/*
 * Representation of scull quantum sets.
 */
struct scull_qset {
	void **data;
	struct scull_qset *next;
};

struct scull_dev {
	struct scull_qset *data;  /* Pointer to first quantum set */
	int quantum;              /* the current quantum size */
	int qset;                 /* the current array size */
	unsigned long size;       /* amount of data stored here */
	unsigned int access_key;  /* used by sculluid and scullpriv */
	struct mutex scull_mutex; /* mutual exclusion mutex */
	struct cdev cdev;	      /* Char device structure		*/
};

/*
 * Split minors in two parts
 */
#define TYPE(minor)	(((minor) >> 4) & 0xf)	/* high nibble */
#define NUM(minor)	((minor) & 0xf)		/* low  nibble */


/*
 * The different configurable parameters
 */
extern int scull_major;     /* main.c */
extern int scull_nr_devs;
extern int scull_quantum;
extern int scull_qset;


/*
 * Prototypes for shared functions
 */

int     scull_p_init(dev_t dev);
void    scull_p_cleanup(void);
int     scull_access_init(dev_t dev);
void    scull_access_cleanup(void);
int     scull_trim(struct scull_dev *dev);
ssize_t scull_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t scull_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
long    scull_ioctl(struct file *filp, unsigned int cmd, unsigned long arg); 
int		scull_init_module(void);
void	scull_cleanup_module(void);

/*
 * Ioctl definitions
 */

/* Use 'k' as magic number */
#define SCULL_IOC_MAGIC  'k'    /* Please use a different 8-bit number in your code */
#define SCULL_IOCRESET    _IO(SCULL_IOC_MAGIC, 0)

/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": switch G and S atomically
 * H means "sHift": switch T and Q atomically
 */
#define SCULL_IOCSQUANTUM _IOW(SCULL_IOC_MAGIC,  1, int)
#define SCULL_IOCSQSET    _IOW(SCULL_IOC_MAGIC,  2, int)
#define SCULL_IOCTQUANTUM _IO(SCULL_IOC_MAGIC,   3)
#define SCULL_IOCTQSET    _IO(SCULL_IOC_MAGIC,   4)
#define SCULL_IOCGQUANTUM _IOR(SCULL_IOC_MAGIC,  5, int)
#define SCULL_IOCGQSET    _IOR(SCULL_IOC_MAGIC,  6, int)
#define SCULL_IOCQQUANTUM _IO(SCULL_IOC_MAGIC,   7)
#define SCULL_IOCQQSET    _IO(SCULL_IOC_MAGIC,   8)
#define SCULL_IOCXQUANTUM _IOWR(SCULL_IOC_MAGIC, 9, int)
#define SCULL_IOCXQSET    _IOWR(SCULL_IOC_MAGIC,10, int)
#define SCULL_IOCHQUANTUM _IO(SCULL_IOC_MAGIC,  11)
#define SCULL_IOCHQSET    _IO(SCULL_IOC_MAGIC,  12)

/*
 * The other entities only have "Tell" and "Query", because they're
 * not printed in the book, and there's no need to have all six.
 * (The previous stuff was only there to show different ways to do it.
 */
#define SCULL_P_IOCTSIZE _IO(SCULL_IOC_MAGIC,   13)
#define SCULL_P_IOCQSIZE _IO(SCULL_IOC_MAGIC,   14)
/* ... more to come */

#define SCULL_IOC_MAXNR 14

#endif /* _SCULL_H_ */

struct scull_dev *scull_devices;	/* allocated in scull_init_module */
struct file_operations scull_fops;
//</scull>

//==================================================================================================
//==================================================================================================
//==================================================================================================

// ***************************
// **  struct declarations  **
// ***************************

#ifndef LPRF_H
#define LPRF_H

struct lprf_platform_data {
	int rstn;
	int slp_tr;
	int dig2;
};

#endif


struct lprf_local;


/* lprf chip depend data.
 * All timings are in us.
 */
struct lprf_chip_data {
	u16 t_from_sleep_to_tx;
	u16 t_from_tx_idle_to_tx;
	u16 t_from_rx_power_to_tx;
	u16 t_from_rx_pll_to_tx;
	
	u16 t_from_sleep_to_rx;
	u16 t_from_tx_to_rx;
	u16 t_from_rxhold_to_rx;

	u16 t_from_sleep_to_txidle;
	u16 t_from_tx_to_txidle;
	u16 t_from_rx_to_txidle; 
	u16 t_from_rx_pll_to_txidle;

	u16 t_from_sleep_to_rxhold;
	u16 t_from_tx_to_rxhold;
	u16 t_from_rx_to_rxhold;

	
	int rssi_base_val;

	int (*set_channel)(struct lprf_local *, u8, u8);
	int (*get_desense_steps)(struct lprf_local *, s32);
};

static const struct ieee802154_ops lprf_ops;
static const struct regmap_config lprf_regmap_spi_config;

#define LPRF_MAX_BUF (256 + 2)

struct lprf_state_change {
	struct lprf_local *lp;

	struct hrtimer timer;
	struct spi_message msg;
	struct spi_transfer trx;
	u8 buf[LPRF_MAX_BUF];

	void (*complete)(void *context);
	u8 from_state;
	u8 to_state;
};



struct lprf_local {
	struct spi_device *spi;

	struct ieee802154_hw *hw;	
	struct lprf_chip_data *data;
	struct regmap *regmap;
	int slp_tr;

	struct completion state_complete;
	struct lprf_state_change state;

	bool tx_aret;
	unsigned long cal_timeout;
	s8 max_frame_retries;
	bool is_tx;
	bool is_tx_from_off;

	u8 tx_retry;
	struct sk_buff *tx_skb;
	struct lprf_state_change tx;

	struct lprf_state_change rx;
	struct lprf_state_change debug;
};



//TODO This really should not be global...
struct lprf_local *lp;


// ***************************
// **   define constants    **
// ***************************

/*
#define	RG_TRX_STATUS	(0x01)
#define	SR_TRX_STATUS		0x01, 0x1f, 0
#define	SR_RESERVED_01_3	0x01, 0x20, 5
#define	SR_CCA_STATUS		0x01, 0x40, 6
#define	SR_CCA_DONE		0x01, 0x80, 7
#define	RG_TRX_STATE	(0x02)
#define	SR_TRX_CMD		0x02, 0x1f, 0
#define	SR_TRAC_STATUS		0x02, 0xe0, 5
#define	RG_TRX_CTRL_0	(0x03)
#define	SR_CLKM_CTRL		0x03, 0x07, 0
#define	SR_CLKM_SHA_SEL		0x03, 0x08, 3
#define	SR_PAD_IO_CLKM		0x03, 0x30, 4
#define	SR_PAD_IO		0x03, 0xc0, 6
#define	RG_TRX_CTRL_1	(0x04)
#define	SR_IRQ_POLARITY		0x04, 0x01, 0
#define	SR_IRQ_MASK_MODE	0x04, 0x02, 1
#define	SR_SPI_CMD_MODE		0x04, 0x0c, 2
#define	SR_RX_BL_CTRL		0x04, 0x10, 4
#define	SR_TX_AUTO_CRC_ON	0x04, 0x20, 5
#define	SR_IRQ_2_EXT_EN		0x04, 0x40, 6
#define	SR_PA_EXT_EN		0x04, 0x80, 7
#define	RG_PHY_TX_PWR	(0x05)
#define	SR_TX_PWR		0x05, 0x0f, 0
#define	SR_PA_LT		0x05, 0x30, 4
#define	SR_PA_BUF_LT		0x05, 0xc0, 6
#define	RG_PHY_RSSI	(0x06)
#define	SR_RSSI			0x06, 0x1f, 0
#define	SR_RND_VALUE		0x06, 0x60, 5
#define	SR_RX_CRC_VALID		0x06, 0x80, 7
#define	RG_PHY_ED_LEVEL	(0x07)
#define	SR_ED_LEVEL		0x07, 0xff, 0
#define	RG_PHY_CC_CCA	(0x08)
#define	SR_CHANNEL		0x08, 0x1f, 0
#define	SR_CCA_MODE		0x08, 0x60, 5
#define	SR_CCA_REQUEST		0x08, 0x80, 7
#define	RG_CCA_THRES	(0x09)
#define	SR_CCA_ED_THRES		0x09, 0x0f, 0
#define	SR_RESERVED_09_1	0x09, 0xf0, 4
#define	RG_RX_CTRL	(0x0a)
#define	SR_PDT_THRES		0x0a, 0x0f, 0
#define	SR_RESERVED_0a_1	0x0a, 0xf0, 4
#define	RG_SFD_VALUE	(0x0b)
#define	SR_SFD_VALUE		0x0b, 0xff, 0
#define	RG_TRX_CTRL_2	(0x0c)
#define	SR_OQPSK_DATA_RATE	0x0c, 0x03, 0
#define	SR_SUB_MODE		0x0c, 0x04, 2
#define	SR_BPSK_QPSK		0x0c, 0x08, 3
#define	SR_OQPSK_SUB1_RC_EN	0x0c, 0x10, 4
#define	SR_RESERVED_0c_5	0x0c, 0x60, 5
#define	SR_RX_SAFE_MODE		0x0c, 0x80, 7
#define	RG_ANT_DIV	(0x0d)
#define	SR_ANT_CTRL		0x0d, 0x03, 0
#define	SR_ANT_EXT_SW_EN	0x0d, 0x04, 2
#define	SR_ANT_DIV_EN		0x0d, 0x08, 3
#define	SR_RESERVED_0d_2	0x0d, 0x70, 4
#define	SR_ANT_SEL		0x0d, 0x80, 7
#define	RG_IRQ_MASK	(0x0e)
#define	SR_IRQ_MASK		0x0e, 0xff, 0
#define	RG_IRQ_STATUS	(0x0f)
#define	SR_IRQ_0_PLL_LOCK	0x0f, 0x01, 0
#define	SR_IRQ_1_PLL_UNLOCK	0x0f, 0x02, 1
#define	SR_IRQ_2_RX_START	0x0f, 0x04, 2
#define	SR_IRQ_3_TRX_END	0x0f, 0x08, 3
#define	SR_IRQ_4_CCA_ED_DONE	0x0f, 0x10, 4
#define	SR_IRQ_5_AMI		0x0f, 0x20, 5
#define	SR_IRQ_6_TRX_UR		0x0f, 0x40, 6
#define	SR_IRQ_7_BAT_LOW	0x0f, 0x80, 7
#define	RG_VREG_CTRL	(0x10)
#define	SR_RESERVED_10_6	0x10, 0x03, 0
#define	SR_DVDD_OK		0x10, 0x04, 2
#define	SR_DVREG_EXT		0x10, 0x08, 3
#define	SR_RESERVED_10_3	0x10, 0x30, 4
#define	SR_AVDD_OK		0x10, 0x40, 6
#define	SR_AVREG_EXT		0x10, 0x80, 7
#define	RG_BATMON	(0x11)
#define	SR_BATMON_VTH		0x11, 0x0f, 0
#define	SR_BATMON_HR		0x11, 0x10, 4
#define	SR_BATMON_OK		0x11, 0x20, 5
#define	SR_RESERVED_11_1	0x11, 0xc0, 6
#define	RG_XOSC_CTRL	(0x12)
#define	SR_XTAL_TRIM		0x12, 0x0f, 0
#define	SR_XTAL_MODE		0x12, 0xf0, 4
#define MYADDRESS2	(0x15)
#define	RG_RX_SYN	(0x15)
#define	SR_RX_PDT_LEVEL		0x15, 0x0f, 0
#define	SR_RESERVED_15_2	0x15, 0x70, 4
#define	SR_RX_PDT_DIS		0x15, 0x80, 7
#define	RG_XAH_CTRL_1	(0x17)
#define	SR_RESERVED_17_8	0x17, 0x01, 0
#define	SR_AACK_PROM_MODE	0x17, 0x02, 1
#define	SR_AACK_ACK_TIME	0x17, 0x04, 2
#define	SR_RESERVED_17_5	0x17, 0x08, 3
#define	SR_AACK_UPLD_RES_FT	0x17, 0x10, 4
#define	SR_AACK_FLTR_RES_FT	0x17, 0x20, 5
#define	SR_CSMA_LBT_MODE	0x17, 0x40, 6
#define	SR_RESERVED_17_1	0x17, 0x80, 7
#define	RG_FTN_CTRL	(0x18)
#define	SR_RESERVED_18_2	0x18, 0x7f, 0
#define	SR_FTN_START		0x18, 0x80, 7
#define	RG_PLL_CF	(0x1a)
#define	SR_RESERVED_1a_2	0x1a, 0x7f, 0
#define	SR_PLL_CF_START		0x1a, 0x80, 7
#define	RG_PLL_DCU	(0x1b)
#define	SR_RESERVED_1b_3	0x1b, 0x3f, 0
#define	SR_RESERVED_1b_2	0x1b, 0x40, 6
#define	SR_PLL_DCU_START	0x1b, 0x80, 7
#define	RG_PART_NUM	(0x1c)
#define	SR_PART_NUM		0x1c, 0xff, 0
#define	RG_VERSION_NUM	(0x1d)
#define	SR_VERSION_NUM		0x1d, 0xff, 0
#define	RG_MAN_ID_0	(0x1e)
#define	SR_MAN_ID_0		0x1e, 0xff, 0
#define	RG_MAN_ID_1	(0x1f)
#define	SR_MAN_ID_1		0x1f, 0xff, 0
#define	RG_SHORT_ADDR_0	(0x20)
#define	SR_SHORT_ADDR_0		0x20, 0xff, 0
#define	RG_SHORT_ADDR_1	(0x21)
#define	SR_SHORT_ADDR_1		0x21, 0xff, 0
#define	RG_PAN_ID_0	(0x22)
#define	SR_PAN_ID_0		0x22, 0xff, 0
#define	RG_PAN_ID_1	(0x23)
#define	SR_PAN_ID_1		0x23, 0xff, 0
#define	RG_IEEE_ADDR_0	(0x24)
#define	SR_IEEE_ADDR_0		0x24, 0xff, 0
#define	RG_IEEE_ADDR_1	(0x25)
#define	SR_IEEE_ADDR_1		0x25, 0xff, 0
#define	RG_IEEE_ADDR_2	(0x26)
#define	SR_IEEE_ADDR_2		0x26, 0xff, 0
#define	RG_IEEE_ADDR_3	(0x27)
#define	SR_IEEE_ADDR_3		0x27, 0xff, 0
#define	RG_IEEE_ADDR_4	(0x28)
#define	SR_IEEE_ADDR_4		0x28, 0xff, 0
#define	RG_IEEE_ADDR_5	(0x29)
#define	SR_IEEE_ADDR_5		0x29, 0xff, 0
#define	RG_IEEE_ADDR_6	(0x2a)
#define	SR_IEEE_ADDR_6		0x2a, 0xff, 0
#define	RG_IEEE_ADDR_7	(0x2b)
#define	SR_IEEE_ADDR_7		0x2b, 0xff, 0
#define	RG_XAH_CTRL_0	(0x2c)
#define	SR_SLOTTED_OPERATION	0x2c, 0x01, 0
#define	SR_MAX_CSMA_RETRIES	0x2c, 0x0e, 1
#define	SR_MAX_FRAME_RETRIES	0x2c, 0xf0, 4
#define	RG_CSMA_SEED_0	(0x2d)
#define	SR_CSMA_SEED_0		0x2d, 0xff, 0
#define	RG_CSMA_SEED_1	(0x2e)
#define	SR_CSMA_SEED_1		0x2e, 0x07, 0
#define	SR_AACK_I_AM_COORD	0x2e, 0x08, 3
#define	SR_AACK_DIS_ACK		0x2e, 0x10, 4
#define	SR_AACK_SET_PD		0x2e, 0x20, 5
#define	SR_AACK_FVN_MODE	0x2e, 0xc0, 6
#define	RG_CSMA_BE	(0x2f)
#define	SR_MIN_BE		0x2f, 0x0f, 0
#define	SR_MAX_BE		0x2f, 0xf0, 4

#define CMD_REG		0x01
#define CMD_REG_MASK	0x3f
#define CMD_WRITE	0x02
#define CMD_FB		0x20

#define IRQ_BAT_LOW	(1 << 7)
#define IRQ_TRX_UR	(1 << 6)
#define IRQ_AMI		(1 << 5)
#define IRQ_CCA_ED	(1 << 4)
#define IRQ_TRX_END	(1 << 3)
#define IRQ_RX_START	(1 << 2)
#define IRQ_PLL_UNL	(1 << 1)
#define IRQ_PLL_LOCK	(1 << 0)

#define IRQ_ACTIVE_HIGH	0
#define IRQ_ACTIVE_LOW	1

#define STATE_P_ON		0x00	// BUSY 
#define STATE_BUSY_RX		0x01
#define STATE_BUSY_TX		0x02
#define STATE_FORCE_TRX_OFF	0x03
#define STATE_FORCE_TX_ON	0x04	// IDLE 
// 0x05 				// INVALID_PARAMETER 
#define STATE_RX_ON		0x06
// 0x07 				// SUCCESS 
#define STATE_TRX_OFF		0x08
#define STATE_TX_ON		0x09
// 0x0a - 0x0e 			// 0x0a - UNSUPPORTED_ATTRIBUTE 
#define STATE_SLEEP		0x0F
#define STATE_PREP_DEEP_SLEEP	0x10
#define STATE_BUSY_RX_AACK	0x11
#define STATE_BUSY_TX_ARET	0x12
#define STATE_RX_AACK_ON	0x16
#define STATE_TX_ARET_ON	0x19
#define STATE_RX_ON_NOCLK	0x1C
#define STATE_RX_AACK_ON_NOCLK	0x1D
#define STATE_BUSY_RX_AACK_NOCLK 0x1E
#define STATE_TRANSITION_IN_PROGRESS 0x1F
*/

#define LPRF_NUMREGS 0x1FFF
#define CELL_SIZE 1  // smallest entity to be read, here 1 byte 
#define FILE_SIZE 8192 

// ***************************
// ** function declarations **
// ***************************


static void lprf_async_state_change(struct lprf_local *lp, struct lprf_state_change *ctx, const u8 state, void (*complete)(void *context), const bool irq_enable);

static void lprf_check_for_data(struct lprf_local *lp);

