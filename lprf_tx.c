/* this is a mixture from at86rf230.c and scull-main
 * all references to at86rf230 are changed to lprf
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
 * Works on kernel version 4.4.15+
 *
 * Written by:
 * Moritz Schrey <mschrey@ias.rwth-aachen.de>
 * Jan Richter-Brockmann <jan.richter-brockmann@rwth-aachen.de>
 * Dmitry Eremin-Solenikov <dbaryshkov@gmail.com>
 * Alexander Smirnov <alex.bluesman.smirnov@gmail.com>
 * Alexander Aring <aar@pengutronix.de>
 * Alessandro Rubini
 * Jonathan Corbet
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/spi/lprf.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/of_gpio.h>
#include <linux/ieee802154.h>

#include <net/mac802154.h>
#include <net/cfg802154.h>

//<scull>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>	/* copy_*_user */
//</scull>

#include "lprf.h"		/* local definitions */
#include "lprf_registers.h"


static inline int __lprf_write(struct lprf_local *lp, unsigned int addr, unsigned int data)
{
	unsigned int retval;
	printk(KERN_DEBUG "lprf: __lprf_write - start: addr=%02X, data=%02X\n", addr, data);
	retval = regmap_write(lp->regmap, addr, data);
	printk(KERN_DEBUG "lprf: __lprf_write - end.\n");	
	return retval;
}

static inline int __lprf_read(struct lprf_local *lp, unsigned int addr, unsigned int *data)
{
	printk(KERN_DEBUG "lprf: __lprf_read: addr=%u\tdata=%u\n", addr, *data);
	return regmap_read(lp->regmap, addr, data);
}

static inline int lprf_read_subreg(struct lprf_local *lp, unsigned int addr, unsigned int mask, unsigned int shift, unsigned int *data)
{
	int rc;

	printk(KERN_DEBUG "lprf: lprf_read_subreg: addr=%u\tdata=%u\tmask=%u\tshift=%u\n", addr, *data, mask, shift);

	rc = __lprf_read(lp, addr, data);
	if (rc > 0)
		*data = (*data & mask) >> shift;
	
	return rc;
}

static inline int lprf_write_subreg(struct lprf_local *lp, unsigned int addr, unsigned int mask, unsigned int shift, unsigned int data)
{
	printk(KERN_DEBUG "lprf: lprf_write_subreg: addr=%d\tdata=%d\tmask=%d\tshift=%d\n", addr, data, mask, shift);
	return regmap_update_bits(lp->regmap, addr, mask, data << shift);
}


static bool lprf_reg_writeable(struct device *dev, unsigned int reg)   //part of struct regmap_config lprf_regmap_spi_config
{
	if(((reg >= 0) && (reg < 53)) || ((reg >= 56) && (reg < 70)) || ((reg >= 80) && (reg < 176)) || ((reg >= 192) && (reg <= 243)))
		return true;
	else
		return false;
}

static bool lprf_reg_readable(struct device *dev, unsigned int reg)   //part of struct regmap_config lprf_regmap_spi_config
{
	bool rc;

	/* all writeable are also readable */
	rc = lprf_reg_writeable(dev, reg);
	if (rc)
		return rc;

	/* readonly regs */
	switch (reg) {
	case RG_PLL_TPM_GAIN_OUT_L:
	case RG_PLL_TPM_GAIN_OUT_M:
	case RG_PLL_TPM_GAIN_OUT_H:
	case RG_DEM_PD_OUT:
	case RG_DEM_GC_AOUT:
	case RG_DEM_GC_BOUT:
	case RG_DEM_GC_COUT:
	case RG_DEM_GC_DOUT:
	case RG_DEM_FREQ_OFFSET_OUT:
	case RG_SM_STATE:
	case RG_SM_FIFO:
	case RG_SM_GLOBAL:
	case RG_SM_POWER:
	case RG_SM_RX:
	case RG_SM_WAKEUP_EN:
	case RG_SM_DEM_ADC:
	case RG_SM_PLL_TX:
	case RG_SM_PLL_CHAN_INT:
	case RG_SM_PLL_CHAN_FRAC_H:
	case RG_SM_PLL_CHAN_FRAC_M:
	case RG_SM_PLL_CHAN_FRAC_L:
	case RG_SM_TX433:
	case RG_SM_TX800:
	case RG_SM_TX24:
		return true;
	default:
		return false;
	}
}

static bool lprf_reg_volatile(struct device *dev, unsigned int reg)      //part of struct regmap_config lprf_regmap_spi_config
{
	/* can be changed during runtime */
	// !modify!
	switch (reg) {
	case RG_SM_STATE:	// set the correct registers 
		return true;
	default:
		return false;
	}
}

static bool lprf_reg_precious(struct device *dev, unsigned int reg)       //part of struct regmap_config lprf_regmap_spi_config
{
	
	// !modify!
	switch (reg) {
	case RG_SM_STATE:	// set the correct registers 
		return true;
	default:
		return false;
	}
}

//function pointer to this passed as parameter to lprf_async_state_change (by lprf_async_error)
static void lprf_async_error_recover(void *context)
{
}

static inline void lprf_async_error(struct lprf_local *lp, struct lprf_state_change *ctx, int rc)
{
	dev_err(&lp->spi->dev, "spi_async error %d\n", rc);

	//lprf_async_state_change(lp, ctx, STATE_FORCE_TRX_OFF, lprf_async_error_recover, false);
}



/* Generic function to get some register value in sync mode, only called within char driver */
static void lprf_sync_read_reg_debugging(struct lprf_local *lp, const u8 reg, struct lprf_state_change *ctx,  const bool output_disable)
{
	int rc;

	u8 *tx_buf = ctx->buf;

	if(!output_disable){	
		printk(KERN_DEBUG "lprf: lprf_sync_read_reg_debugging - start: reg=%u  \n", reg);	
	}

	tx_buf[0] = 0x80; 	// read cmd
	tx_buf[1] = reg;
	ctx->trx.len = 3;		
	rc = spi_sync(lp->spi, &ctx->msg);
	if (rc) {
		lprf_async_error(lp, ctx, rc);
	}

	if(!output_disable){	 
		printk(KERN_DEBUG "lprf: lprf_sync_read_reg_debugging - end.\n");
	}	
}

/* Generic function to get some register value in async mode */
static void lprf_async_read_reg(struct lprf_local *lp, const u8 reg, struct lprf_state_change *ctx, void (*complete)(void *context), const bool output_disable)
{
	int rc;

	u8 *tx_buf = ctx->buf;

	if(!output_disable){	
		printk(KERN_DEBUG "lprf: lprf_async_read_reg - start: reg=%u  \n", reg);	
	}

	tx_buf[0] = 0x80; 	// read cmd 
	tx_buf[1] = reg;
	ctx->trx.len = 3;		
	ctx->msg.complete = complete;
	rc = spi_async(lp->spi, &ctx->msg);
	if (rc) {
		lprf_async_error(lp, ctx, rc);
	}

	if(!output_disable){	
		printk(KERN_DEBUG "lprf: lprf_async_read_reg - end.\n");
	}	
}

/* expects SM_STATE formatted values, returns SM_MAIN formatted values */
static int lprf_get_state(u8 value){
	printk(KERN_DEBUG "lprf: lprf_get_state. %s:%i\n", __FILE__, __LINE__);
	
	switch(value){
		case 0x00:
			return STATE_CMD_NONE;
		case STATE_RECEIVING:	
			return STATE_CMD_RX;
		case STATE_RX_RDY:
			return STATE_CMD_RXHOLD;
		case STATE_SENDING:
			return STATE_CMD_TX;
		case STATE_TX_RDY:
			return STATE_CMD_TXIDLE;
		case STATE_SLEEP:
			return STATE_CMD_DEEPSLEEP;
		case STATE_DEEPSLEEP:
			return STATE_CMD_SLEEP;
		default:
			return -1;			
	}
	
}

static void
lprf_async_state_assert(void *context)
{
	struct lprf_state_change *ctx = context;
	// struct lprf_local *lp = ctx->lp;
	const u8 *buf = ctx->buf;
	const u8 trx_state = buf[2];
	u8 act_state;

	printk(KERN_DEBUG "lprf: lprf_async_state_assert - start. %s:%i\n", __FILE__, __LINE__); 

	// modify: what can we do if the state change fail?
	if ((trx_state & STATE_BUSY)) { 
		printk(KERN_DEBUG "lprf: lprf_async_state_assert - state machine is busy: trx_state=%u. %s:%i\n", trx_state, __FILE__, __LINE__);
		return;
	}
	else{
		act_state = lprf_get_state(trx_state);
		if(act_state == ctx->to_state){
			printk(KERN_DEBUG "lprf: lprf_async_state_assert - state change was successful, trx_state=%u. %s:%i\n", trx_state, __FILE__, __LINE__);
		}		
		else{
			printk(KERN_DEBUG "lprf: lprf_async_state_assert - state change failed, trx_state=%u. %s:%i\n", trx_state, __FILE__, __LINE__);
			return;
		}	
	}		

	if (ctx->complete)
		ctx->complete(context);   //this actually calls lprf_async_state_change_complete()

	printk(KERN_DEBUG "lprf: lprf_async_state_assert - end. %s:%i\n", __FILE__, __LINE__);
}


static enum hrtimer_restart lprf_async_state_timer(struct hrtimer *timer)
{
	struct lprf_state_change *ctx = container_of(timer, struct lprf_state_change, timer);
	struct lprf_local *lp = ctx->lp;

	printk(KERN_DEBUG "lprf: lprf_async_state_timer - start. %s:%i\n", __FILE__, __LINE__);

	lprf_async_read_reg(lp, RG_SM_STATE, ctx, lprf_async_state_assert, 0);	

	printk(KERN_DEBUG "lprf: lprf_async_state_timer - end. %s:%i\n", __FILE__, __LINE__);

	return HRTIMER_NORESTART;
}


static void lprf_async_state_delay(void *context)
{
	struct lprf_state_change *ctx = context;
	struct lprf_local *lp = ctx->lp;
	struct lprf_chip_data *c = lp->data;
	ktime_t tim;

	printk(KERN_DEBUG "lprf: lprf_async_state_delay - start. ctx->from_state=%u ctx->to_state=%u. %s:%i\n",ctx->from_state, ctx->to_state, __FILE__, __LINE__);

	switch (ctx->from_state){
		case STATE_CMD_SLEEP:
		case STATE_CMD_NONE:		// first sending (from_state is 0)
		case STATE_CMD_DEEPSLEEP:
			switch (ctx->to_state){
				case STATE_CMD_TX:
					tim = ktime_set(0, c->t_from_sleep_to_tx * NSEC_PER_USEC);
					goto change;
				case STATE_CMD_RX:
					tim = ktime_set(0, c->t_from_sleep_to_rx * NSEC_PER_USEC);
					goto change;
				case STATE_CMD_TXIDLE:
					tim = ktime_set(0, c->t_from_sleep_to_txidle * NSEC_PER_USEC);
					goto change;
				case STATE_CMD_RXHOLD:
					tim = ktime_set(0, c->t_from_sleep_to_rxhold * NSEC_PER_USEC);
					goto change;
				default:
					break;
			}
		case STATE_CMD_TXIDLE:
			switch (ctx->to_state){
				case STATE_CMD_TX:
					tim = ktime_set(0, c->t_from_tx_idle_to_tx * NSEC_PER_USEC);
					goto change;
				case STATE_CMD_RXHOLD:
					tim = ktime_set(0, c->t_from_tx_to_rxhold * NSEC_PER_USEC);
					goto change;
				default:
					break;
			}
		case STATE_CMD_RX:
		case STATE_CMD_RXHOLD:	
			switch (ctx->to_state){
				case STATE_CMD_TX:
					if(T_PLL_SET_TIME < T_POWER_TX_TIME){
						tim = ktime_set(0, c->t_from_rx_power_to_tx * NSEC_PER_USEC);
					}
					else{
						tim = ktime_set(0, c->t_from_rx_pll_to_tx* NSEC_PER_USEC);
					}
					goto change;
				case STATE_CMD_RX:
					tim = ktime_set(0, c->t_from_rxhold_to_rx * NSEC_PER_USEC);
					goto change;
				case STATE_CMD_TXIDLE:
					if(T_PLL_SET_TIME < T_POWER_TX_TIME){
						tim = ktime_set(0, c->t_from_rx_to_txidle * NSEC_PER_USEC);
					}
					else{
						tim = ktime_set(0, c->t_from_rx_pll_to_txidle * NSEC_PER_USEC);
					}
					goto change;
				case STATE_CMD_RXHOLD:
					tim = ktime_set(0, c->t_from_rx_to_rxhold * NSEC_PER_USEC);
					goto change;	
				default:
					break;
			}
		case STATE_CMD_TX:
			switch (ctx->to_state){
				case STATE_CMD_RX:
					tim = ktime_set(0, c->t_from_tx_to_rx * NSEC_PER_USEC);
					goto change;
				case STATE_CMD_TXIDLE:
					tim = ktime_set(0, c->t_from_tx_to_txidle * NSEC_PER_USEC);
					goto change;
				case STATE_CMD_RXHOLD:
					tim = ktime_set(0, c->t_from_tx_to_rxhold * NSEC_PER_USEC);
					goto change;
				default:
					break;	
			}
		default:
			break;
			
	}	



	/* Default delay is 1us in the most cases */
	tim = ktime_set(0, NSEC_PER_USEC);


	// hrtimer_start - (re)start an hrtimer on the current CPU
	// @timer:      the timer to be added
	// @tim:        expiry time
	// @mode:       expiry mode: absolute (HRTIMER_MODE_ABS) or
	//             relative (HRTIMER_MODE_REL)
change:
	hrtimer_start(&ctx->timer, tim, HRTIMER_MODE_REL);
	
	printk(KERN_DEBUG "lprf: lprf_async_state_delay - end. %s:%i\n", __FILE__, __LINE__);
}


/*
static void
lprf_sync_state_change_complete(void *context)
{
	struct lprf_state_change *ctx = context;
	struct lprf_local *lp = ctx->lp;

	printk(KERN_DEBUG "lprf: lprf_sync_state_change_complete - start. %s:%i\n",  __FILE__, __LINE__);

	complete(&lp->state_complete);

	printk(KERN_DEBUG "lprf: lprf_sync_state_change_complete - end. %s:%i\n",  __FILE__, __LINE__);
}

static int
lprf_sync_state_change(struct lprf_local *lp, unsigned int state)
{
	unsigned long rc;

	printk(KERN_DEBUG "lprf: lprf_sync_state_change - start. state=%u. %s:%i\n", state, __FILE__, __LINE__);

	lprf_async_state_change(lp, &lp->state, state,
				     lprf_sync_state_change_complete,
				     false);

	// wait 	
	rc = wait_for_completion_timeout(&lp->state_complete,
					 msecs_to_jiffies(100));
	if (!rc) {
		lprf_async_error(lp, &lp->state, -ETIMEDOUT);
		return -ETIMEDOUT;
	}

	printk(KERN_DEBUG "lprf: lprf_sync_state_change - end.%s:%i\n", __FILE__, __LINE__);

	return 0;
}*/


// new function: completion function for async state change (or should we use the sync state change structure?) 
static void lprf_async_state_change_complete(void *context){
	struct lprf_state_change *ctx = context;
	struct lprf_local *lp = ctx->lp;
	int rc;
	u8 buffer = ctx->buf[2]; 

	//printk(KERN_DEBUG "lprf: lprf_async_state_change_complete - start. %s:%i\n",  __FILE__, __LINE__);

	if(lp->is_tx){
		if(ctx->iterations == 0) {
			printk(KERN_DEBUG "lprf: lprf_async_state_change_complete - from TX. buffer=%u. %s:%i\n", buffer,  __FILE__, __LINE__);
		}
		ctx->iterations++;
	
		if(buffer & STATE_SENDING){
			lprf_async_read_reg(lp, RG_SM_STATE, ctx, lprf_async_state_change_complete, 1);	// read SM_STATE again (recursion)
			return;
		} 
		else{	// sending finished, reset SM_MAIN
			printk(KERN_DEBUG "lprf: lprf_async_state_change_complete - reset SM_MAIN. iterations=%d, %s:%i\n",  ctx->iterations, __FILE__, __LINE__);	
			rc = lprf_write_subreg(lp, SR_SM_COMMAND, 0);	
			if (rc){
				return;
			}

			// disable FIFO mode 
			rc = lprf_write_subreg(lp, SR_FIFO_MODE_EN, 0);
			if(rc){
				return;
			}

			lp->is_tx = 0;

			// we need to call 
			ieee802154_xmit_complete(lp->hw, lp->tx_skb, false);
		}
	}
	else{	
		complete(&lp->state_complete);
	}

	printk(KERN_DEBUG "lprf: lprf_async_state_change_complete - end. %s:%i\n",  __FILE__, __LINE__);
}

//function pointer to this is passed as parameter to lprf_async_state_change (by this)
static void lprf_async_state_change_start(void *context)
{
	struct lprf_state_change *ctx = context;
	struct lprf_local *lp = ctx->lp;
	u8 *buf = ctx->buf;
	const u8 sm_main = buf[2]; // get the actual state, no mask
	int rc;
	u8 sm_main_lower_bits;

	printk(KERN_DEBUG "lprf: lprf_async_state_change_start - start, state=%u %s:%i\n", ctx->to_state, __FILE__, __LINE__);

	/* Check for "possible" STATE_TRANSITION_IN_PROGRESS */
	// state machine busy? 
	/*
	if (trx_state == (1 << 4)) { 		
		udelay(1);	// wait and read again 
		lprf_async_read_reg(lp, RG_SM_STATE, ctx, lprf_async_state_change_start, 0);
		return;
	}*/
	
	// get the 4 LSBs from RG_SM_MAIN
	sm_main_lower_bits = sm_main & 15; // 

	buf[0] = 0xC0; 		// write cmd 
	buf[1] = RG_SM_MAIN; 		
	buf[2] = (ctx->to_state << 4) | sm_main_lower_bits;	// shift: SM_COMMAND <7:4> 
	ctx->msg.complete = lprf_async_state_delay;
	rc = spi_async(lp->spi, &ctx->msg);
	if (rc) {
		lprf_async_error(lp, ctx, rc);
	}
	
	printk(KERN_DEBUG "lprf: lprf_async_state_change_start - end. %s:%i\n", __FILE__, __LINE__);
}


static void lprf_async_state_change(struct lprf_local *lp, struct lprf_state_change *ctx, const u8 state, void (*complete)(void *context), const bool output_disable)
{
	/* Initialization for the state change context */
	printk(KERN_DEBUG "lprf: lprf_async_state_change - start, to_state=%u from_state=%u. %s:%i\n", state, ctx->from_state, __FILE__, __LINE__);

	ctx->to_state = state;
	ctx->complete = complete;
	lprf_async_read_reg(lp, RG_SM_MAIN, ctx, lprf_async_state_change_start, output_disable);

	printk(KERN_DEBUG "lprf: lprf_async_state_change - end, state=%u. %s:%i\n", state, __FILE__, __LINE__);

}


static void
lprf_rx_read_frame_complete(void *context)
{
	struct lprf_state_change *ctx = context;
	struct lprf_local *lp = ctx->lp;
	u8 rx_local_buf[LPRF_MAX_BUF - 2];	
	const u8 *buf = ctx->buf;
	struct sk_buff *skb;
	u8 len;

	printk(KERN_DEBUG "lprf: lprf_rx_read_frame_complete - start. %s:%i\n", __FILE__, __LINE__);

	len = buf[1];
	if (!ieee802154_is_valid_psdu_len(len)) {
		dev_vdbg(&lp->spi->dev, "corrupted frame received\n");
		len = IEEE802154_MTU;
	}


	memcpy(rx_local_buf, buf + 2, len);
	ctx->trx.len = 2;


	skb = dev_alloc_skb(IEEE802154_MTU);
	if (!skb) {
		dev_vdbg(&lp->spi->dev, "failed to allocate sk_buff\n");
		return;
	}

	memcpy(skb_put(skb, len), rx_local_buf, len);
	
	//ieee802154_rx(lp->hw, skb);

	// activate polling
	lprf_check_for_data(lp);

	printk(KERN_DEBUG "lprf: lprf_rx_read_frame_complete - end. %s:%i\n", __FILE__, __LINE__);
}


static void
lprf_rx_read_frame(void *context)
{
	struct lprf_state_change *ctx = context;
	struct lprf_local *lp = ctx->lp;
	u8 *buf = ctx->buf;
	int rc;

	printk(KERN_DEBUG "lprf: lprf_rx_read_frame - start. %s:%i\n", __FILE__, __LINE__);

	buf[0] = 0x20;			// Frame Read Access - FRMR
	ctx->trx.len = LPRF_MAX_BUF;	// max buffer length - 256
	ctx->msg.complete = lprf_rx_read_frame_complete;
	rc = spi_async(lp->spi, &ctx->msg);
	if (rc) {
		ctx->trx.len = 2;
		lprf_async_error(lp, ctx, rc);
	}

	printk(KERN_DEBUG "lprf: lprf_rx_read_frame - end. %s:%i\n", __FILE__, __LINE__);
}

// TX
static uint8_t lprf_reverse_bit_order(uint8_t val){
	//printk(KERN_DEBUG "lprf: lprf_reverse_bit_order- start. val=%u %s:%i\n", val, __FILE__, __LINE__);
								// bit masks 
	val = ((val & 0xF0) >> 4) | ((val & 0x0F) << 4);	// 1111 0000	0000 1111
	val = ((val & 0xCC) >> 2) | ((val & 0x33) << 2);	// 1100 1100	0011 0011
	val = ((val & 0xAA) >> 1) | ((val & 0x55) << 1); 	// 1010 1010	0101 0101
	
	//printk(KERN_DEBUG "lprf: lprf_reverse_bit_order- end. val=%u %s:%i\n", val, __FILE__, __LINE__);

	return val;
}

static void lprf_check_state_complete(void *context){
	struct lprf_state_change *ctx = context;
	struct lprf_local *lp = ctx->lp;
	u8 buffer = ctx->buf[2];

	printk(KERN_DEBUG "lprf: lprf_check_state_complete - start. buffer=%u. %s:%i\n", buffer, __FILE__, __LINE__);

	if(!(buffer & STATE_BUSY)){	// State machine not busy?
		printk(KERN_DEBUG "lprf: lprf_check_state_complete, not busy %s:%i\n", __FILE__, __LINE__);

		ctx->from_state = lprf_get_state(buffer);

		printk(KERN_DEBUG "lprf: lprf_check_state_complete, ctx->from_state=%u. %s:%i\n", ctx->from_state, __FILE__, __LINE__);

		lprf_async_state_change(lp, ctx, STATE_CMD_TX, lprf_async_state_change_complete, false);
	}
	else{
		udelay(1); // wait and read again
		lprf_async_read_reg(lp, RG_SM_STATE, ctx, lprf_check_state_complete, 0);
		return;
	}

	printk(KERN_DEBUG "lprf: lprf_check_state_complete - end. %s:%i\n", __FILE__, __LINE__);		
}

static void
lprf_write_frame_complete(void *context)
{
	struct lprf_state_change *ctx = context;
	struct lprf_local *lp = ctx->lp;
	int rc;
	// u8 *buf = ctx->buf;

	printk(KERN_DEBUG "lprf: lprf_write_frame_complete - start. %s:%i\n", __FILE__, __LINE__);

	ctx->trx.len = 2;

	// enable FIFO mode 
	rc = lprf_write_subreg(lp, SR_FIFO_MODE_EN, 1);
	if(rc){
		return;
	}

	// check for the actuel state
	lprf_async_read_reg(lp, RG_SM_STATE, ctx, lprf_check_state_complete, 0);


	// Test: start lprf_check_for_data
	/*
	lp->is_tx = 0;
	lprf_check_for_data(&lp->rx);	
	*/

	printk(KERN_DEBUG "lprf: lprf_write_frame_complete - end. %s:%i\n", __FILE__, __LINE__);
}


static void
lprf_write_frame(void *context)
{
	struct lprf_state_change *ctx = context;
	struct lprf_local *lp = ctx->lp;
	struct sk_buff *skb = lp->tx_skb;
	u8 *buf = ctx->buf;
	int rc, pos;
	// only used for outputs
	int i;	

	printk(KERN_DEBUG "lprf: lprf_write_frame - start. %s:%i\n", __FILE__, __LINE__);

	// 1. frame write access (FRMW): 0x60
	// 2. number of bytes to be written (n+2 bytes, to transmitt n bytes)
	// 3. data 

	buf[0] = 0x60;
	buf[1] = skb->len;

	// ######################################################################
	// buffer begins with the IEEE 802.15.4 header, data starts in buf[11]
	// outputs
	for(i = 0; i < skb->len; i ++){
		printk(KERN_DEBUG "lprf: lprf_write_frame, buf[%u]=0x%02X  %s:%i\n",i, *(skb->data + i), __FILE__, __LINE__);
	}	
	
	printk(KERN_DEBUG "lprf: lprf_write_frame - skb->data=%u %s:%i\n", *(skb->data), __FILE__, __LINE__);
	printk(KERN_DEBUG "lprf: lprf_write_frame - skb->len=%u %s:%i\n", skb->len, __FILE__, __LINE__);
	// ######################################################################

	// bit reversal 
	for(pos = 0; pos < skb->len;  pos++){
		*(skb->data + pos) = lprf_reverse_bit_order(*(skb->data + pos));
	}

	memcpy(buf + 2, skb->data, skb->len);

	ctx->trx.len = skb->len + 2;
	ctx->msg.complete = lprf_write_frame_complete;	
	rc = spi_async(lp->spi, &ctx->msg);
	if (rc) {
		ctx->trx.len = 2;
		lprf_async_error(lp, ctx, rc);
	}
	
	printk(KERN_DEBUG "lprf: lprf_write_frame - end. %s:%i\n", __FILE__, __LINE__);
}



static void
lprf_xmit_start(void *context)
{
	struct lprf_state_change *ctx = context;

	printk(KERN_DEBUG "lprf: lprf_xmit_start - start. %s:%i\n", __FILE__, __LINE__);

	// write data into FIFO	
	lprf_write_frame(ctx);

	printk(KERN_DEBUG "lprf: lprf_xmit_start - end. %s:%i\n", __FILE__, __LINE__);
}


static int lprf_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)    //part of struct ieee802154_ops lprf_ops
{
	struct lprf_local *lp = hw->priv;
	struct lprf_state_change *ctx = &lp->tx;

	printk(KERN_DEBUG "lprf: lprf_xmit - start. %s:%i\n", __FILE__, __LINE__);

	// set TX flag
	lp->is_tx = 1;


	lp->tx_skb = skb;	// skb contains the buffer starting from the IEEE 802.15.4 header, is sent in lprf_write_frame (also contains the data)
	lp->tx_retry = 0;

	// printk(KERN_DEBUG "lprf: lprf_xmit, skb->data=%u. %s:%i\n", *(skb->data), __FILE__, __LINE__);
	// printk(KERN_DEBUG "lprf: lprf_xmit, *(skb->data + 11)=%u. %s:%i\n", *(skb->data + 11), __FILE__, __LINE__);
	// printk(KERN_DEBUG "lprf: lprf_xmit, skb->head=%u. %s:%i\n", *(skb->head), __FILE__, __LINE__);

	lprf_xmit_start(ctx);

	printk(KERN_DEBUG "lprf: lprf_xmit - end. %s:%i\n", __FILE__, __LINE__);

	return 0;
}




// first solutuion for RX polling (not tested yet)
static enum hrtimer_restart lprf_check_for_data_timer(struct hrtimer *timer){
	struct lprf_state_change *ctx = container_of(timer, struct lprf_state_change, timer);	
	unsigned int data = 0;
	unsigned int sending_bit = 0;	
	int rc;	
	ktime_t tim;

	// FIFO_EMPTY_BIT
	rc = __lprf_read(lp, RG_SM_FIFO , &data);
	if(rc){
		printk(KERN_DEBUG "lprf: lprf_check_for_data - reading RG_SM_FIFO failed. %s:%i\n", __FILE__, __LINE__);
		return rc;	
	}
	
	// SM_SENDING 
	rc = __lprf_read(lp, RG_SM_STATE , &sending_bit);
	if(rc){
		printk(KERN_DEBUG "lprf: lprf_check_for_data - reading RG_SM_STATE failed. %s:%i\n", __FILE__, __LINE__);
		return rc;	
	}

	// check SENDING bit
	if(!ctx->lp->is_tx){	
		if(!(0x02 & data) && !(0x04 & sending_bit)){
			lprf_rx_read_frame(ctx);
		}
		else {
			tim = ktime_set(0, 1000 * NSEC_PER_USEC);	// set the right time
			hrtimer_start(&ctx->timer, tim, HRTIMER_MODE_REL);
		}
	}

	return HRTIMER_NORESTART;
}

// first solutuion for RX polling (not tested yet)
static void lprf_check_for_data(struct lprf_local *lp){
	struct lprf_state_change *ctx = &lp->rx; 
	ktime_t tim;

	printk(KERN_DEBUG "lprf: lprf_check_for_data - start. %s:%i\n", __FILE__, __LINE__);


	tim = ktime_set(0, 1000 * NSEC_PER_USEC);	// set the right time
	
	hrtimer_start(&ctx->timer, tim, HRTIMER_MODE_REL);

	
	printk(KERN_DEBUG "lprf: lprf_check_for_data - end. %s:%i\n", __FILE__, __LINE__);
}


static int lprf_start(struct ieee802154_hw *hw)    //part of struct ieee802154_ops lprf_ops
{
	printk(KERN_DEBUG "lprf: lprf_start - start %s:%i\n", __FILE__, __LINE__);
	
	// start to check for received data
	//lprf_check_for_data(hw->priv);

	return 0;
}

// ieee802154_ops lprf_ops - file operations for ieee802154 layer
static int lprf_ed(struct ieee802154_hw *hw, u8 *level)    //part of struct ieee802154_ops lprf_ops
{
	printk(KERN_DEBUG "lprf: lprf_ed - not implemented. %s:%i\n", __FILE__, __LINE__);	

	return 0;
}

static void lprf_stop(struct ieee802154_hw *hw)    //part of struct ieee802154_ops lprf_ops
{
	printk(KERN_DEBUG "lprf: lprf_stop - not implemented. %s:%i\n", __FILE__, __LINE__);
}


static int lprf_set_channel(struct lprf_local *lp, u8 page, u8 channel)		//part of struct lprf_chip_data, stored in lp->data
{
	printk(KERN_DEBUG "lprf: lprf_set_channel - not implemented. %s:%i\n", __FILE__, __LINE__);

	return 0;	
}

static int lprf_channel(struct ieee802154_hw *hw, u8 page, u8 channel)     //part of struct ieee802154_ops lprf_ops
{
	struct lprf_local *lp = hw->priv;
	int rc;
	rc = lp->data->set_channel(lp, page, channel);
	return rc;
}

static int lprf_set_hw_addr_filt(struct ieee802154_hw *hw, struct ieee802154_hw_addr_filt *filt, unsigned long changed)    //part of struct ieee802154_ops lprf_ops
{
	// we have no hardware Frame Filter
	// Do we need some implementation in software?	 

	/**
	struct ieee802154_hw_addr_filt - hardware address filtering settings
 
 	@pan_id: pan_id which should be set to the hardware address filter.
 
 	@short_addr: short_addr which should be set to the hardware address filter.
 
 	@ieee_addr: extended address which should be set to the hardware address filter.
 
 	@pan_coord: boolean if hardware filtering should be operate as coordinator.
	*/

	printk(KERN_DEBUG "lprf: lprf_set_hw_addr_filt - start. %s:%i\n", __FILE__, __LINE__);

	printk(KERN_DEBUG "lprf: lprf_set_hw_addr_filt - not implemented. %s:%i\n", __FILE__, __LINE__);

	printk(KERN_DEBUG "lprf: lprf_set_hw_addr_filt - end. %s:%i\n", __FILE__, __LINE__);

	return 0;
}


static int
lprf_get_desens_steps(struct lprf_local *lp, s32 level)
{
	printk(KERN_DEBUG "at86rf230: at86rf23x_get_desens_steps. %s:%i\n", __FILE__, __LINE__);

	return (level - lp->data->rssi_base_val) / 2;
}


static int lprf_set_txpower(struct ieee802154_hw *hw, int db)    //part of struct ieee802154_ops lprf_ops
{
	printk(KERN_DEBUG "lprf: lprf_set_txpower - not implemented. %s:%i\n", __FILE__, __LINE__);
	return 0;
}

static int lprf_set_lbt(struct ieee802154_hw *hw, bool on)    //part of struct ieee802154_ops lprf_ops
{
	printk(KERN_DEBUG "lprf: lprf_set_lbt - not implemented. %s:%i\n", __FILE__, __LINE__);
	return 0;
}

static int lprf_set_cca_mode(struct ieee802154_hw *hw, const struct wpan_phy_cca *cca)    //part of struct ieee802154_ops lprf_ops
{	
	printk(KERN_DEBUG "lprf: lprf_set_cca_mode - not implemented. %s:%i\n", __FILE__, __LINE__);
	return 0;
}

static int lprf_set_cca_ed_level(struct ieee802154_hw *hw, s32 level)     //part of struct ieee802154_ops lprf_ops
{
	printk(KERN_DEBUG "lprf: lprf_set_cca_ed_level - not implemented. %s:%i\n", __FILE__, __LINE__);

	return 0;    
}

static int lprf_set_csma_params(struct ieee802154_hw *hw, u8 min_be, u8 max_be, u8 retries)    //part of struct ieee802154_ops lprf_ops
{
	printk(KERN_DEBUG "lprf: lprf_set_csma_params - not implemented. %s:%i\n", __FILE__, __LINE__);

	return 0;
}

static int lprf_set_frame_retries(struct ieee802154_hw *hw, s8 retries)    //part of struct ieee802154_ops lprf_ops
{
	struct lprf_local *lp = hw->priv;
	printk(KERN_DEBUG "lprf: lprf_set_frame_retries - not implemented %s:%i\n", __FILE__, __LINE__);

	lp->tx_aret = 0;
	lp->max_frame_retries = 0;

	return 0;
}

static int lprf_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)    //part of struct ieee802154_ops lprf_ops
{
	printk(KERN_DEBUG "lprf: lprf_set_promiscuous_mode - not implemented %s:%i\n", __FILE__, __LINE__);
	return 0;
}


// setup functions
// gets called only by lprf_probe()
static int lprf_hw_init(struct lprf_local *lp)
{
	int rc;

	printk(KERN_DEBUG "lprf: lprf_hw_init - start%s:%i\n", __FILE__, __LINE__);	

	// enable extern supply clock 
	rc = __lprf_write(lp, RG_CLK_FALLBACK, 1);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_OSCI_BUFFER_EN, 1);		
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_ULP_BUFFER_EN, 1);	
	if (rc){
		return rc;
	}
	
	rc = __lprf_write(lp, RG_CLK_MAIN, 41);	//0b00101001; 0x29
	if (rc){
		return rc;
	}

	rc = __lprf_write(lp, RG_CLK_SET, 102);	//0b01100110; 0x60
	if (rc){
		return rc;
	}

	// enable all LDOs
	rc = __lprf_write(lp, RG_LDO, 57);	//0b00111001; 0x39
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_LDO_D_VOUT, 15);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_LDO_PLL_VOUT, 31);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_LDO_VCO_VOUT, 31);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_LDO_TX24_VOUT, 25);
	if (rc){
		return rc;
	}

	// Buffer PLL
	rc = lprf_write_subreg(lp, SR_PLL_BUFFER_EN, 1);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_EN, 1);
	if (rc){
		return rc;
	}

	// reset PLL
	rc = lprf_write_subreg(lp, SR_PLL_RESETB, 0);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_RESETB, 1);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_CTRL_CLK_ADC, 0);
	if (rc){
		return rc;
	}

	/*
	// set channel 
	rc = lprf_write_subreg(lp, SR_PLL_CHN_INT_IN, 105);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_VCO_TUNE, 196);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_IREF_PLL_CTRLB, 0);
	if (rc){
		return rc;
	}

	rc = __lprf_write(lp, RG_PLL_MAIN, 152);	//0b10011000; 0x98
	if (rc){
		return rc;
	}*/
	

	// state machine operations 
	rc = lprf_write_subreg(lp, SR_WAKEUPONSPI, 0);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_TX_CHAN_INT, 112);	//Set PLL divider, write PLL_CHN through statemachine,    //1792MHz an PLL_OUT
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_TX_CHAN_FRAC_H, 0);	
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_TX_CHAN_FRAC_M, 0);	
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_TX_CHAN_FRAC_L, 0);	
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_VCO_TUNE, 152);	//set VCO tune word for 1792MHz an PLL_OUT
	if (rc){
		return rc;
	}

	//configure modulation
	rc = lprf_write_subreg(lp, SR_TX_ON_CHIP_MOD, 1);		// enable on-chip modulation	
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_MOD_EN, 1);			//enable modulation of PLL
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_MOD_DATA_RATE, 1);	//set tx data rate to <500kBit/s
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_TX_ON_CHIP_MOD_SP, 9);    //set TX_ON_CHIP_MOD_SP to ... (3: 250kHz; 8: 20kHz; 9: 10kHz)
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_MOD_FREQ_DEV, 4);		//set frequency deviation to 128kHz
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_INVERT_FIFO_CLK, 0);		//invert FIFO clock to enable R/W operation to the fifo
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_TX_SPI_FIFO_OUT_EN, 1);	// enable FIFO -> txpath
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_TPM_COMP_EN, 0); 	// disable two point modulation compensation loop
	if (rc){
		return rc;
	}

	// enable the state machine 
	rc = lprf_write_subreg(lp, SR_SM_EN, 1); 
	if (rc){
		return rc;
	}

	// set TX mode (00: 2.4 GHz, 01: 800 MHz, 10: 433 MHz, 11: 433 MHz)
	rc = lprf_write_subreg(lp, SR_TX_MODE, 0);
	if(rc){
		return rc;
	}


	/*
	rc = lprf_write_subreg(lp, SR_TX_PWR_CTRL, 15);		//set TX power control to maximum
	if (rc){
		return rc;
	}

		rc = __lprf_write(lp, RG_TX24, 96);			//0b01100000; 0x60
	if (rc){						// enable tx mixer and enable 2.4 GHz tx frontend 
		return rc;
	}

	rc = __lprf_write(lp, RG_TX_MAIN, 60);			//0b00111100; 0x3C
	if (rc){						// enable tx frontend, enable on-chip modulation, enable FIFO -> txpath
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_TX_AMPLI_OUT_MAN_L, 240); // set lower 7 of 8 bits to 240
	if (rc){
		return rc;
	}

	rc = __lprf_write(lp, RG_SM_TX_POWER_CTRL, 31);		//0b00011111; 0x1F
	if (rc){						// set maximum PA output level (SM), set MAXAMP bit (SM)
		return rc;
	}
	*/
	

	// set time registers 
	rc = lprf_write_subreg(lp, SR_POWER_TX_TIME, T_POWER_TX_TIME);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_POWER_RX_TIME, T_POWER_RX_TIME);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_PON_TIME, T_PLL_PON_TIME);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PLL_SET_TIME  , T_PLL_SET_TIME);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_TX_TIME , T_TX_TIME);
	if (rc){
		return rc;
	}

	rc = lprf_write_subreg(lp, SR_PD_EN_TIME , T_PD_EN_TIME);
	if (rc){
		return rc;
	}
	
	// enable DIRECT_RX
	rc = lprf_write_subreg(lp, SR_DIRECT_RX , 1);
	if(rc){
		return rc;
	}
	
	printk(KERN_DEBUG "lprf: lprf_hw_init - end. %s:%i\n", __FILE__, __LINE__);


	return 0;
}


// gets called only by lprf_probe()
static int
lprf_get_pdata(struct spi_device *spi, int *rstn, int *slp_tr,
		    u8 *xtal_trim)
{
	struct lprf_platform_data *pdata = spi->dev.platform_data; // declaration in lprf.h 
	int ret;

	printk(KERN_DEBUG "lprf: lprf_get_pdata - start %s:%i\n", __FILE__, __LINE__);

	if (!IS_ENABLED(CONFIG_OF) || !spi->dev.of_node) {
		if (!pdata)
			return -ENOENT;

		*rstn = pdata->rstn;
		*slp_tr = pdata->slp_tr;
		*xtal_trim = pdata->xtal_trim;
		
		printk(KERN_DEBUG "lprf: lprf_get_pdata - 1 rstn: %i, slp_tr: %i, xtal_trim: %i: %s:%i\n", *rstn, *slp_tr, *xtal_trim, __FILE__, __LINE__);

		return 0;
	}

	*rstn = of_get_named_gpio(spi->dev.of_node, "reset-gpio", 0);
	*slp_tr = of_get_named_gpio(spi->dev.of_node, "sleep-gpio", 0);
	ret = of_property_read_u8(spi->dev.of_node, "xtal-trim", xtal_trim);
	printk(KERN_DEBUG "lprf: lprf_get_pdata - 2 rstn: %i, slp_tr: %i, xtal_trim: %i: %s:%i\n", *rstn, *slp_tr, *xtal_trim, __FILE__, __LINE__);
	if (ret < 0 && ret != -EINVAL)
		return ret;

	printk(KERN_DEBUG "lprf: lprf_get_pdata - end %s:%i\n", __FILE__, __LINE__);

	return 0;
}


static struct lprf_chip_data lprf_data = {
	// waiting times
	.t_from_sleep_to_tx = (T_POWER_TX_TIME + T_PLL_PON_TIME + T_TX_TIME) / 32, 	// get times in us (clock is 32 MHz)
	.t_from_tx_idle_to_tx = (T_TX_TIME) / 32,
	.t_from_rx_power_to_tx = (T_POWER_TX_TIME + T_TX_TIME) / 32,
	.t_from_rx_pll_to_tx = (T_TX_TIME + T_PLL_SET_TIME) / 32,

	.t_from_sleep_to_rx = (T_POWER_RX_TIME + T_PLL_PON_TIME + T_PD_EN_TIME) / 32,
	.t_from_tx_to_rx = (T_POWER_RX_TIME - T_POWER_TX_TIME + T_PD_EN_TIME + T_PLL_SET_TIME) / 32,
	.t_from_rxhold_to_rx = 0, 	//(T_POWER_RX_TIME + T_PLL_PON_TIME + T_PD_EN_TIME - 1023) / 32, 	// (max time - state_ctr_max) - negativ???

	.t_from_sleep_to_txidle = (T_POWER_TX_TIME + T_PLL_PON_TIME) / 32,
	.t_from_tx_to_txidle = 0, // same problem as above [(T_POWER_TX_TIME + T_PLL_PON_TIME - 1023) / 32]
	.t_from_rx_to_txidle = T_POWER_TX_TIME / 32,
	.t_from_rx_pll_to_txidle = T_PLL_SET_TIME / 32,

	.t_from_sleep_to_rxhold = (T_POWER_RX_TIME + T_PLL_PON_TIME) / 32,
	.t_from_tx_to_rxhold = (T_POWER_RX_TIME - T_POWER_TX_TIME + T_PLL_SET_TIME) / 32,
	.t_from_rx_to_rxhold = 0,

	.set_channel = lprf_set_channel,
	.get_desense_steps = lprf_get_desens_steps
};


// gets called only by lprf_probe()
static int lprf_detect_device(struct lprf_local *lp)
{
	unsigned int val;	
	int rc;	
	
	printk(KERN_DEBUG "lprf: lprf_detect_device - start %s:%i\n", __FILE__, __LINE__);

	// get chip ID
	// higher register - value: 0x1A
	rc = __lprf_read(lp, RG_CHIP_ID_H, &val);
	if(rc){
		return rc;
	}
	printk(KERN_DEBUG "lprf: lprf_detect_device - CHIP_ID_H=%02X %s:%i\n", val, __FILE__, __LINE__);

	if(val != 0x1A){
		printk(KERN_DEBUG "lprf: lprf_detect_device - device detection failed %s:%i\n", __FILE__, __LINE__);
		return -1;
	}	

	// lower register - value: 0x51
	rc = __lprf_read(lp, RG_CHIP_ID_L, &val);
	if(rc){
		return rc;
	}
	printk(KERN_DEBUG "lprf: lprf_detect_device - CHIP_ID_L=%02X %s:%i\n", val, __FILE__, __LINE__);

	if(val != 0x51){
		printk(KERN_DEBUG "lprf: lprf_detect_device - device detection failed %s:%i\n", __FILE__, __LINE__);
		return -1;
	}


	/*
	// Indicates that xmitter will add FCS on it's own. 
		#define IEEE802154_HW_TX_OMIT_CKSUM     0x00000001
	// Indicates that receiver will autorespond with ACK frames.
		#define IEEE802154_HW_AACK              0x00000002
	// Indicates that transceiver will support transmit power setting.
		#define IEEE802154_HW_TXPOWER           0x00000004
	// Indicates that transceiver will support listen before transmit. 
		#define IEEE802154_HW_LBT               0x00000008
	// Indicates that transceiver will support cca mode setting.
		#define IEEE802154_HW_CCA_MODE          0x00000010
	// Indicates that transceiver will support cca ed level setting. 
		#define IEEE802154_HW_CCA_ED_LEVEL      0x00000020
	// Indicates that transceiver will support csma (max_be, min_be, csma retries) settings.
		#define IEEE802154_HW_CSMA_PARAMS       0x00000040
	// Indicates that transceiver will support ARET frame retries setting.
		#define IEEE802154_HW_FRAME_RETRIES     0x00000080
	// Indicates that transceiver will support hardware address filter setting.
		#define IEEE802154_HW_AFILT             0x00000100
	// Indicates that transceiver will support promiscuous mode setting.
		#define IEEE802154_HW_PROMISCUOUS       0x00000200
	// Indicates that receiver omits FCS. 
		#define IEEE802154_HW_RX_OMIT_CKSUM     0x00000400
	// Indicates that receiver will not filter frames with bad checksum.
		#define IEEE802154_HW_RX_DROP_BAD_CKSUM 0x00000800
	*/

	//lp->hw->flags = IEEE802154_HW_TX_OMIT_CKSUM;		// Which flags are missing?

	// @NL802154_CCA_ENERGY: Energy above threshold
	lp->hw->phy->cca.mode = NL802154_CCA_ENERGY;

	lp->data = &lprf_data;
	//lp->hw->phy->channels_supported[0] = 0x7FFF800;
	lp->hw->phy->supported.channels[0] = 0x7FFF800;	
	lp->hw->phy->current_channel = 11;
	lp->hw->phy->symbol_duration = 16;

	printk(KERN_DEBUG "lprf: lprf_detect_device - end %s:%i\n", __FILE__, __LINE__);

	return 0;
}


static void lprf_setup_spi_messages(struct lprf_local *lp)
{
	lp->state.lp = lp;
	spi_message_init(&lp->state.msg);
	lp->state.msg.context = &lp->state;
	lp->state.trx.len = 3;
	lp->state.trx.tx_buf = lp->state.buf;
	lp->state.trx.rx_buf = lp->state.buf;
	spi_message_add_tail(&lp->state.trx, &lp->state.msg);
	hrtimer_init(&lp->state.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);	//initialise timer 
	lp->state.timer.function = lprf_async_state_timer;
	lp->state.iterations = 0;     //count state_change_complete iterations

	lp->tx.lp = lp;
	spi_message_init(&lp->tx.msg);
	lp->tx.msg.context = &lp->tx;
	lp->tx.trx.len = 3;
	lp->tx.trx.tx_buf = lp->tx.buf;
	lp->tx.trx.rx_buf = lp->tx.buf;
	spi_message_add_tail(&lp->tx.trx, &lp->tx.msg);
	hrtimer_init(&lp->tx.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);		//initialise timer
	lp->tx.timer.function = lprf_async_state_timer;
	lp->tx.iterations = 0;     //count state_change_complete iterations

	lp->rx.lp = lp;
	spi_message_init(&lp->rx.msg);
	lp->rx.msg.context = &lp->rx;
	lp->rx.trx.len = 3;
	lp->rx.trx.tx_buf = lp->rx.buf;
	lp->rx.trx.rx_buf = lp->rx.buf;
	spi_message_add_tail(&lp->rx.trx, &lp->rx.msg);
	hrtimer_init(&lp->rx.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);		//initialise timer
	lp->rx.timer.function = lprf_check_for_data_timer;

	// is used for the char driver
	lp->debug.lp = lp;
	spi_message_init(&lp->debug.msg);
	lp->debug.msg.context = &lp->debug;
	lp->debug.trx.len = 3;
	lp->debug.from_state = 15;	// debugging (no other funtion)
	lp->debug.trx.tx_buf = lp->debug.buf;
	lp->debug.trx.rx_buf = lp->debug.buf;
	spi_message_add_tail(&lp->debug.trx, &lp->debug.msg);
	hrtimer_init(&lp->debug.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);	//initialise timer - not needed here
	lp->debug.timer.function = lprf_async_state_timer;
}

static int lprf_probe(struct spi_device *spi)   // part of struct spi_driver lprf_driver
{	
	struct ieee802154_hw *hw;
	//struct lprf_local *lp;	// the definition of lp is global (lprf.h)			
	int rc, rstn, slp_tr;
	u8 xtal_trim = 0;


	printk(KERN_DEBUG "lprf: lprf_probe - start: %s:%i\n", __FILE__, __LINE__);	


	rc = lprf_get_pdata(spi, &rstn, &slp_tr, &xtal_trim);		
	if (rc < 0) {
		dev_err(&spi->dev, "failed to parse platform_data: %d\n", rc);
		return rc;
	}

	if (gpio_is_valid(rstn)) {
		rc = devm_gpio_request_one(&spi->dev, rstn,
					   GPIOF_OUT_INIT_HIGH, "rstn");
		if (rc)
			return rc;
	}

	if (gpio_is_valid(slp_tr)) {
		rc = devm_gpio_request_one(&spi->dev, slp_tr,
					   GPIOF_OUT_INIT_LOW, "slp_tr");
		if (rc)
			return rc;
	}

	// Reset 
	if (gpio_is_valid(rstn)) {
		udelay(1);
		gpio_set_value(rstn, 0);
		udelay(1);
		gpio_set_value(rstn, 1);
		usleep_range(120, 240);
	}

	hw = ieee802154_alloc_hw(sizeof(*lp), &lprf_ops);
	if (!hw)
		return -ENOMEM;

	lp = hw->priv;  //needed for 'lp' access in functions that do not get 'lp' passed
	lp->hw = hw;
	lp->spi = spi;
	lp->slp_tr = slp_tr;
	hw->parent = &spi->dev;
	//hw->vif_data_size = sizeof(*lp);
	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

	// initialise hrtimer for rx
	//hrtimer_init(&lp->rx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	//lp->rx_timer.function = lprf_check_for_data_timer;

	// initialisation of regmap
	lp->regmap = devm_regmap_init_spi(spi, &lprf_regmap_spi_config);
	if (IS_ERR(lp->regmap)) {
		rc = PTR_ERR(lp->regmap);
		dev_err(&spi->dev, "Failed to allocate register map: %d\n",	rc);
		goto free_dev;
	}

	//setup different spi messages
	lprf_setup_spi_messages(lp);
	

	rc = lprf_detect_device(lp);
	if (rc < 0)
		goto free_dev;

	
	// initialise completion 
	init_completion(&lp->state_complete);
	

	// set lp to spi->driver_data
	spi_set_drvdata(spi, lp);

	rc = lprf_hw_init(lp);
	if (rc)
		goto free_dev;
	
	// register the char driver 
	scull_init_module();

	
	rc = ieee802154_register_hw(lp->hw);
	if (rc)
		goto free_dev;
	
	
	printk(KERN_DEBUG "lprf: lprf_probe - end: %s:%i\n", __FILE__, __LINE__);	

	return rc;

free_dev:
	ieee802154_free_hw(lp->hw);

	printk(KERN_DEBUG "lprf: lprf_probe - enderror: %s:%i\n", __FILE__, __LINE__);	
	return rc;
}

static int lprf_remove(struct spi_device *spi)   // part of struct spi_driver lprf_driver
{
	struct lprf_local *lp = spi_get_drvdata(spi);
	printk(KERN_DEBUG "lprf: lprf_remove - start: %s:%i\n", __FILE__, __LINE__);
	// !modify!
	//lprf_write_subreg(lp, SR_IRQ_MASK, 0);
	ieee802154_unregister_hw(lp->hw);
	ieee802154_free_hw(lp->hw); 

	dev_dbg(&spi->dev, "unregistered lprf\n");
	scull_cleanup_module();
	printk(KERN_DEBUG "lprf: lprf_remove - end: %s:%i\n", __FILE__, __LINE__);
	return 0;
}


int scull_major =   SCULL_MAJOR;
int scull_minor =   0;
int scull_nr_devs = SCULL_NR_DEVS;	// number of bare scull devices 
int scull_quantum = SCULL_QUANTUM;
int scull_qset =    SCULL_QSET;

module_param(scull_major, int, S_IRUGO);
module_param(scull_minor, int, S_IRUGO);
module_param(scull_nr_devs, int, S_IRUGO);
module_param(scull_quantum, int, S_IRUGO);
module_param(scull_qset, int, S_IRUGO);

/*
 * Empty out the scull device; must be called with the device
 * semaphore held. 
 */ 
int scull_trim(struct scull_dev *dev)
{
	struct scull_qset *next, *dptr;
	int qset = dev->qset;   // "dev" is not-null 
	int i;
	printk(KERN_DEBUG "lprf: scull_trim - start: %s:%i\n", __FILE__, __LINE__);
	for (dptr = dev->data; dptr; dptr = next) { // all the list items 
		if (dptr->data) {
			for (i = 0; i < qset; i++)
				kfree(dptr->data[i]);
			kfree(dptr->data);
			dptr->data = NULL;
		}
		next = dptr->next;
		kfree(dptr);
	}
	dev->size = 0;
	dev->quantum = scull_quantum;
	dev->qset = scull_qset;
	dev->data = NULL;
	printk(KERN_DEBUG "lprf: scull_trim - end: quantum=%d  %s:%i\n", scull_quantum, __FILE__, __LINE__);	
	return 0;
}



/*
 * Open and close
 */
int scull_open(struct inode *inode, struct file *filp)
{
	struct scull_dev *dev; /* device information */
	dev = container_of(inode->i_cdev, struct scull_dev, cdev);    //http://stackoverflow.com/questions/15832301/understanding-container-of-macro-in-linux-kerne0;
	filp->private_data = dev; /* for other methods */
	dev->size = FILE_SIZE;	//define: FILE_SIZE 8192

	printk(KERN_DEBUG "lprf: scull_open: dev->size: %ld   FILE_SIZE=%d 5=%d\n", dev->size, FILE_SIZE, 5);	
	return 0;          // success 
}

int scull_release(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "lprf: scull_release: %s:%i\n", __FILE__, __LINE__);
	return 0;
}



loff_t scull_llseek(struct file *filp, loff_t offset, int whence)
{
	loff_t newpos; 
	printk(KERN_DEBUG "lprf: scull_llseek - start");
	//printk(KERN_DEBUG "lprf: scull_llseek: filp->f_pos=%lld   offset=%lld    whence=%d  5=%d\n", filp->f_pos, offset, whence, 5);
	switch(whence) {
	  case 0:   /* SEEK_SET */
		newpos = offset;
		break;
		
	  case 1:   /* SEEK_CUR */
		newpos = filp->f_pos + offset;
		break;
		
	  case 2:   /* SEEK_END */
		newpos = FILE_SIZE + offset;
		break;	
			
	  default:
		return -EINVAL;
		break;		
	}
	
	if (newpos >= FILE_SIZE)
	{
		printk(KERN_DEBUG "lprf: scull_llseek: newpos >= FILE_SIZE!\n");
		return -EINVAL;
	}
	if (newpos < 0)
	{
		printk(KERN_DEBUG "lprf: scull_llseek: newpos < 0!\n");
		return -EINVAL;
	}
		
	filp->f_pos = newpos;  
	//printk(KERN_DEBUG "lprf: scull_llseek end filp->f_pos=%lld   offset=%lld    whence=%d  5=%d\n", filp->f_pos, offset, whence, 5);	
	printk(KERN_DEBUG "lprf: scull_llseek - end");
	return newpos;
}

/*
 * called when userspace reads from device file, e.g. cat /dev/lprf0
 * count is the size of the requested data transfer 
 * f_pos indicates the file position the user is accessing 
 */
ssize_t scull_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t retval = 0;
	unsigned int myval = 1;
	unsigned int dev_size = 2 + *f_pos;
	unsigned int rc;
	//struct scull_dev *dev = filp->private_data; 
	struct lprf_state_change *ctx = &(lp->debug);

	printk(KERN_DEBUG "lprf: scull_read - start.");

	// read from regmap cache
	//myrc = __lprf_read(lp, *f_pos, &myval);
	//if(myrc){
	//	printk(KERN_DEBUG "lprf: scull_read: Error reading register\n");
	//}
	
	// read from hardware via spi_sync()
	lprf_sync_read_reg_debugging(lp, *f_pos, ctx, 0);

	//printk(KERN_DEBUG "lprf: scull_read: lp->debug.buf[0]=%u lp->debug.buf[1]=%u lp->debug.buf[2]=%u \n", ctx->buf[0], ctx->buf[1], ctx->buf[2]);

	// get read value
	myval = ctx->buf[2];	
	
	count = 1;			
	
	//printk(KERN_DEBUG "lprf: scull_read:      *f_pos=0x%04X   count=%2d   myval=0x%02X  5=%d\n", (unsigned int)*f_pos, count, myval, 5);
	rc = copy_to_user(buf, &myval, count);	
	if (rc) {		//1 char is transfered, count = 1	
		retval = -EFAULT;
		goto out;
	}
	//printk(KERN_DEBUG "lprf: scull_read: *f_pos=0x%04X   count=%2d   myval=0x%02X *buf=%u 5=%d\n", (unsigned int)*f_pos, count, myval, (unsigned int)*buf, 5);
	
	*f_pos += count;	//+= 1

		
	if (*f_pos + count > dev_size){		
		count = dev_size - *f_pos;
		printk(KERN_DEBUG "lprf: scull_read: count=%u\n", count);
		return 0;
	}

	retval = count;

  out:
	printk(KERN_DEBUG "lprf: scull_read - end. %s:%i\n", __FILE__, __LINE__);	
	return retval;
	return 0;
}


int lprf_read_status(void)
{
/*
	int rc;
	u8 *tx_buf, *rx_buf;
	
	tx_buf = kmalloc(sizeof(u8), GFP_KERNEL);
	rx_buf = kmalloc(sizeof(u8), GFP_KERNEL);
	memset(tx_buf, 0x05, sizeof(u8));
	memset(rx_buf, 0, sizeof(u8));
	rc = spi_write_then_read(lp->spi, tx_buf, 1, rx_buf, 1);
	if(rc)
		printk(KERN_ERR "lprf: spi_write_then_read failed at %s, %i\n", __FILE__, __LINE__);
	return *rx_buf;
	*/
	
	return 0;
}


// called when userspace writes to device file, e.g. ls -l > /dev/lprf0  
ssize_t scull_write(struct file *filp, const char __user *userbuf, size_t count, loff_t *f_pos)
{
	ssize_t retval = -ENOMEM; // value used in "goto out" statements 
	u8 *buffer2;
	unsigned int addr;
	int rc;
	buffer2 = kmalloc(sizeof(u8), GFP_KERNEL);	
	memset(buffer2, 0, sizeof(u8));

	printk(KERN_DEBUG "lprf: scull_write: start1: STATUS=0x%02X \n", lprf_read_status());  //XXX

	printk(KERN_DEBUG "lprf: scull_write: start2: STATUS=0x%02X \n", lprf_read_status());  //XXX	
	
	//write data
	count = 1;	
	rc = copy_from_user(buffer2, userbuf, count);
	if(rc){
		return -EFAULT;
	}

	addr = (unsigned int)(*f_pos);	// is set by fseek

	printk(KERN_DEBUG "lprf: scull_write: waiting for completion!\n");
//	wait_for_completion(&lp->state_complete);
	printk(KERN_DEBUG "lprf: about to write data. &lp=%p, addr=%d, buffer2=%p buffer2=%c userbuf=%c\n", &lp, addr, buffer2, *buffer2, *userbuf);

	// __lprf_write(lp, addr, *buffer2);	
	*f_pos += count;
	retval = count;

	printk(KERN_DEBUG "lprf: scull_write - end: addr=0x%02X *f_pos=0x%02X buffer=0x%02X retval=%d status=0x%02X \n", addr, (unsigned int)(*f_pos), *buffer2, retval, lprf_read_status());	
	return retval;
}


/*
 * The ioctl() implementation
 * call has changed from int scull_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
 * to                   long scull_ioctl(                     struct file *filp, unsigned int cmd, unsigned long arg)
 * Source http://tuxthink.blogspot.in/2012/12/implementing-ioctl-call-for-kernel.html
 */
//int scull_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
long scull_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0, tmp;
	int retval = 0;
	  
	  //extract the type and number bitfields, and don't decode 
	  //wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 
	if (_IOC_TYPE(cmd) != SCULL_IOC_MAGIC)  //21505
	{
	  	//printk(KERN_DEBUG "lprf: ioctl case _IOC_TYPE(cmd) != SCULL_IOC_MAGIC\n");	
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > SCULL_IOC_MAXNR) 
	{
	  	//printk(KERN_DEBUG "lprf: ioctl case cmd > SCULL_ICO_MAXNR\n");	
		return -ENOTTY;
	}

	/* 
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch(cmd) {

	  case SCULL_IOCRESET:
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCRESET\n");
		scull_quantum = SCULL_QUANTUM;
		scull_qset = SCULL_QSET;
		break;
        
	  case SCULL_IOCSQUANTUM: // Set: arg points to the value 
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCSQUANTUM\n");	  
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		retval = __get_user(scull_quantum, (int __user *)arg);
		break;

	  case SCULL_IOCTQUANTUM: //Tell: arg is the value 
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCTQUANTUM\n");	  
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		scull_quantum = arg;
		break;

	  case SCULL_IOCGQUANTUM: // Get: arg is pointer to result 
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCGQUANTUM\n");
		retval = __put_user(scull_quantum, (int __user *)arg);
		break;

	  case SCULL_IOCQQUANTUM: // Query: return it (it's positive) 
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCQQANTUM\n");  
		return scull_quantum;

	  case SCULL_IOCXQUANTUM: // eXchange: use arg as pointer 
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCXQANTUM\n"); 	  
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		tmp = scull_quantum;
		retval = __get_user(scull_quantum, (int __user *)arg);
		if (retval == 0)
			retval = __put_user(tmp, (int __user *)arg);
		break;

	  case SCULL_IOCHQUANTUM: // sHift: like Tell + Query 
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCHQANTUM\n"); 	  
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		tmp = scull_quantum;
		scull_quantum = arg;
		return tmp;
        
	  case SCULL_IOCSQSET:
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCQSET\n");
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		retval = __get_user(scull_qset, (int __user *)arg);
		break;

	  case SCULL_IOCTQSET:
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCTQSET\n");	  	  	  
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		scull_qset = arg;
		break;

	  case SCULL_IOCGQSET:
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCGQSET\n");
		retval = __put_user(scull_qset, (int __user *)arg);
		break;

	  case SCULL_IOCQQSET:
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCQQSET\n");	  	  	  
		return scull_qset;

	  case SCULL_IOCXQSET:
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCXQSET\n");  	  	  
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		tmp = scull_qset;
		retval = __get_user(scull_qset, (int __user *)arg);
		if (retval == 0)
			retval = put_user(tmp, (int __user *)arg);
		break;

	  case SCULL_IOCHQSET:
	  	printk(KERN_DEBUG "lprf: ioctl case SCULL_IOCHQSET\n");	  	  	  
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		tmp = scull_qset;
		scull_qset = arg;
		return tmp;

		/*        
		 * The following two change the buffer size for scullpipe.
		 * The scullpipe device uses this same ioctl method, just to
		 * write less code. Actually, it's the same driver, isn't it?
		 */         

	  case SCULL_P_IOCTSIZE:
	    printk(KERN_DEBUG "lprf: switch case SCULL_P_IOCTSIZE of scull_p reached, this shouldn't be the case!%s:%i\n", __FILE__, __LINE__);
		//scull_p_buffer = arg;
		break;

	  case SCULL_P_IOCQSIZE:
	    printk(KERN_DEBUG "lprf: switch case SCULL_P_IOCQSIZE of scull_p reached, this shouldn't be the case!%s:%i\n", __FILE__, __LINE__);	  
		return 0;  //return scull_p_buffer;


	  default:  // redundant, as cmd was checked against MAXNR 
		return -ENOTTY;
	}
	printk(KERN_DEBUG "lprf: scull_ioctl - end: quantum=%d\n", scull_quantum);		
	return retval;

}


/*
 * Finally, the module stuff
 */
/*
 * The cleanup function is used to handle initialization failures as well.
 * Thefore, it must be careful to work correctly even if some of the items
 * have not been initialized
 */
void scull_cleanup_module(void)
{	
	int i;
	dev_t devno = MKDEV(scull_major, scull_minor);

	printk(KERN_DEBUG "lprf: scull_cleanup_module - start: %s:%i\n", __FILE__, __LINE__);
	// Get rid of our char dev entries 
	if (scull_devices) {
		for (i = 0; i < scull_nr_devs; i++) {
			scull_trim(scull_devices + i);
			cdev_del(&scull_devices[i].cdev);
		}
		kfree(scull_devices);
	}

	// cleanup_module is never called if registering failed 
	unregister_chrdev_region(devno, scull_nr_devs);

	printk(KERN_DEBUG "lprf: scull_cleanup_module - end: %s:%i\n", __FILE__, __LINE__);
}


/*
 * Set up the char_dev structure for this device.
 */
static void scull_setup_cdev(struct scull_dev *dev, int index)
{
	int err, devno = MKDEV(scull_major, scull_minor + index);

	printk(KERN_DEBUG "lprf: scull_setup_cdev index %i - start: %s:%i\n", index, __FILE__, __LINE__);    
	cdev_init(&dev->cdev, &scull_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &scull_fops;
	err = cdev_add (&dev->cdev, devno, 1);
	// Fail gracefully if need be 
	if (err)
		printk(KERN_NOTICE "Error %d adding scull%d", err, index);
	printk(KERN_DEBUG "lprf: scull_setup_cdev - end: %s:%i\n", __FILE__, __LINE__);		
}


int scull_init_module(void)
{
	int result, i;
	dev_t dev = 0;
/*
 * Get a range of minor numbers to work with, asking for a dynamic
 * major unless directed otherwise at load time.
 */
	if (scull_major) {
		dev = MKDEV(scull_major, scull_minor);
		result = register_chrdev_region(dev, scull_nr_devs, "lprf");             //<-- name of entry into /proc/devices
	} else {
		result = alloc_chrdev_region(&dev, scull_minor, scull_nr_devs, "lprf");  //<-- name of entry into /proc/devices
		scull_major = MAJOR(dev);
	}
	if (result < 0) {
		printk(KERN_WARNING "lprf: can't get major %d\n", scull_major);
		return result;
	}

	/*     
	 * allocate the devices -- we can't have them static, as the number
	 * can be specified at load time
	 */
	scull_devices = kmalloc(scull_nr_devs * sizeof(struct scull_dev), GFP_KERNEL);
	if (!scull_devices) {
		result = -ENOMEM;
		goto fail;  // Make this more graceful 
	}
	memset(scull_devices, 0, scull_nr_devs * sizeof(struct scull_dev));

        // Initialize each device. 
	for (i = 0; i < scull_nr_devs; i++) {
		scull_devices[i].quantum = scull_quantum;
		scull_devices[i].qset = scull_qset;
		mutex_init(&scull_devices[i].scull_mutex);
		scull_setup_cdev(&scull_devices[i], i);
		printk(KERN_DEBUG "lprf: scull_init_module registered lprf%i  quantum=%d  qset=%d\n", i, scull_quantum, scull_qset);
	}

        // At this point call the init function for any friend device 
	dev = MKDEV(scull_major, scull_minor + scull_nr_devs);

	return 0; // succeed 

  fail:
	scull_cleanup_module();
	printk(KERN_DEBUG "lprf: scull_init_module - fail: %s:%i\n", __FILE__, __LINE__);	
	return result;
}


struct file_operations scull_fops = {
	.owner =             THIS_MODULE,
	.llseek =            scull_llseek,	
	.read =              scull_read,
	.write =             scull_write,
	.unlocked_ioctl =    scull_ioctl,
	.open =              scull_open,
	.release =           scull_release,
};

//this is the interface to upper SPI layers!
static const struct regmap_config lprf_regmap_spi_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.write_flag_mask = 0xC0,
	.read_flag_mask = 0x80,
	.max_register = 0xF3, 	// specifies the maximum valid register index (optional)
	.cache_type = REGCACHE_RBTREE,	
	.writeable_reg = lprf_reg_writeable,
	.readable_reg = lprf_reg_readable,
	.volatile_reg = lprf_reg_volatile,
	.precious_reg = lprf_reg_precious,
};

//this is the interface to upper ieee802154 layers!
static const struct ieee802154_ops lprf_ops = {
	.owner = THIS_MODULE,
	.xmit_async = lprf_xmit,
	.ed = lprf_ed,
	.set_channel = lprf_channel,
	.start = lprf_start,
	.stop = lprf_stop,
	.set_hw_addr_filt = lprf_set_hw_addr_filt,
	.set_txpower = lprf_set_txpower,
	.set_lbt = lprf_set_lbt,
	.set_cca_mode = lprf_set_cca_mode,
	.set_cca_ed_level = lprf_set_cca_ed_level,
	.set_csma_params = lprf_set_csma_params,
	.set_frame_retries = lprf_set_frame_retries,
	.set_promiscuous_mode = lprf_set_promiscuous_mode,
};



//interface to device tree file (bcm2835-rpi-b-plus.dts);
// !modify! 
static const struct of_device_id lprf_of_match[] = {    // part of struct spi_driver lprf_driver
	{ .compatible = "atmel,at86rf233", },    
	{ .compatible = "atmel,at86rf212", },
	{ .compatible = "ias,lprf", },
	{ },
};
MODULE_DEVICE_TABLE(of, lprf_of_match);   		//second argument is name of struct of_device_id 

static const struct spi_device_id lprf_device_id[] = {  // part of struct spi_driver lprf_driver
	{ .name = "lprf", },
	{ .name = "at86rf231", },
	{ .name = "at86rf233", },
	{ .name = "at86rf212", },
	{ },
};
MODULE_DEVICE_TABLE(spi, lprf_device_id);

static struct spi_driver lprf_driver = {    //this is (probably) spi equivalent of platform_driver
	.id_table = lprf_device_id,
	.driver = {
		.of_match_table = of_match_ptr(lprf_of_match),
		.name	= "lprf",
		.owner	= THIS_MODULE,
	},
	.probe      = lprf_probe,
	.remove     = lprf_remove,
};

module_spi_driver(lprf_driver);    //this registers "lprf_driver" as an SPI driver

MODULE_DESCRIPTION("lprf Transceiver Driver");
MODULE_LICENSE("GPL v2");

//for dynamic debugging, read https://www.kernel.org/doc/Documentation/dynamic-debug-howto.txt & https://lwn.net/Articles/434833/
