/*
 *  linux/drivers/char/8250.c
 *
 *  Driver for 8250/16550-type serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *  $Id: 8250.c,v 1.90 2002/07/28 10:03:27 rmk Exp $
 *
 * A note about mapbase / membase
 *
 *  mapbase is the physical address of the IO port.  Currently, we don't
 *  support this very well, and it may well be dropped from this driver
 *  in future.  As such, mapbase should be NULL.
 *
 *  membase is an 'ioremapped' cookie.  This is compatible with the old
 *  serial.c driver, and is currently the preferred form.
 */
/*
 * Serial Interface for Modem Driver
 *
 * This module is adapted to be a serial interface driver for modem
 * controller drivers.  It provides a generic serial driver like
 * interface to the operating system.  It can work in conjunction
 * with a modem controller driver which supports the required
 * interface. The modem controller driver is expected to perform
 * core modem functions and interact with the modem hardware.
 * - design done by Soumyendu Sarkar of Agere Systems.
 *
 * Adapted to work with Agere Soft Modem driver module by
 * Soumyendu Sarkar of Agere Systems on Dec 2002.
 *
 * Copyright (C) 2002, 2003, 2004, 2005 Agere Systems Inc.
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version. This program
 * is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details. You should have received a copy of the
 * GNU General Public License along with this program; if not, write
 * to the Free Software Foundation, Inc., 675 Mass Ave, Cambridge,
 * MA 02139, USA.
 *
 * Migrated base code from serial.c for Kernel 2.4.x to the UART specific
 * 8250.c for Kernel 2.6.x by Soumyendu Sarkar on March 01, 2004. The 
 * serial_core.c for Kernel 2.6.x does not directly support UART specific
 * interface which is exposed by the Agere Modem Controller module.
 *
 *   Soumyendu Sarkar       11/16/2005    Adapted to kernel 2.6.14 with PCI PP modem support
 */

#define LUCENT_MODEM // modifications to the 8250.c code for Lucent controllerless modem 
#define AGERE_SOFT_MODEM // additional modifications for Agere Softmodem to the 8250.c code for Lucent controllerless modem
#ifndef __KERNEL__
#define __KERNEL__
#endif

static char *serialif_version = "2.1.80.0";
static char *serialif_revdate = "2007-10-01";
static char *serialif_name = "Agere Modem Interface driver";
#include <linux/version.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/irq.h>

#if defined(CONFIG_SERIAL_8250_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/serial_core.h>
#include "8250.h"

#ifndef UPF_RESOURCES
#define UPF_RESOURCES (1 << 30)
#endif
#ifndef MCA_bus
#define MCA_bus 0
#endif
/*
 * Configuration:
 *   share_irqs - whether we pass IRQF_SHARED to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
unsigned int share_irqs = SERIAL8250_SHARE_IRQS;

/*
 * Debugging.
 */
#ifdef DEBac 
#define DEBUG_AUTOCONF(fmt...)	printk(fmt)
#else
#define DEBUG_AUTOCONF(fmt...)	do { } while (0)
#endif

#ifdef DEBin
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

#define PASS_LIMIT	256

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)	((irq) != 0)

/*
 * This converts from our new CONFIG_ symbols to the symbols
 * that asm/serial.h expects.  You _NEED_ to comment out the
 * linux/config.h include contained inside asm/serial.h for
 * this to work.
 */
#undef CONFIG_SERIAL_MANY_PORTS
#undef CONFIG_SERIAL_DETECT_IRQ
#undef CONFIG_SERIAL_MULTIPORT
#undef CONFIG_HUB6

#ifdef CONFIG_SERIAL_8250_DETECT_IRQ
#define CONFIG_SERIAL_DETECT_IRQ 1
#endif
#ifdef CONFIG_SERIAL_8250_MULTIPORT
#define CONFIG_SERIAL_MULTIPORT 1
#endif
#ifdef CONFIG_SERIAL_8250_MANY_PORTS
#define CONFIG_SERIAL_MANY_PORTS 1
#endif

#ifdef CONFIG_SERIAL_SHARE_IRQ
#undef CONFIG_SERIAL_SHARE_IRQ
#endif
#define IRQ_T(state) ((state->flags & ASYNC_SHARE_IRQ) ? IRQF_SHARED : IRQF_DISABLED)
int int_hooked = 0;
int (*GetAgrModemInterface)(void *mdmdata) = NULL;
int (*SetAgrModemInterface)(int state) = NULL;
// extern unsigned int BaseAddress;
// extern byte Irq;

/*
 * HUB6 is always on.  This will be removed once the header
 * files have been cleaned.
 */
#define CONFIG_HUB6 1

#include <asm/serial.h>

static struct old_serial_port old_serial_port[] = {
	SERIAL_PORT_DFNS /* defined in asm/serial.h */
};

#define UART_NR	ARRAY_SIZE(old_serial_port)

#if defined(NDZ)
#if defined(CONFIG_SERIAL_8250_RSA) && defined(MODULE)

#define PORT_RSA_MAX 4
static int probe_rsa[PORT_RSA_MAX];
static int force_rsa[PORT_RSA_MAX];
#endif /* CONFIG_SERIAL_8250_RSA  */
#endif

struct uart_8250_port {
	struct uart_port	port;
	struct timer_list	timer;		/* "no irq" timer */
	struct list_head	list;		/* ports on this IRQ */
	unsigned int		capabilities;	/* port capabilities */
	unsigned short		rev;
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */
	unsigned char		lsr_break_flag;

	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
			unsigned int state, unsigned int old);
};

struct irq_info {
	spinlock_t		lock;
	struct list_head	*head;
};

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
static const struct serial_uart_config uart_config[] = {
	{ "unknown",	1,	0 },
	{ "8250",	1,	0 },
	{ "16450",	1,	0 },
	{ "16550",	1,	0 },
	{ "AgereModem", 16, 0 },
	{ "Cirrus",	1, 	0 },
	{ "ST16650",	1,	UART_CLEAR_FIFO | UART_STARTECH },
	{ "ST16650V2",	32,	UART_CLEAR_FIFO | UART_USE_FIFO | UART_STARTECH },
	{ "TI16750",	64,	UART_CLEAR_FIFO | UART_USE_FIFO },
	{ "Startech",	1,	0 },
	{ "16C950/954",	128,	UART_CLEAR_FIFO | UART_USE_FIFO },
	{ "ST16654",	64,	UART_CLEAR_FIFO | UART_USE_FIFO | UART_STARTECH },
	{ "XR16850",	128,	UART_CLEAR_FIFO | UART_USE_FIFO | UART_STARTECH },
	{ "RSA",	2048,	UART_CLEAR_FIFO | UART_USE_FIFO },
	{ "NS16550A",	16,	UART_CLEAR_FIFO | UART_USE_FIFO | UART_NATSEMI }
};

// ensure that this remains after <linux/serial.h> for PORT_MAX def
typedef unsigned char byte ;
typedef unsigned short word ;
typedef unsigned long dword ;
typedef int BOOL;

struct ltmodem_ops
{
	int (*detect_modem)(void *pltmodem_res);
	void (*init_modem)(void);
	int (*PortOpen)(void);
	void (*PortClose)(void);
	int (*read_vuart_register)(int offset);
	void (*write_vuart_register)(int offset, int value);
	int (*app_ioctl_handler)(unsigned int cmd, unsigned long arg);
	byte(*dsp_isr)(void);
	word (*read_buffer)( byte *data, word size );
};

struct ltmodem_res
{
	word BaseAddress;
	word BaseAddress2;
	byte Irq;
};

#define IOCTL_MODEM_APP_1	_IOR(62,41,char *)
#define IOCTL_MODEM_APP_2	_IOR(62,42,char *)
#define IOCTL_MODEM_APP_3	_IOR(62,43,char *)
#define IOCTL_MODEM_APP_4	_IOR(62,44,char *)
#define IOCTL_MODEM_APP_5	_IOR(62,45,char *)
#define IOCTL_MODEM_APP_6	_IOR(62,46,char *)
#define IOCTL_MODEM_APP_7	_IOR(62,47,char *)
#define IOCTL_MODEM_APP_8	_IOR(62,48,char *)
/* for call progress */
#define IOCTL_MODEM_APP_GET_CALL_PROGRESS_AUDIO  _IOR(62,49,char *)  
#define IOCTL_MODEM_APP_PASS_PID_TO_DRIVER	 _IOR(62,50,char *)

#ifdef PORT_MAX_8250
#undef PORT_MAX_8250
#endif

#define PORT_LTMODEM      PORT_16550A
#define PORT_LT_MODEM_IF  3
#define PORT_MAX_8250     15

#ifdef CONFIG_SERIAL_8250_CONSOLE
#undef CONFIG_SERIAL_8250_CONSOLE
#endif

struct ltmodem_ops lt_modem_ops;
struct ltmodem_res lt_modem_res;
static void agr_rs_interrupt (unsigned long);
static int intf_flag = 0;
static int uart_flag = 0;
static int tx_empty_flag = 0;
static struct uart_8250_port serial8250_ports[UART_NR];
//bala InterModule Fix
unsigned int start_int=0;
struct timer_list serial_timer;

#define _INLINE_
static _INLINE_ unsigned int serial_in(struct uart_8250_port *up, int offset)
{
	
//	printk("Entering proprietary code: Offset is: %d...\n",offset);
	return(lt_modem_ops.read_vuart_register( offset ));   //only have support for single modem so we can just use offsets.
//	printk("Finshed proprietary code...\n");
}

	static _INLINE_ void
serial_out(struct uart_8250_port *up, int offset, int value)
{
	
	lt_modem_ops.write_vuart_register(offset, value);
}

/*
 * We used to support using pause I/O for certain machines.  We
 * haven't supported this for a while, but just in case it's badly
 * needed for certain old 386 machines, I've left these #define's
 * in....
 */
#define serial_inp(up, offset)		serial_in(up, offset)
#define serial_outp(up, offset, value)	serial_out(up, offset, value)


/*
 * For the 16C950
 */
static void serial_icr_write(struct uart_8250_port *up, int offset, int value)
{
	//printk("%s offset %d value 0x%02x\n",__FUNCTION__,offset,value);
	serial_out(up, UART_SCR, offset);
	serial_out(up, UART_ICR, value);
}

#if defined(NDZ)
static unsigned int serial_icr_read(struct uart_8250_port *up, int offset)
{
	unsigned int value;

	
	serial_icr_write(up, UART_ACR, up->acr | UART_ACR_ICRRD);
	serial_out(up, UART_SCR, offset);
	value = serial_in(up, UART_ICR);
	serial_icr_write(up, UART_ACR, up->acr);

	//printk("%s offset %d value 0x%02x\n",__FUNCTION__,offset,value);

	return value;
}
#endif

#ifdef CONFIG_SERIAL_8250_RSA
/*
 * Attempts to turn on the RSA FIFO.  Returns zero on failure.
 * We set the port uart clock rate if we succeed.
 */
static int __enable_rsa(struct uart_8250_port *up)
{
	unsigned char mode;
	int result;

	
	mode = serial_inp(up, UART_RSA_MSR);
	result = mode & UART_RSA_MSR_FIFO;

	if (!result) {
		serial_outp(up, UART_RSA_MSR, mode | UART_RSA_MSR_FIFO);
		mode = serial_inp(up, UART_RSA_MSR);
		result = mode & UART_RSA_MSR_FIFO;
	}

	if (result)
		up->port.uartclk = SERIAL_RSA_BAUD_BASE * 16;

	return result;
}

static void enable_rsa(struct uart_8250_port *up)
{
	
	if (up->port.type == PORT_RSA) {
		if (up->port.uartclk != SERIAL_RSA_BAUD_BASE * 16) {
			spin_lock_irq(&up->port.lock);
			__enable_rsa(up);
			spin_unlock_irq(&up->port.lock);
		}
		if (up->port.uartclk == SERIAL_RSA_BAUD_BASE * 16)
			serial_outp(up, UART_RSA_FRR, 0);
	}
}

/*
 * Attempts to turn off the RSA FIFO.  Returns zero on failure.
 * It is unknown why interrupts were disabled in here.  However,
 * the caller is expected to preserve this behaviour by grabbing
 * the spinlock before calling this function.
 */
static void disable_rsa(struct uart_8250_port *up)
{
	unsigned char mode;
	int result;

	
	if (up->port.type == PORT_RSA &&
			up->port.uartclk == SERIAL_RSA_BAUD_BASE * 16) {
		spin_lock_irq(&up->port.lock);

		mode = serial_inp(up, UART_RSA_MSR);
		result = !(mode & UART_RSA_MSR_FIFO);

		if (!result) {
			serial_outp(up, UART_RSA_MSR, mode & ~UART_RSA_MSR_FIFO);
			mode = serial_inp(up, UART_RSA_MSR);
			result = !(mode & UART_RSA_MSR_FIFO);
		}

		if (result)
			up->port.uartclk = SERIAL_RSA_BAUD_BASE_LO * 16;
		spin_unlock_irq(&up->port.lock);
	}
}
#endif /* CONFIG_SERIAL_8250_RSA */

#if defined(NDZ)
/*
 * This is a quickie test to see how big the FIFO is.
 * It doesn't work at all the time, more's the pity.
 */
static int size_fifo(struct uart_8250_port *up)
{
	unsigned char old_fcr, old_mcr, old_dll, old_dlm;
	int count;

	
	old_fcr = serial_inp(up, UART_FCR);
	old_mcr = serial_inp(up, UART_MCR);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_outp(up, UART_MCR, UART_MCR_LOOP);
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	old_dll = serial_inp(up, UART_DLL);
	old_dlm = serial_inp(up, UART_DLM);
	serial_outp(up, UART_DLL, 0x01);
	serial_outp(up, UART_DLM, 0x00);
	serial_outp(up, UART_LCR, 0x03);
	for (count = 0; count < 256; count++)
		serial_outp(up, UART_TX, count);
	mdelay(20);/* FIXME - schedule_timeout */
	for (count = 0; (serial_inp(up, UART_LSR) & UART_LSR_DR) &&
			(count < 256); count++)
		serial_inp(up, UART_RX);
	serial_outp(up, UART_FCR, old_fcr);
	serial_outp(up, UART_MCR, old_mcr);
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	serial_outp(up, UART_DLL, old_dll);
	serial_outp(up, UART_DLM, old_dlm);

	return count;
}
#endif

#if defined(NDZ)
/*
 * This is a helper routine to autodetect StarTech/Exar/Oxsemi UART's.
 * When this function is called we know it is at least a StarTech
 * 16650 V2, but it might be one of several StarTech UARTs, or one of
 * its clones.  (We treat the broken original StarTech 16650 V1 as a
 * 16550, and why not?  Startech doesn't seem to even acknowledge its
 * existence.)
 * 
 * What evil have men's minds wrought...
 */
static void autoconfig_has_efr(struct uart_8250_port *up)
{
	unsigned char id1, id2, id3, rev, saved_dll, saved_dlm;

	
	/*
	 * First we check to see if it's an Oxford Semiconductor UART.
	 *
	 * If we have to do this here because some non-National
	 * Semiconductor clone chips lock up if you try writing to the
	 * LSR register (which serial_icr_read does)
	 */

	/*
	 * Check for Oxford Semiconductor 16C950.
	 *
	 * EFR [4] must be set else this test fails.
	 *
	 * This shouldn't be necessary, but Mike Hudson (Exoray@isys.ca)
	 * claims that it's needed for 952 dual UART's (which are not
	 * recommended for new designs).
	 */
	up->acr = 0;
	serial_out(up, UART_LCR, 0xBF);
	serial_out(up, UART_EFR, UART_EFR_ECB);
	serial_out(up, UART_LCR, 0x00);
	id1 = serial_icr_read(up, UART_ID1);
	id2 = serial_icr_read(up, UART_ID2);
	id3 = serial_icr_read(up, UART_ID3);
	rev = serial_icr_read(up, UART_REV);

	DEBUG_AUTOCONF("950id=%02x:%02x:%02x:%02x ", id1, id2, id3, rev);

	if (id1 == 0x16 && id2 == 0xC9 &&
			(id3 == 0x50 || id3 == 0x52 || id3 == 0x54)) {
		up->port.type = PORT_16C950;
		up->rev = rev | (id3 << 8);
		return;
	}

	/*
	 * We check for a XR16C850 by setting DLL and DLM to 0, and then
	 * reading back DLL and DLM.  The chip type depends on the DLM
	 * value read back:
	 *  0x10 - XR16C850 and the DLL contains the chip revision.
	 *  0x12 - XR16C2850.
	 *  0x14 - XR16C854.
	 */
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	saved_dll = serial_inp(up, UART_DLL);
	saved_dlm = serial_inp(up, UART_DLM);
	serial_outp(up, UART_DLL, 0);
	serial_outp(up, UART_DLM, 0);
	id2 = serial_inp(up, UART_DLL);
	id1 = serial_inp(up, UART_DLM);
	serial_outp(up, UART_DLL, saved_dll);
	serial_outp(up, UART_DLM, saved_dlm);

	DEBUG_AUTOCONF("850id=%02x:%02x ", id1, id2);

	if (id1 == 0x10 || id1 == 0x12 || id1 == 0x14) {
		if (id1 == 0x10)
			up->rev = id2;
		up->port.type = PORT_16850;
		return;
	}

	/*
	 * It wasn't an XR16C850.
	 *
	 * We distinguish between the '654 and the '650 by counting
	 * how many bytes are in the FIFO.  I'm using this for now,
	 * since that's the technique that was sent to me in the
	 * serial driver update, but I'm not convinced this works.
	 * I've had problems doing this in the past.  -TYT
	 */
	if (size_fifo(up) == 64)
		up->port.type = PORT_16654;
	else
		up->port.type = PORT_16650V2;
}
#endif

#if defined(NDZ)
/*
 * We detected a chip without a FIFO.  Only two fall into
 * this category - the original 8250 and the 16450.  The
 * 16450 has a scratch register (accessible with LCR=0)
 */
static void autoconfig_8250(struct uart_8250_port *up)
{
	unsigned char scratch, status1, status2;

	
	up->port.type = PORT_8250;

	scratch = serial_in(up, UART_SCR);
	serial_outp(up, UART_SCR, 0xa5);
	status1 = serial_in(up, UART_SCR);
	serial_outp(up, UART_SCR, 0x5a);
	status2 = serial_in(up, UART_SCR);
	serial_outp(up, UART_SCR, scratch);

	if (status1 == 0xa5 && status2 == 0x5a)
		up->port.type = PORT_16450;
}

/*
 * We know that the chip has FIFOs.  Does it have an EFR?  The
 * EFR is located in the same register position as the IIR and
 * we know the top two bits of the IIR are currently set.  The
 * EFR should contain zero.  Try to read the EFR.
 */
static void autoconfig_16550a(struct uart_8250_port *up)
{
	unsigned char status1, status2;

	
	up->port.type = PORT_16550A;

	/*
	 * Check for presence of the EFR when DLAB is set.
	 * Only ST16C650V1 UARTs pass this test.
	 */
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	if (serial_in(up, UART_EFR) == 0) {
		DEBUG_AUTOCONF("EFRv1 ");
		up->port.type = PORT_16650;
		return;
	}

	/*
	 * Maybe it requires 0xbf to be written to the LCR.
	 * (other ST16C650V2 UARTs, TI16C752A, etc)
	 */
	serial_outp(up, UART_LCR, 0xBF);
	if (serial_in(up, UART_EFR) == 0) {
		DEBUG_AUTOCONF("EFRv2 ");
		autoconfig_has_efr(up);
		return;
	}

	/*
	 * Check for a National Semiconductor SuperIO chip.
	 * Attempt to switch to bank 2, read the value of the LOOP bit
	 * from EXCR1. Switch back to bank 0, change it in MCR. Then
	 * switch back to bank 2, read it from EXCR1 again and check
	 * it's changed. If so, set baud_base in EXCR2 to 921600.
	 */
	serial_outp(up, UART_LCR, 0);
	status1 = serial_in(up, UART_MCR);
	serial_outp(up, UART_LCR, 0xE0);
	status2 = serial_in(up, 0x02); /* EXCR1 */

	if (!((status2 ^ status1) & UART_MCR_LOOP)) {
		serial_outp(up, UART_LCR, 0);
		serial_outp(up, UART_MCR, status1 ^ UART_MCR_LOOP);
		serial_outp(up, UART_LCR, 0xE0);
		status2 = serial_in(up, 0x02); /* EXCR1 */
		serial_outp(up, UART_LCR, 0);
		serial_outp(up, UART_MCR, status1);

		if ((status2 ^ status1) & UART_MCR_LOOP) {
			serial_outp(up, UART_LCR, 0xE0);
			status1 = serial_in(up, 0x04); /* EXCR1 */
			status1 &= ~0xB0; /* Disable LOCK, mask out PRESL[01] */
			status1 |= 0x10;  /* 1.625 divisor for baud_base --> 921600 */
			serial_outp(up, 0x04, status1);
			serial_outp(up, UART_LCR, 0);

			up->port.type = PORT_NS16550A;
			up->port.uartclk = 921600*16;
			return;
		}
	}

	/*
	 * No EFR.  Try to detect a TI16750, which only sets bit 5 of
	 * the IIR when 64 byte FIFO mode is enabled when DLAB is set.
	 * Try setting it with and without DLAB set.  Cheap clones
	 * set bit 5 without DLAB set.
	 */
	serial_outp(up, UART_LCR, 0);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR7_64BYTE);
	status1 = serial_in(up, UART_IIR) >> 5;
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR7_64BYTE);
	status2 = serial_in(up, UART_IIR) >> 5;
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);

	DEBUG_AUTOCONF("iir1=%d iir2=%d ", status1, status2);

	if (status1 == 6 && status2 == 7) {
		up->port.type = PORT_16750;
		return;
	}
}
#endif

/*
 * This routine is called by rs_init() to initialize a specific serial
 * port.  It determines what type of UART chip this serial port is
 * using: 8250, 16450, 16550, 16550A.  The important question is
 * whether or not this UART is a 16550A or not, since this will
 * determine whether or not we can use its FIFO features or not.
 */
static void autoconfig(struct uart_8250_port *up, unsigned int probeflags)
{
	printk("%s: probeflags 0x%04x lt_modem_res.BaseAddress 0x%04x\n",__FUNCTION__,probeflags,lt_modem_res.BaseAddress);
	up->port.type     = PORT_16550A;
	up->port.iobase   = lt_modem_res.BaseAddress;
	up->port.uartclk  = BASE_BAUD * 16;
	up->port.iotype   = UPIO_PORT;
	up->port.fifosize = 64;
	up->port.flags    = 0;
	return;
}

static void autoconfig_irq(struct uart_8250_port *up)
{
	unsigned char save_mcr, save_ier;
	unsigned char save_ICP = 0;
	unsigned int ICP = 0;
	unsigned long irqs;
	int irq;

	
	if (up->port.flags & UPF_FOURPORT) {
		ICP = (up->port.iobase & 0xfe0) | 0x1f;
		save_ICP = inb_p(ICP);
		outb_p(0x80, ICP);
		(void) inb_p(ICP);
	}

	/* forget possible initially masked and pending IRQ */
	probe_irq_off(probe_irq_on());
	save_mcr = serial_inp(up, UART_MCR);
	save_ier = serial_inp(up, UART_IER);
	serial_outp(up, UART_MCR, UART_MCR_OUT1 | UART_MCR_OUT2);

	irqs = probe_irq_on();
	serial_outp(up, UART_MCR, 0);
	udelay (10);
	if (up->port.flags & UPF_FOURPORT)  {
		serial_outp(up, UART_MCR,
				UART_MCR_DTR | UART_MCR_RTS);
	} else {
		serial_outp(up, UART_MCR,
				UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2);
	}
	serial_outp(up, UART_IER, 0x0f);	/* enable all intrs */
	(void)serial_inp(up, UART_LSR);
	(void)serial_inp(up, UART_RX);
	(void)serial_inp(up, UART_IIR);
	(void)serial_inp(up, UART_MSR);
	serial_outp(up, UART_TX, 0xFF);
	udelay (20);
	irq = probe_irq_off(irqs);

	serial_outp(up, UART_MCR, save_mcr);
	serial_outp(up, UART_IER, save_ier);

	if (up->port.flags & UPF_FOURPORT)
		outb_p(save_ICP, ICP);

	up->port.irq = (irq > 0) ? irq : 0;
}

static void serial8250_stop_tx(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;

	
	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
	if (up->port.type == PORT_16C950) {
		up->acr |= UART_ACR_TXDIS;
		serial_icr_write(up, UART_ACR, up->acr);
	}
}

static void serial8250_start_tx(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;

	
	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
	/*
	 * We only do this from uart_start
	 */
	if (up->port.type == PORT_16C950) {
		up->acr &= ~UART_ACR_TXDIS;
		serial_icr_write(up, UART_ACR, up->acr);
	}
}

static void serial8250_stop_rx(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;

	
	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
}

static void serial8250_enable_ms(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;

	
	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}
//bala-dbg
	static _INLINE_ void
receive_chars(struct uart_8250_port *up, int *status, struct pt_regs *regs)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION (2, 6, 32)
	struct tty_struct *tty = up->port.state->port.tty;
#else
	struct tty_struct *tty = up->port.info->port.tty;
#endif
	unsigned char ch, lsr = *status;
	int max_count = 256;
	char flag;

	
	do {
		ch = serial_inp(up, UART_RX);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

#ifdef CONFIG_SERIAL_8250_CONSOLE
		/*
		 * Recover the break flag from console xmit
		 */
		if (up->port.line == up->port.cons->index) {
			lsr |= up->lsr_break_flag;
			up->lsr_break_flag = 0;
		}
#endif

		if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE |
						UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				up->port.icount.parity++;
			else if (lsr & UART_LSR_FE)
				up->port.icount.frame++;
			if (lsr & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= up->port.read_status_mask;

			if (lsr & UART_LSR_BI) {
				DEBUG_INTR("handling break....");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);

ignore_char:
		lsr = serial_inp(up, UART_LSR);
	} while ((lsr & UART_LSR_DR) && (max_count-- > 0));
	//bala-dbg
	//spin_lock(&up->port.lock);
	tty_flip_buffer_push(tty);
	//spin_unlock(&up->port.lock);
	*status = lsr;
}


static _INLINE_ void transmit_chars(struct uart_8250_port *up)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION (2, 6, 32)
	struct circ_buf *xmit = &up->port.state->xmit;
#else
	struct circ_buf *xmit = &up->port.info->xmit;
#endif
	int count;

	
	if (up->port.x_char) {
		serial_outp(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serial8250_stop_tx(&up->port);
		return;
	}

	count = up->port.fifosize;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	DEBUG_INTR("THRE...");

	if (uart_circ_empty(xmit))
		serial8250_stop_tx(&up->port);
}

static _INLINE_ void check_modem_status(struct uart_8250_port *up)
{
	int status;

	status = serial_in(up, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		up->port.icount.rng++;
	if (status & UART_MSR_DDSR)
		up->port.icount.dsr++;
	if (status & UART_MSR_DDCD)
		uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

#if LINUX_VERSION_CODE >= KERNEL_VERSION (2, 6, 32)
	wake_up_interruptible(&up->port.state->port.delta_msr_wait);
#else
	wake_up_interruptible(&up->port.info->delta_msr_wait);
#endif
}

/*
 * This handles the interrupt from one port.
 */
	static inline void
serial8250_handle_port(struct uart_8250_port *up, struct pt_regs *regs)
{
	unsigned int status;

	
	spin_lock(&up->port.lock);

	status = serial_inp(up, UART_LSR);

	DEBUG_INTR("status = %x...\n", status);

	if (status & UART_LSR_DR)
		receive_chars(up, &status, regs);
	check_modem_status(up);
	if (status & UART_LSR_THRE)
		transmit_chars(up);

	spin_unlock (&up->port.lock);
}

/*
 * This is the serial driver's interrupt routine.
 *
 * Arjan thinks the old way was overly complex, so it got simplified.
 * Alan disagrees, saying that need the complexity to handle the weird
 * nature of ISA shared interrupts.  (This is a special exception.)
 *
 * In order to handle ISA shared interrupts properly, we need to check
 * that all ports have been serviced, and therefore the ISA interrupt
 * line has been de-asserted.
 *
 * This means we need to loop through all ports. checking that they
 * don't have an interrupt pending.
 */
static irqreturn_t serial8250_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uart_8250_port *up = &serial8250_ports[PORT_LT_MODEM_IF];
	unsigned int iir;
	
	iir = serial_in(up, UART_IIR);
	if (!(iir & UART_IIR_NO_INT))
		serial8250_handle_port(up, regs);
	/* FIXME! Was it really ours? */
	return IRQ_HANDLED;
}

static void agr_rs_interrupt (unsigned long z)
{
//	printk("%s lt_modem_res.Irq %d\n",__FUNCTION__,lt_modem_res.Irq);
	serial8250_interrupt(lt_modem_res.Irq,NULL,NULL);
	if(start_int)
		add_timer(&serial_timer);
}

#if defined(LUCENT_MODEM) && !defined(AGERE_SOFT_MODEM)
/* DSP interrupt service routine */
static irqreturn_t VMODEM_Hw_Int_Proc (int irq, void *dev_id, struct pt_regs * regs)
{
	unsigned long flags;
	

	save_flags(flags);
	lt_modem_ops.dsp_isr();
	restore_flags(flags);
	return IRQ_HANDLED;
}
#endif

static void serial_unlink_irq_chain(struct uart_8250_port *up)
{
	//printk("dummy %s:\n",__FUNCTION__);
}

static unsigned int serial8250_tx_empty(struct uart_port *port)
{
	tx_empty_flag = 1;
	return 0;
}

static unsigned int serial8250_get_mctrl(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned char status;
	unsigned int ret = 0;

	
	//spin_lock_irqsave(&up->port.lock, flags);
	status = serial_in(up, UART_MSR);
	//spin_unlock_irqrestore(&up->port.lock, flags);

	if (tx_empty_flag == 1)  {
		tx_empty_flag = 0;
		ret = TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
	}
	else  {
		if (status & UART_MSR_DCD)
			ret |= TIOCM_CAR;
		if (status & UART_MSR_RI)
			ret |= TIOCM_RNG;
		if (status & UART_MSR_DSR)
			ret |= TIOCM_DSR;
		if (status & UART_MSR_CTS)
			ret |= TIOCM_CTS;
	}
	return ret;
}

static void serial8250_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned char mcr = 0;

	
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force;

	serial_out(up, UART_MCR, mcr);
}

static void serial8250_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;

	
	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int serial8250_startup(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;
	
	if (intf_flag == 0)
	{
		//  if (SetAgrModemInterface != NULL)   //bala-dbg
		//		  SetAgrModemInterface(1);
		intf_flag = 1;
		//bala InterModule Fix
		//This code has been separated from the modem module to avoid inter_module_* funtions. Now __symbol_get/__symbol_put funtions are used instead of inter_module_* funtiond.
		start_int=1;
		init_timer(&serial_timer);
		serial_timer.expires=jiffies+1;
		serial_timer.function=agr_rs_interrupt;
		serial_timer.data=0;
		add_timer(&serial_timer);
	}

	if (!int_hooked)
	{
		int_hooked++;
	    lt_modem_ops.PortOpen();
	}

	up->capabilities = uart_config[up->port.type].flags;

	if (up->port.type == PORT_16C950) {
		/* Wake up and initialize UART */
		up->acr = 0;
		serial_outp(up, UART_LCR, 0xBF);
		serial_outp(up, UART_EFR, UART_EFR_ECB);
		serial_outp(up, UART_IER, 0);
		serial_outp(up, UART_LCR, 0);
		serial_icr_write(up, UART_CSR, 0); /* Reset the UART */
		serial_outp(up, UART_LCR, 0xBF);
		serial_outp(up, UART_EFR, UART_EFR_ECB);
		serial_outp(up, UART_LCR, 0);
	}

#ifdef CONFIG_SERIAL_8250_RSA
	/*
	 * If this is an RSA port, see if we can kick it up to the
	 * higher speed clock.
	 */
	enable_rsa(up);
#endif

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reeanbled in set_termios())
	 */
	if (up->capabilities & UART_CLEAR_FIFO) {
		serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO |
				UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_outp(up, UART_FCR, 0);
	}

	/*
	 * Clear the interrupt registers.
	 */
	// printk("Registers: %d - %d - %d - %d\n",UART_LSR,UART_RX,UART_IIR,UART_MSR);
	(void) serial_inp(up, UART_LSR);
	(void) serial_inp(up, UART_RX);
	(void) serial_inp(up, UART_IIR);
	(void) serial_inp(up, UART_MSR);

	/*
	 * At this point, there's no way the LSR could still be 0xff;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if (!(up->port.flags & UPF_BUGGY_UART) &&
			(serial_inp(up, UART_LSR) == 0xff)) {
		printk("ttyS%d: LSR safety check engaged!\n", up->port.line);
		return -ENODEV;
	}

	/*
	 * If the "interrupt" for this port doesn't correspond with any
	 * hardware interrupt, we use a timer-based system.  The original
	 * driver used to do this with IRQ0.
	 */

	/*
	 * Now, initialize the UART
	 */
	serial_outp(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);
	if (up->port.flags & UPF_FOURPORT) {
		if (!is_real_interrupt(up->port.irq))
			up->port.mctrl |= TIOCM_OUT1;
	} else
		/*
		 * Most PC uarts need OUT2 raised to enable interrupts.
		 */
		if (is_real_interrupt(up->port.irq))
			up->port.mctrl |= TIOCM_OUT2;

	serial8250_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_outp(up, UART_IER, up->ier);

	if (up->port.flags & UPF_FOURPORT) {
		unsigned int icp;
		/*
		 * Enable interrupts on the AST Fourport board
		 */
		icp = (up->port.iobase & 0xfe0) | 0x01f;
		outb_p(0x80, icp);
		(void) inb_p(icp);
	}

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void) serial_inp(up, UART_LSR);
	(void) serial_inp(up, UART_RX);
	(void) serial_inp(up, UART_IIR);
	(void) serial_inp(up, UART_MSR);

	return 0;
}

static void serial8250_shutdown(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;
	start_int=0;

	
	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_outp(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	if (up->port.flags & UPF_FOURPORT) {
		/* reset interrupts on the AST Fourport board */
		inb((up->port.iobase & 0xfe0) | 0x1f);
		up->port.mctrl |= TIOCM_OUT1;
	} else
		up->port.mctrl &= ~TIOCM_OUT2;

	serial8250_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_inp(up, UART_LCR) & ~UART_LCR_SBC);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR |
			UART_FCR_CLEAR_XMIT);
	serial_outp(up, UART_FCR, 0);

#ifdef CONFIG_SERIAL_8250_RSA
	/*
	 * Reset the RSA board back to 115kbps compat mode.
	 */
	disable_rsa(up);
#endif

	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	(void) serial_in(up, UART_RX);

	lt_modem_ops.PortClose () ;
	if (intf_flag == 1)
		intf_flag = 0;

	serial_unlink_irq_chain(up);
	int_hooked--;
}

static unsigned int serial8250_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	
	/*
	 * Handle magic divisors for baud rates above baud_base on
	 * SMSC SuperIO chips.
	 */
	if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
			baud == (port->uartclk/4))
		quot = 0x8001;
	else if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
			baud == (port->uartclk/8))
		quot = 0x8002;
	else
		quot = uart_get_divisor(port, baud);

	return quot;
}

	static void
serial8250_set_termios(struct uart_port *port, struct ktermios *termios,
		struct ktermios *old)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	
	switch (termios->c_cflag & CSIZE) {
		case CS5:
			cval = 0x00;
			break;
		case CS6:
			cval = 0x01;
			break;
		case CS7:
			cval = 0x02;
			break;
		default:
		case CS8:
			cval = 0x03;
			break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= 0x04;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16); 
	quot = serial8250_get_divisor(port, baud);

	/*
	 * Work around a bug in the Oxford Semiconductor 952 rev B
	 * chip which causes it to seriously miscalculate baud rates
	 * when DLL is 0.
	 */
	if ((quot & 0xff) == 0 && up->port.type == PORT_16C950 &&
			up->rev == 0x5201)
		quot ++;

	if (up->capabilities & UART_USE_FIFO) {
		if (baud < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
#ifdef CONFIG_SERIAL_8250_RSA
		else if (up->port.type == PORT_RSA)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_14;
#endif
		else
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_8;
	}
	if (up->port.type == PORT_16750)
		fcr |= UART_FCR7_64BYTE;

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;

	serial_out(up, UART_IER, up->ier);

	if (up->capabilities & UART_STARTECH) {
		serial_outp(up, UART_LCR, 0xBF);
		serial_outp(up, UART_EFR,
				termios->c_cflag & CRTSCTS ? UART_EFR_CTS :0);
	}

	if (up->capabilities & UART_NATSEMI) {
		/* Switch to bank 2 not bank 1, to avoid resetting EXCR2 */
		serial_outp(up, UART_LCR, 0xe0);
	} else {
		serial_outp(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */
	}
	serial_outp(up, UART_DLL, quot & 0xff);		/* LS of divisor */
	serial_outp(up, UART_DLM, quot >> 8);		/* MS of divisor */
	if (up->port.type == PORT_16750)
		serial_outp(up, UART_FCR, fcr);		/* set fcr */
	serial_outp(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;					/* Save LCR */
	if (up->port.type != PORT_16750) {
		if (fcr & UART_FCR_ENABLE_FIFO) {
			/* emulated UARTs (Lucent Venus 167x) need two steps */
			serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
		}
		serial_outp(up, UART_FCR, fcr);		/* set fcr */
	}
	spin_unlock_irqrestore(&up->port.lock, flags);
}

	static void
serial8250_pm(struct uart_port *port, unsigned int state,
		unsigned int oldstate)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	
	if (state) {
		/* sleep */
		if (up->capabilities & UART_STARTECH) {
			/* Arrange to enter sleep mode */
			serial_outp(up, UART_LCR, 0xBF);
			serial_outp(up, UART_EFR, UART_EFR_ECB);
			serial_outp(up, UART_LCR, 0);
			serial_outp(up, UART_IER, UART_IERX_SLEEP);
			serial_outp(up, UART_LCR, 0xBF);
			serial_outp(up, UART_EFR, 0);
			serial_outp(up, UART_LCR, 0);
		}
		if (up->port.type == PORT_16750) {
			/* Arrange to enter sleep mode */
			serial_outp(up, UART_IER, UART_IERX_SLEEP);
		}

		if (up->pm)
			up->pm(port, state, oldstate);
	} else {
		/* wake */
		if (up->capabilities & UART_STARTECH) {
			/* Wake up UART */
			serial_outp(up, UART_LCR, 0xBF);
			serial_outp(up, UART_EFR, UART_EFR_ECB);
			/*
			 * Turn off LCR == 0xBF so we actually set the IER
			 * register on the XR16C850
			 */
			serial_outp(up, UART_LCR, 0);
			serial_outp(up, UART_IER, 0);
			/*
			 * Now reset LCR so we can turn off the ECB bit
			 */
			serial_outp(up, UART_LCR, 0xBF);
			serial_outp(up, UART_EFR, 0);
			/*
			 * For a XR16C850, we need to set the trigger levels
			 */
			if (up->port.type == PORT_16850) {
				unsigned char fctr;

				fctr = serial_inp(up, UART_FCTR) &
					~(UART_FCTR_RX | UART_FCTR_TX);
				serial_outp(up, UART_FCTR, fctr |
						UART_FCTR_TRGD |
						UART_FCTR_RX);
				serial_outp(up, UART_TRG, UART_TRG_96);
				serial_outp(up, UART_FCTR, fctr |
						UART_FCTR_TRGD |
						UART_FCTR_TX);
				serial_outp(up, UART_TRG, UART_TRG_96);
			}
			serial_outp(up, UART_LCR, 0);
		}

		if (up->port.type == PORT_16750) {
			/* Wake up UART */
			serial_outp(up, UART_IER, 0);
		}

		if (up->pm)
			up->pm(port, state, oldstate);
	}
}

/*
 * Resource handling.  This is complicated by the fact that resources
 * depend on the port type.  Maybe we should be claiming the standard
 * 8250 ports, and then trying to get other resources as necessary?
 */
	static int
serial8250_request_std_resource(struct uart_8250_port *up, struct resource **res)
{
	unsigned int size = 8 << up->port.regshift;
	int ret = 0;

	
	switch (up->port.iotype) {
		case SERIAL_IO_MEM:
			if (up->port.mapbase) {
				*res = request_mem_region(up->port.mapbase, size, "serial");
				if (!*res)
					ret = -EBUSY;
			}
			break;

		case SERIAL_IO_HUB6:
		case SERIAL_IO_PORT:
			*res = request_region(up->port.iobase, size, "serial");
			if (!*res)
				ret = -EBUSY;
			break;
	}
	return ret;
}

static void serial8250_release_port(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long start, offset = 0, size = 0;

	
	if (up->port.type == PORT_RSA) {
		offset = UART_RSA_BASE << up->port.regshift;
		size = 8;
	}

	size <<= up->port.regshift;

	switch (up->port.iotype) {
		case SERIAL_IO_MEM:
			if (up->port.mapbase) {
				/*
				 * Unmap the area.
				 */
				iounmap(up->port.membase);
				up->port.membase = NULL;

				start = up->port.mapbase;

				if (size)
					release_mem_region(start + offset, size);
				release_mem_region(start, 8 << up->port.regshift);
			}
			break;

		case SERIAL_IO_HUB6:
		case SERIAL_IO_PORT:
			break;

		default:
			break;
	}
}

static int serial8250_request_port(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	struct resource *res = NULL, *res_rsa = NULL;
	int ret = 0;


	

	ret = serial8250_request_std_resource(up, &res);

	/*
	 * If we have a mapbase, then request that as well.
	 */
	if (ret == 0 && up->port.flags & UPF_IOREMAP) {
		int size = res->end - res->start + 1;

		up->port.membase = ioremap(up->port.mapbase, size);
		if (!up->port.membase)
			ret = -ENOMEM;
	}

	if (ret < 0) {
		if (res_rsa)
			release_resource(res_rsa);
		if (res)
			release_resource(res);
	}
	return ret;
}

static void serial8250_config_port(struct uart_port *port, int flags)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	struct resource *res_std = NULL, *res_rsa = NULL;
	int probeflags = PROBE_ANY;
	//	int ret;

	
#if defined(NDZ)
#ifdef CONFIG_MCA
	/*
	 * Don't probe for MCA ports on non-MCA machines.
	 */
	if (up->port.flags & UPF_BOOT_ONLYMCA && !MCA_bus)
		return;
#endif
#endif

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */

	probeflags &= ~PROBE_RSA;

	if (flags & UART_CONFIG_TYPE)
		autoconfig(up, probeflags);
	if (up->port.type != PORT_UNKNOWN && flags & UART_CONFIG_IRQ)
		autoconfig_irq(up);

	/*
	 * If the port wasn't an RSA port, release the resource.
	 */
	if (up->port.type != PORT_RSA && res_rsa)
		release_resource(res_rsa);

	if (up->port.type == PORT_UNKNOWN && res_std)
		release_resource(res_std);
}

	static int
serial8250_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->irq >= NR_IRQS || ser->irq < 0 ||
			ser->baud_base < 9600 || ser->type < PORT_UNKNOWN ||
			ser->type > (PORT_MAX_8250 + 1) ||
			ser->type == PORT_CIRRUS ||
			ser->type == PORT_STARTECH)
		return -EINVAL;
	return 0;
}

	static const char *
serial8250_type(struct uart_port *port)
{
	int type = port->type;

	if (type >= ARRAY_SIZE(uart_config))
		type = 0;
	return uart_config[type].name;
}

	static
int	serial8250_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	
	switch (cmd) {
		case IOCTL_MODEM_APP_1:
		case IOCTL_MODEM_APP_2:
		case IOCTL_MODEM_APP_3:
		case IOCTL_MODEM_APP_4:
		case IOCTL_MODEM_APP_5:
		case IOCTL_MODEM_APP_6:
		case IOCTL_MODEM_APP_7:
		case IOCTL_MODEM_APP_8:
			/* for call progress */
		case IOCTL_MODEM_APP_GET_CALL_PROGRESS_AUDIO:  	
		case IOCTL_MODEM_APP_PASS_PID_TO_DRIVER:
			lt_modem_ops.app_ioctl_handler(cmd,arg);
			return 0;
	}
	return -ENOIOCTLCMD;
}

static struct uart_ops serial8250_pops = {
	.tx_empty	= serial8250_tx_empty,
	.set_mctrl	= serial8250_set_mctrl,
	.get_mctrl	= serial8250_get_mctrl,
	.stop_tx	= serial8250_stop_tx,
	.start_tx	= serial8250_start_tx,
	.stop_rx	= serial8250_stop_rx,
	.enable_ms	= serial8250_enable_ms,
	.break_ctl	= serial8250_break_ctl,
	.startup	= serial8250_startup,
	.shutdown	= serial8250_shutdown,
	.ioctl		= serial8250_ioctl,
	.set_termios	= serial8250_set_termios,
	.pm		= serial8250_pm,
	.type		= serial8250_type,
	.release_port	= serial8250_release_port,
	.request_port	= serial8250_request_port,
	.config_port	= serial8250_config_port,
	.verify_port	= serial8250_verify_port,
};


static int serial8250_register_ports(struct uart_driver *drv)
{

	struct uart_8250_port *up = &serial8250_ports[PORT_LT_MODEM_IF];
	int ret_val = 0;
	
	printk("%s: BaseAddress 0x%04x Irq %d \n",__FUNCTION__,lt_modem_res.BaseAddress,lt_modem_res.Irq);

	up->port.type     = PORT_LTMODEM;
	up->port.iobase   = lt_modem_res.BaseAddress;
	up->port.irq	    = irq_canonicalize(lt_modem_res.Irq);
	up->port.uartclk  = BASE_BAUD * 16;
	up->port.iotype   = UPIO_PORT;
	up->port.fifosize = uart_config[up->port.type].dfl_xmit_fifo_size;
	up->port.flags    = uart_config[up->port.type].flags;
	up->port.line     = PORT_LT_MODEM_IF;
	up->port.ops      = &serial8250_pops;

	/*
	 * ALPHA_KLUDGE_MCR needs to be killed.
	 */
	up->mcr_mask  = ~ALPHA_KLUDGE_MCR;
	up->mcr_force = ALPHA_KLUDGE_MCR;

	if ((ret_val = uart_add_one_port(drv, &up->port)) < 0)
	{
		printk(KERN_ERR"uart_add_one_port error code = %d\n", ret_val);
		printk(KERN_ERR"serial8250_register_ports unable to register port\n");
	}
	else
		uart_flag = 1;

	return (ret_val);
}

#ifdef CONFIG_SERIAL_8250_CONSOLE

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

/*
 *	Wait for transmitter & holding register to empty
 */
static inline void wait_for_xmitr(struct uart_8250_port *up)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & BOTH_EMPTY) != BOTH_EMPTY);

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		while (--tmout &&
				((serial_in(up, UART_MSR) & UART_MSR_CTS) == 0))
			udelay(1);
	}
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
	static void
serial8250_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_8250_port *up = &serial8250_ports[co->index];
	unsigned int ier;
	int i;

	/*
	 *	First save the UER then disable the interrupts
	 */
	ier = serial_in(up, UART_IER);
	serial_out(up, UART_IER, 0);

	/*
	 *	Now, do each character
	 */
	for (i = 0; i < count; i++, s++) {
		wait_for_xmitr(up);

		/*
		 *	Send the character out.
		 *	If a LF, also do CR...
		 */
		serial_out(up, UART_TX, *s);
		if (*s == 10) {
			wait_for_xmitr(up);
			serial_out(up, UART_TX, 13);
		}
	}

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up);
	serial_out(up, UART_IER, ier);
}

static int __init serial8250_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	port = &serial8250_ports[co->index].port;

	/*
	 * Temporary fix.
	 */
	spin_lock_init(&port->lock);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

extern struct uart_driver serial8250_reg;
static struct console serial8250_console = {
	.name		= "ttyS",
	.write		= serial8250_console_write,
	.device		= uart_console_device,
	.setup		= serial8250_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial8250_reg,
};

static int __init serial8250_console_init(void)
{
	serial8250_isa_init_ports();
	register_console(&serial8250_console);
	return 0;
}
console_initcall(serial8250_console_init);

#define SERIAL8250_CONSOLE	&serial8250_console
#else
#define SERIAL8250_CONSOLE	NULL
#endif

static struct uart_driver serial8250_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "agrserial",
	//bala-dbg
	//	.devfs_name		= "tts/AGS",
	.dev_name		= "ttyAGS",
	.major			= 62,
	.minor			= 64,
	.nr			= UART_NR,
	.cons			= NULL,
	.state  = NULL,
	.tty_driver = NULL
};

/*
 * This is for ISAPNP only.
 */
void serial8250_get_irq_map(unsigned int *map)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		if (serial8250_ports[i].port.type != PORT_UNKNOWN &&
				serial8250_ports[i].port.irq < 16)
			*map |= 1 << serial8250_ports[i].port.irq;
	}
}

/**
 *	serial8250_suspend_port - suspend one serial port
 *	@line: serial line number
 *
 *	Suspend one serial port.
 */
void serial8250_suspend_port(int line)
{
	uart_suspend_port(&serial8250_reg, &serial8250_ports[line].port);
}

/**
 *	serial8250_resume_port - resume one serial port
 *	@line: serial line number
 *
 *	Resume one serial port.
 */
void serial8250_resume_port(int line)
{
	uart_resume_port(&serial8250_reg, &serial8250_ports[line].port);
}

static int serial8250_init(void)
{
	int ret;
	

//	printk("%s serial8250_reg \n",__FUNCTION__);
	ret = uart_register_driver(&serial8250_reg);
	if (ret >= 0)  
	{
		ret = serial8250_register_ports(&serial8250_reg);
		if (ret < 0)  {
			printk(KERN_INFO "serial8250_init: serial8250_register_ports returns err code %d\n", ret);
			uart_unregister_driver(&serial8250_reg);
		}
	}
	else
		printk(KERN_INFO "serial8250_init: uart_register_driver returns err code %d\n", ret);

	return ret;
}

static void __exit serial8250_exit(void)
{
	printk(KERN_INFO "Unloading module %s version %s (%s)\n", serialif_name, serialif_version, serialif_revdate);
	if (uart_flag == 1)
	{
		uart_remove_one_port(&serial8250_reg, &serial8250_ports[PORT_LT_MODEM_IF].port);
		uart_flag = 0;
	}

	uart_unregister_driver(&serial8250_reg);
	__symbol_put("GetAgrModemInterface");
	SetAgrModemInterface = NULL;
	GetAgrModemInterface = NULL;
}

int __init agr_init_module(void)
{
	int ret_val;

	GetAgrModemInterface = (int (*)(void *mdmdata))__symbol_get("GetAgrModemInterface");

	if (GetAgrModemInterface != NULL)
	{
		GetAgrModemInterface(&lt_modem_ops);
		if (!(lt_modem_ops.detect_modem(&lt_modem_res)))
		{
			ret_val =  serial8250_init();
			if (ret_val < 0 )
				printk(KERN_ERR"agrserial - serial8250_init failed\n");
		}
		else
		{
			printk(KERN_ERR"agrserial - No device detected\n");
			ret_val = -ENODEV;
		}

		if (ret_val < 0 )
		{
			__symbol_put("GetAgrModemInterface");
			GetAgrModemInterface = NULL;
		}
		else
		{
			printk(KERN_ERR"agrserial - ret_val %d, call: lt_modem_ops.init_modem\n",ret_val);
			lt_modem_ops.init_modem();
		}
	}
	else
		ret_val = -ENODEV;

	if (ret_val >= 0)
		printk(KERN_INFO "Loading module %s version %s (%s)\n", serialif_name, serialif_version, serialif_revdate);
	return ret_val;
}

module_init(agr_init_module);
module_exit(serial8250_exit);
MODULE_AUTHOR("Agere Systems Inc (cleanup by ZsoltTech.Com)");
MODULE_DESCRIPTION("Agere Modem Interface driver");
MODULE_LICENSE("GPL");

#if defined(NDZ)
#if defined(CONFIG_SERIAL_8250_RSA) && defined(MODULE)
//MODULE_PARM(probe_rsa, "1-" __MODULE_STRING(PORT_RSA_MAX) "i");
MODULE_PARM_DESC(probe_rsa, "Probe I/O ports for RSA");
//MODULE_PARM(force_rsa, "1-" __MODULE_STRING(PORT_RSA_MAX) "i");
MODULE_PARM_DESC(force_rsa, "Force I/O ports for RSA");
#endif
#endif
