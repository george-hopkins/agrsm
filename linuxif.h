/****************************************************************************
 *
 * linuxif.h
 *
 * Copyright (C) 2002, 2003, 2004 Agere Systems Inc. All rights reserved.
 *
 * Description:
 * Open source Header file for Agere specific Linux driver files
 *
 * Provided by Agere Systems, Inc.
 * Author: Soumyendu Sarkar
 *
 ***************************************************************************/

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

#define LT_SERIAL_PORT_DEFNS			        \
	/* UART CLK   PORT IRQ     FLAGS         */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS0 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS1 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS2 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS3 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 }, 	/* ttyS4 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS5 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS6 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS7 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS8 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS9 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS10 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS11 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS12 */	\
	{ 0, BASE_BAUD, 0x000, 0, 0 },	/* ttyS13 */	\
	{ 0, BASE_BAUD, 0x000, 0, ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST },	/* ttyS14 */	


#ifdef PORT_MAX
#undef PORT_MAX
#endif

#define PORT_LTMODEM 15
#define PORT_MAX     15
#define TRUE 1
#define FALSE 0

