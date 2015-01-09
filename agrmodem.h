/**************************************************************
 * File: agrmodem.h
 *
 * Copyright (c) 2002, 2003, 2004 Agere Systems, Inc.  All rights reserved.
 *
 * Description:
 *   Header for Agere Soft Modem driver interface functions for Linux
 *
 * Revision History:
 *   Name                   Date          Change
 *   Soumyendu Sarkar       12/03/2002    Initial
 *************************************************************/
#ifndef AGRMODEM_H
#define AGRMODEM_H

/******************************* Entry Points **************************/
extern int linux_modem_init (void);
extern void linux_modem_cleanup (void);
extern int linux_modem_open (void);
extern int linux_modem_close (void);

/*************************** Supported Devices ************************/
typedef struct supportedDevices {
  unsigned short vendorID;
  unsigned short deviceID;
  unsigned short subSystemID;
  unsigned short subVendorID;
} SUPPORTED_DEVICES;

/************************** OS Operations **************************/
typedef struct { unsigned long dummy [20];	} x_work_struct_t;
typedef struct { unsigned int dummy [sizeof(struct work_struct)];	} x_tq_struct_t;
typedef void (*x_func_ptr) (void *);

#define fnatr __attribute__((regparm(0)))

extern int modemPortOpen ( void );
extern void modemPortClose ( void );

extern int lucent_detect_modem ( void *pltmodem_res ) fnatr;
extern void lucent_init_modem ( void ) fnatr;
extern int read_vuart_port( int offset) fnatr;
extern void write_vuart_port( int offset, int value) fnatr;
extern int app_ioctl_handler ( unsigned int cmd, unsigned long arg ) fnatr;
extern byte dum_dp_dsp_isr ( void );
extern word V16550_Read_RBR_buffer (byte *data, word size) fnatr;
extern byte vxdPortOpen ( void ) fnatr;
extern void vxdPortClose ( void ) fnatr;
extern int LXHardwareCommonTopHalf (unsigned irq, void *_common, void *regs) fnatr;
extern void timertick_function ( unsigned long Instance ) fnatr;
extern void LXHardwareBottomHalf (void *hardware) fnatr;
extern int LX_isr_handler (int irq, void *_isr, void *regs) fnatr;

int wrap_lucent_detect_modem( void *pltmodem_res ){return lucent_detect_modem (pltmodem_res);}
void wrap_lucent_init_modem( void ) {lucent_init_modem();}
byte wrap_vxdPortOpen( void ) {return vxdPortOpen();}
void wrap_vxdPortClose( void ) {vxdPortClose();}
int wrap_linux_modem_init(void){return linux_modem_init();}
void wrap_linux_modem_cleanup(void){linux_modem_cleanup();}
int wrap_linux_modem_open(void){return linux_modem_open();}
int wrap_linux_modem_close(void){return linux_modem_close();}
int wrap_read_vuart_port( int offset){return read_vuart_port(offset);}
void wrap_write_vuart_port( int offset, int value){write_vuart_port(offset, value);}
//void wrap_write_vuart_port( int offset, int value)
//{
//	printk("<1>Loading module %s version %s (%s)\n", modem_name, modem_version, modem_revdate);
//	write_vuart_port(offset, value);
//}
int wrap_app_ioctl_handler( unsigned int cmd, unsigned long arg ){return app_ioctl_handler(cmd, arg);}
byte wrap_dum_dp_dsp_isr( void ){return dum_dp_dsp_isr();}
word wrap_V16550_Read_RBR_buffer(byte *data, word size){return V16550_Read_RBR_buffer(data, size);}

/******************* Functions passed on to Kernel *****************/  
void wrap_timertick_function ( unsigned long Instance ) { timertick_function (Instance);}
void wrap_LXHardwareBottomHalf (void *hardware){LXHardwareBottomHalf(hardware);}
irqreturn_t wrap_LX_isr_handler (int irq, void *_isr){LX_isr_handler (irq, _isr, NULL); return IRQ_HANDLED;}

/************************ Memory allocation ************************/
void *x_vmalloc (unsigned int size) fnatr;
void x_vfree (void *ptr) fnatr;
void *x_kmalloc (unsigned int size) fnatr;
void x_kfree (void *ptr) fnatr;

unsigned x_virt_to_bus (void *virt_addr) fnatr;
void *x_ioremap_nocache (unsigned int phys_addr, unsigned size) fnatr;
void x_iounmap (void *virt_addr) fnatr;
	
/************************* I/O Functions ***************************/
unsigned char agr_inp (unsigned address) fnatr;
unsigned short agr_inpw (unsigned address) fnatr;
unsigned agr_inpl (unsigned address) fnatr;

void agr_outp (unsigned address, unsigned char value) fnatr;
void agr_outpw (unsigned address, unsigned short value) fnatr;
void agr_outpl (unsigned address, unsigned value) fnatr;
unsigned long inpd (unsigned long address) fnatr;
void outpd (unsigned long address, unsigned long value) fnatr;

void ModemRegOut8(unsigned int port, unsigned char databyte) fnatr;
void ModemRegOut16(unsigned int port, unsigned int dataword) fnatr;
void ModemRegOut32(unsigned int port, unsigned long dataword ) fnatr;
unsigned char ModemRegIn8(unsigned int port) fnatr;
unsigned short ModemRegIn16(unsigned int port) fnatr;
unsigned long ModemRegIn32(unsigned int port) fnatr;
#ifndef LT_KER_26
void x__SLOW_DOWN_IO (void) { __SLOW_DOWN_IO; }
#else
void x__SLOW_DOWN_IO (void) fnatr;
#endif
/******************* Processor Flag Operations *********************/
#ifndef LT_KER_26
void x_save_flags (unsigned long *flags);
void x_restore_flags (unsigned long *flags);
void x_sti (void);
void x_cli (void);
#else
void x_save_flags (unsigned long *flags) fnatr;
void x_restore_flags (unsigned long *flags) fnatr;
void x_sti (void) fnatr;
void x_cli (void) fnatr;
#endif
/************************** IO - Interrupts ************************/
void *x_request_region(unsigned long start, unsigned long n, const char *name) fnatr;
#ifndef LT_KER_26
int x_check_region(unsigned long start, unsigned long n);
void x_release_region(unsigned long start, unsigned long n);
#else
int x_check_region(unsigned long start, unsigned long n) fnatr;
void x_release_region(unsigned long start, unsigned long n) fnatr;
#endif
int x_request_irq(unsigned int irq, void (*handler), unsigned long irqflags, const char * devname, void *dev_id) fnatr;
void x_free_irq(unsigned int irq, void *dev_id) fnatr;
/********************** Time Operations *************************/
void x_ms_delay (unsigned ms) fnatr;
void *x_do_gettimeofday(void *tval) fnatr;
void lt_add_timer( void (*timerfunction)(unsigned long) ) fnatr;
dword VMODEM_Get_System_Time (void) fnatr;
void lt_init_timer ( void  ) fnatr;
/******************** String / Mem Operations **********************/
char *x_strcpy (char *dest, const char *src) fnatr;
char *x_strncpy (char *dest, const char *src, int maxlen) fnatr;
char *x_strcat (char *dest, const char *src) fnatr;
int x_strlen (const char *str) fnatr;
char *x_strchr (const char *str, char c) fnatr;
void *x_memset (void *dest, char c, int len) fnatr;
void *x_memcpy (void *dest, const void *src, int len) fnatr;
int x_memcmp (const void *ptr1, const void *ptr2, int len) fnatr;
unsigned char x_isprint (char c) fnatr;
unsigned char x_toupper (char c) fnatr;
int x_sprintf(char * buf, const char * fmt, ...) fnatr;
/************************** Debug Prints ***************************/
void linux_debug_message ( byte *msg ) fnatr;

void x_linux_dbg_print(const char *fmt, ...) fnatr;
void x_linux_dbg_print_crit(const char *fmt, ...) fnatr;
/************************** PCI Operations *************************/
#ifndef LT_KER_26
int x_pcibios_present(void);
#else
int x_pcibios_present(void) fnatr;
#endif
unsigned int agr_pci_find_device ( unsigned int vendor, unsigned int device, unsigned int *irqno ) fnatr;
int x_pcibios_read_config_byte (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned char *val) fnatr;
int x_pcibios_read_config_word (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned short *val) fnatr;
int x_pcibios_read_config_dword (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned int *val) fnatr;
int x_pcibios_write_config_byte (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned char val) fnatr;
int x_pcibios_write_config_word (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned short val) fnatr;
int x_pcibios_write_config_dword (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned int val) fnatr;
#ifndef LT_KER_26
int x_pcibios_find_class (unsigned int class_code, unsigned short index, unsigned char *bus, unsigned char *dev_fn);
int x_pcibios_find_device (unsigned short vendor, unsigned short dev_id,
			 unsigned short index, unsigned char *bus,
			 unsigned char *dev_fn);
#endif
/************************** OS Operations **************************/
int x_smp_processor_id (void) fnatr;
#ifndef LT_KER_26
int x_smp_num_cpus (void);
#else
int x_smp_num_cpus (void) fnatr;
#endif
#ifndef LT_KER_26
void x_MOD_INC_USE_COUNT (void);
void x_MOD_DEC_USE_COUNT (void);
#else
void x_MOD_INC_USE_COUNT (void) fnatr;
void x_MOD_DEC_USE_COUNT (void) fnatr;
#endif

void x_task_queue_init (x_tq_struct_t *task_x, x_func_ptr func, void *data) fnatr;
void x_task_queue_init_usb (x_tq_struct_t *task_x, x_func_ptr func, void *data) fnatr;
void x_queue_task (x_tq_struct_t *task_x) fnatr;
// void x_task_queue_init (x_tq_struct_t *task_x, x_func_ptr func, void *data) fnatr;;
// void x_queue_task (x_tq_struct_t *task_x) fnatr;;
/************************ Atomic  Operations ***********************/
int x_atomic_read (int *atomic_x) fnatr;
void x_atomic_set (int *atomic_x, int value) fnatr;
void x_atomic_inc (int *atomic_x) fnatr;
void x_atomic_dec (int *atomic_x) fnatr;
int x_atomic_dec_and_test (int *atomic_x) fnatr;

/************************ USB Operations ***************************/
#ifdef USB_MODEM
#define LOOP_UNTIL -1 
#define MAX_URBS   2
#define USB_USS_VENDOR_ID      	    0x047e
#define USB_USS_PRODUCT_ID_2828     0x2828
#define USB_USS_PRODUCT_ID_2892     0x2892

extern fnatr int wrap_uss_probe (void *interface, const void *id);
extern fnatr int wrap_uss_probe (void *interface, const void *id);
extern fnatr void wrap_uss_disconnect(void *intf);
extern fnatr void wrap_uss_read_bulk_callback(void *urb);
extern fnatr void wrap_uss_write_bulk_callback(void *p);

int uss_probe(struct usb_interface *interface, const struct usb_device_id *id);
void uss_disconnect(struct usb_interface *p);

struct usb_uss {
	struct usb_device *     udev;                   /* the usb device for this device */
	struct usb_interface *  interface;              /* the interface for this device */
	unsigned char *         bulk_in_buffer;         /* the buffer to receive data */
	size_t                  bulk_in_size;           /* the size of the receive buffer */
	__u8                    bulk_in_endpointAddr;   /* the address of the bulk in endpoint */
	__u8                    bulk_out_endpointAddr;  /* the address of the bulk out endpoint */
	struct kref             kref;
	struct urb				*read_urb[MAX_URBS];
	struct urb				*write_urb[MAX_URBS];
	struct urb				*readurb;
	struct urb 				*writeurb;
	unsigned char			read_buffer[2000];
	unsigned char			transfer_in_progress;
	struct urb 				*control_urb;	
	unsigned char*			control_buffer;
};
#endif
#endif

