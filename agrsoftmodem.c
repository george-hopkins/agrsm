/**************************************************************
 * File: agrsoftmodem.c
 *
 * Copyright (c) 2002, 2003, 2004, 2005, 2006 Agere Systems, Inc.  All rights reserved.
 *
 * Description:
 *   Agere Soft Modem driver interface functions for Linux
 *
 * Revision History:
 *   Name                   Date          Change
 *   Soumyendu Sarkar       12/03/2002    Initial as agrmodem.c
 *   Soumyendu Sarkar       03/11/2004    Adapted to kernel 2.6.x with PCI support
 *   Soumyendu Sarkar       11/16/2005    Adapted to kernel 2.6.14 with PCI PP modem support
 *************************************************************/
 
#define LUCENT_MODEM
#ifndef AGERE_SOFT_MODEM
#define AGERE_SOFT_MODEM
#endif
#define USB_MODEM 1

//bala-dbg
//#include <linux/config.h>
#include <linux/module.h>

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/sched.h>

#include <linux/ctype.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/pid.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/smp.h>
#include <asm/io.h>
#include <asm/atomic.h>

#ifdef USB_MODEM
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/kref.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#endif

#include "linuxif.h"
#include "agrmodem.h"



extern unsigned int BaseAddress;

#ifndef LT_KER_26
#define LT_KER_26 1
#endif

int numSupportedDevice;
/** list of supported devices - vendorID, deviceID, subSystemID, subVendorID **/
/* if both subSystemID and subVendorID are set to 0, the driver will ignore these fields */
const SUPPORTED_DEVICES agereSupDevices[] = {
  { 0x11C1, 0x0620, 0x0000, 0x0000 },
  { 0x1039, 0x7013, 0x0000, 0x0000 },
  { 0x1106, 0x3068, 0x0000, 0x0000 },
  { 0x8086, 0x2668, 0x0000, 0x0000 }, //ich6 hda
  { 0x8086, 0x27D8, 0x0000, 0x0000 }, //ich7 hda
  { 0x8086, 0x284B, 0x0000, 0x0000 }, //ich8 hda
  { 0x8086, 0x269a, 0x0000, 0x0000 },  /* ESB2 */
  { 0x8086, 0x293e, 0x0000, 0x0000 }, /* ICH9 */
  { 0x8086, 0x293f, 0x0000, 0x0000 }, /* ICH9 */
  { 0x1002, 0x437b, 0x0000, 0x0000 },  /* ATI SB450 */
  { 0x1002, 0x4383, 0x0000, 0x0000 }, /* ATI SB600 */
  { 0x1002, 0x793b, 0x0000, 0x0000 }, /* ATI RS600 HDMI */
  { 0x1002, 0x7919, 0x0000, 0x0000 },  /* ATI RS690 HDMI */
  { 0x1002, 0x960c, 0x0000, 0x0000 }, /* ATI RS780 HDMI */
  { 0x1002, 0xaa00, 0x0000, 0x0000 }, /* ATI R600 HDMI */
  { 0x1106, 0x3288, 0x0000, 0x0000 },  /* VIA VT8251/VT8237A */
  { 0x1039, 0x7502, 0x0000, 0x0000 },  /* SIS966 */
  { 0x10b9, 0x5461, 0x0000, 0x0000 }, /* ULI M5461 */
  { 0x10de, 0x026c, 0x0000, 0x0000 },  /* NVIDIA MCP51 */
  { 0x10de, 0x0371, 0x0000, 0x0000 },  /* NVIDIA MCP55 */
  { 0x10de, 0x03e4, 0x0000, 0x0000 },  /* NVIDIA MCP61 */
  { 0x10de, 0x03f0, 0x0000, 0x0000 }, /* NVIDIA MCP61 */
  { 0x10de, 0x044a, 0x0000, 0x0000 }, /* NVIDIA MCP65 */
  { 0x10de, 0x044b, 0x0000, 0x0000 }, /* NVIDIA MCP65 */
  { 0x10de, 0x055c, 0x0000, 0x0000 },  /* NVIDIA MCP67 */
  { 0x10de, 0x055d, 0x0000, 0x0000 },  /* NVIDIA MCP67 */
  { 0x8086, 0x2416, 0x0000, 0x0000 },
  { 0x8086, 0x2426, 0x0000, 0x0000 },
  { 0x8086, 0x2446, 0x0000, 0x0000 },
  { 0x8086, 0x7196, 0x0000, 0x0000 },
  { 0x8086, 0x2486, 0x0000, 0x0000 },
  { 0x8086, 0x24C6, 0x0000, 0x0000 },
  { 0x8086, 0x24D6, 0x0000, 0x0000 },
  { 0x11C1, 0x048C, 0x0000, 0x0000 },
  { 0x11C1, 0x048F, 0x0000, 0x0000 },
  { 0x8086, 0x266D, 0x0000, 0x0000 },
};

/************** Ensure Kernel Version 2.4.0 or later *************/
#if LINUX_VERSION_CODE < (((2)<<16)|((4)<<8)|((0)))
#error This driver compiles for Linux versions 2.4.0 and later
#endif

static char *modem_name = "Agere Modem Controller driver";
static char *modem_version = "2.1.80";
static char *modem_revdate = "2007-10-01";
struct timer_list timerList;
int (*agr_rs_interrupt)(void) = NULL;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,36)
static spinlock_t modem_driver_lock = SPIN_LOCK_UNLOCKED;
#else
static DEFINE_SPINLOCK(modem_driver_lock);
#endif

/******************* USB Data  **********************************/
#ifdef USB_MODEM

static struct usb_device_id uss_table [] = {
        { USB_DEVICE(USB_USS_VENDOR_ID, USB_USS_PRODUCT_ID_2828) },
        { USB_DEVICE(USB_USS_VENDOR_ID, USB_USS_PRODUCT_ID_2892) },
        { }                                     /* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, uss_table);

static struct usb_driver uss_driver = { 
//bala-dbg
//	.owner		= THIS_MODULE, 
	.name		= "agr12dec2006", 
    	.probe		= uss_probe, 
	.disconnect 	= uss_disconnect, 
	.id_table   	= uss_table, 
};  

struct usb_uss *dev = NULL;
extern unsigned char IsCmdCompleted;
#endif
/********************* Interface Operations *********************/
static int GetAgrModemInterface(void *mdmdata)
{
	struct ltmodem_ops *lucent_modem_ops;

	lucent_modem_ops = (struct ltmodem_ops *)mdmdata;
	lucent_modem_ops->detect_modem = wrap_lucent_detect_modem;
	lucent_modem_ops->init_modem = wrap_lucent_init_modem;
	lucent_modem_ops->PortOpen = modemPortOpen;
	lucent_modem_ops->PortClose = modemPortClose;
	lucent_modem_ops->read_vuart_register = wrap_read_vuart_port;
	lucent_modem_ops->write_vuart_register = wrap_write_vuart_port;
	lucent_modem_ops->app_ioctl_handler = wrap_app_ioctl_handler;
	lucent_modem_ops->dsp_isr = wrap_dum_dp_dsp_isr;
	lucent_modem_ops->read_buffer = wrap_V16550_Read_RBR_buffer;
  return TRUE;
}
EXPORT_SYMBOL(GetAgrModemInterface);  //bala-dbg

/************************ Open / Close *************************/
int modemPortOpen ( void )
{
	wrap_linux_modem_open ();
	return( wrap_vxdPortOpen() );
}

void modemPortClose ( void )
{
	wrap_vxdPortClose();
//	SetAgrModemInterface(0);     //bala-dbg
	wrap_linux_modem_close();
}


/************************** Entry Points ***************************/
int __init modem_init_module (void)
	{
  int ret = 0;
 
	printk("* Hello Agere Driver **\n");
#ifdef USB_MODEM
	ret = usb_register(&uss_driver);
#endif 
  numSupportedDevice = sizeof(agereSupDevices) / sizeof(agereSupDevices[0]);
  ret = wrap_linux_modem_init ();
  if (!ret) {
	  printk("<1>Loading module %s version %s (%s)\n", modem_name, modem_version, modem_revdate);
	  add_taint(TAINT_PROPRIETARY_MODULE);
  }
  else
	  printk("<1>Could not detect Agere soft modem device\n Agere soft modem driver not loaded\n");

  return (ret);
	}

void __exit modem_cleanup_module (void)
	{
#ifdef USB_MODEM
	usb_deregister(&uss_driver);
#endif
	wrap_linux_modem_cleanup ();
	printk("<1>Unloading %s: version %s\n", modem_name, modem_version);
	}

module_init(modem_init_module);
module_exit(modem_cleanup_module);
MODULE_AUTHOR("Agere Systems Inc");
MODULE_DESCRIPTION("Agere Modem Controller driver");
MODULE_LICENSE("Proprietary");
//MODULE_LICENSE("GPL");

/************************ Memory allocation ************************/
fnatr void *x_vmalloc (unsigned int size) { return vmalloc (size); }
fnatr void x_vfree (void *ptr) { vfree (ptr); }
fnatr void *x_kmalloc (unsigned int size) { return kmalloc (size, GFP_KERNEL | GFP_DMA); }
fnatr void x_kfree (void *ptr) { kfree (ptr); }

fnatr unsigned x_virt_to_bus (void *virt_addr) { return virt_to_bus (virt_addr); }
fnatr void *x_ioremap_nocache (unsigned int phys_addr, unsigned size) { return ioremap_nocache (phys_addr, size); }
fnatr void x_iounmap (void *virt_addr) { iounmap (virt_addr); }
	
/************************* I/O Functions ***************************/
fnatr unsigned char agr_inp (unsigned address) { return inb (address); }
fnatr unsigned short agr_inpw (unsigned address) { return inw (address); }
fnatr unsigned agr_inpl (unsigned address) { return inl (address); }

fnatr void agr_outp (unsigned address, unsigned char value) { outb (value, address); }
fnatr void agr_outpw (unsigned address, unsigned short value) { outw (value, address); }
fnatr void agr_outpl (unsigned address, unsigned value) { outl (value, address); }
fnatr unsigned long inpd (unsigned long address) {return (inl (address));}
fnatr void outpd (unsigned long address, unsigned long value) { outl (address, value); }

fnatr void ModemRegOut8(unsigned int port, unsigned char databyte)  { outb ( databyte, (port + BaseAddress) ); }
fnatr void ModemRegOut16(unsigned int port, unsigned int dataword)  { outw( dataword, (port + BaseAddress)); }
fnatr void ModemRegOut32(unsigned int port, unsigned long dataword )	{ outl( dataword, (port + BaseAddress)); }
fnatr unsigned char ModemRegIn8(unsigned int port)  { return (inb (port + BaseAddress)); }
fnatr unsigned short ModemRegIn16(unsigned int port) { return (inw( port + BaseAddress)); }
fnatr unsigned long ModemRegIn32(unsigned int port) { return (inl( port + BaseAddress)); }

#ifndef LT_KER_26
void W_DOWN_IO (void) { __SLOW_DOWN_IO; }
#else
fnatr void x__SLOW_DOWN_IO (void) {}
#endif

/******************* Processor Flag Operations *********************/
#ifndef LT_KER_26
void x_save_flags (unsigned long *flags) { save_flags (*flags); }
void x_restore_flags (unsigned long *flags) { restore_flags (*flags); }
void x_sti (void){ sti (); }
void x_cli (void) { cli (); }
#else   //bala-dbg
fnatr void x_save_flags (unsigned long *flags) { spin_lock_irqsave(&modem_driver_lock, (*flags)); }
fnatr void x_restore_flags (unsigned long *flags) { spin_unlock_irqrestore(&modem_driver_lock, (*flags)); }
fnatr void x_sti (void){}
fnatr void x_cli (void) {}
#endif

/************************** IO - Interrupts ************************/
fnatr void *x_request_region(unsigned long start, unsigned long n, const char *name)
{
  return(request_region(start, n, name));
}

#ifndef LT_KER_26
int x_check_region(unsigned long start, unsigned long n) { return (check_region (start, n)); }
void x_release_region(unsigned long start, unsigned long n) { release_region(start, n); }
#else
fnatr int x_check_region(unsigned long start, unsigned long n) { return (__check_region(&ioport_resource, start, n)); }
fnatr void x_release_region(unsigned long start, unsigned long n) { __release_region(&ioport_resource, (start), (n)); }
#endif

fnatr int x_request_irq(unsigned int irq, void (*handler), unsigned long irqflags, const char * devname, void *dev_id)
{
char devName[] ="LSI Mdm";
//bala Fix for PCI driver. For latest kernel (2.6.18 and above) always use Shared interrupt (SA_SHIRQ).
  return (request_irq(irq, wrap_LX_isr_handler, irqflags | IRQF_SHARED, devName, dev_id));
}
fnatr void x_free_irq(unsigned int irq, void *dev_id) { free_irq( irq, dev_id); }

/********************** Time Operations *************************/
fnatr void x_ms_delay (unsigned ms) { mdelay (ms); }
fnatr void *x_do_gettimeofday(void *tval) { do_gettimeofday ((struct timeval *)tval); return(tval); }
fnatr void lt_add_timer( void (*timerfunction)(unsigned long) )
{
	timerList.expires = jiffies+1;
	timerList.function = wrap_timertick_function;
	timerList.data = 0;
	add_timer(&timerList);
}
fnatr dword VMODEM_Get_System_Time (void)
{
	struct timeval time;
	do_gettimeofday( &time);
	return((time.tv_usec/1000)+(time.tv_sec*1000));
}
fnatr void lt_init_timer ( void  ) { init_timer(&timerList); }

/******************** String / Mem Operations **********************/
fnatr char *x_strcpy (char *dest, const char *src) { return (strcpy (dest, src)); }
fnatr char *x_strncpy (char *dest, const char *src, int maxlen) { return (strncpy (dest, src, maxlen)); }
fnatr char *x_strcat (char *dest, const char *src) { return (strcat (dest, src)); }
fnatr int x_strlen (const char *str) { return (strlen (str)); }
fnatr char *x_strchr (const char *str, char c) { return (strchr (str, c)); }
fnatr void *x_memset (void *dest, char c, int len) { return (memset (dest, c, len)); }
fnatr void *x_memcpy (void *dest, const void *src, int len) { return (memcpy (dest, src, len)); }
fnatr int x_memcmp (const void *ptr1, const void *ptr2, int len) { return (memcmp (ptr1, ptr2, len)); }
fnatr unsigned char x_isprint (char c) { return(isprint (c)); }
fnatr unsigned char x_toupper (char c) { return toupper (c); }
fnatr int kill_proc_wrap(pid_t pid, int sig, int opt) {
	struct task_struct *p;	
	if (pid > 500) {
		p = pid_task(PIDTYPE_PID,pid);
		if (p)
			return (send_sig(sig, p, opt));
		else
			return -EINVAL;
	} else {
		return -ESRCH;
	}
}
fnatr int x_sprintf(char * buf, const char * fmt, ...)
{
	va_list args;
	int i;
//	char buf[2048];
	va_start(args, fmt);
	i=vsprintf(buf,fmt,args);
	va_end(args);
	return (i);
}

/************************** Debug Prints ***************************/
fnatr void linux_debug_message ( byte *msg ) { printk("<1>ltmodem: %s\n",msg); }

fnatr void x_linux_dbg_print(const char *fmt, ...)
	{
	va_list args;
//	char buf[2048];
//	char buf[1536];
	char *buf;
       	buf = kmalloc(1536, GFP_KERNEL);
	if (!buf) return;
	va_start (args, fmt);
	vsprintf (buf, fmt, args);
	va_end (args);
	printk ("%s", buf);
	}

fnatr void x_linux_dbg_print_crit(const char *fmt, ...)
	{
	va_list args;
//	char buf[2048];
//	char buf[1536];
	char *buf;
        buf = kmalloc(1536, GFP_KERNEL);
        if (!buf) return;
	va_start (args, fmt);
	vsprintf (buf, fmt, args);
	va_end (args);
	printk (KERN_CRIT "%s", buf);
	}

/************************** PCI Operations *************************/
#ifndef LT_KER_26
int x_pcibios_present(void) { return(pcibios_present()); }
#else
fnatr int x_pcibios_present(void) { return(1); }
#endif

fnatr unsigned int agr_pci_find_device ( unsigned int vendor, unsigned int device, unsigned int *irqno )
{
	struct pci_dev *dev = NULL;
	while ((dev = pci_get_device(vendor, device, dev)) != NULL) {
		if (pci_enable_device(dev) < 0) {
			return -EIO;
		} else {
			*irqno = dev->irq;
			return 1;
		}
	}
	return -ENODEV;
}

fnatr unsigned int agr_pci_get_irq ( unsigned int vendor, unsigned int device, unsigned int *irqno )
{
	struct pci_dev *dev = NULL;
	while ((dev = pci_get_device(vendor, device, dev)) != NULL) {
		if (pci_enable_device(dev) < 0) {
			return -EIO;
		} else {
			*irqno = dev->irq;
			return 1;
		}
	}
	return -ENODEV;
}
	
fnatr int x_pcibios_read_config_byte (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned char *val)
{struct pci_dev *dev = pci_get_bus_and_slot(bus, dev_fn); return ((!dev)? PCIBIOS_DEVICE_NOT_FOUND : pci_read_config_byte(dev, where, val));}
            
fnatr int x_pcibios_read_config_word (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned short *val)
{struct pci_dev *dev = pci_get_bus_and_slot(bus, dev_fn); return ((!dev)? PCIBIOS_DEVICE_NOT_FOUND : pci_read_config_word(dev, where, val));}

fnatr int x_pcibios_read_config_dword (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned int *val)
{struct pci_dev *dev = pci_get_bus_and_slot(bus, dev_fn); return ((!dev)? PCIBIOS_DEVICE_NOT_FOUND : pci_read_config_dword(dev, where, val));}

fnatr int x_pcibios_write_config_byte (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned char val)
{struct pci_dev *dev = pci_get_bus_and_slot(bus, dev_fn); return ((!dev)? PCIBIOS_DEVICE_NOT_FOUND : pci_write_config_byte(dev, where, val));}

fnatr int x_pcibios_write_config_word (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned short val)
{struct pci_dev *dev = pci_get_bus_and_slot(bus, dev_fn); return ((!dev)? PCIBIOS_DEVICE_NOT_FOUND : pci_write_config_word(dev, where, val));}

fnatr int x_pcibios_write_config_dword (unsigned char bus, unsigned char dev_fn, unsigned char where, unsigned int val)
{struct pci_dev *dev = pci_get_bus_and_slot(bus, dev_fn); return ((!dev)? PCIBIOS_DEVICE_NOT_FOUND : pci_write_config_dword(dev, where, val));}

#ifndef LT_KER_26
int x_pcibios_find_class (unsigned int class_code, unsigned short index, unsigned char *bus, unsigned char *dev_fn)
{
  return(pcibios_find_class (class_code, index, bus, dev_fn));
}
int x_pcibios_find_device (unsigned short vendor, unsigned short dev_id,
			 unsigned short index, unsigned char *bus,
			 unsigned char *dev_fn)
{
return(pcibios_find_device (vendor, dev_id, index, bus, dev_fn));
}
#endif

/************************** OS Operations **************************/
fnatr int x_smp_processor_id (void) { return (smp_processor_id ()); }
#ifndef LT_KER_26
int x_smp_num_cpus (void) {if (smp_num_cpus > 0) return smp_num_cpus;else return 1; }
#else
fnatr int x_smp_num_cpus (void) {if (num_online_cpus() > 0) return (num_online_cpus());else return 1; }
#endif

#ifndef LT_KER_26
void x_MOD_INC_USE_COUNT (void) { MOD_INC_USE_COUNT; }
void x_MOD_DEC_USE_COUNT (void) { MOD_DEC_USE_COUNT; }
#else
fnatr void x_MOD_INC_USE_COUNT (void) { try_module_get(THIS_MODULE); }
fnatr void x_MOD_DEC_USE_COUNT (void) { module_put(THIS_MODULE); }
#endif

fnatr void x_task_queue_init (x_tq_struct_t *task_x, x_func_ptr func, void *data)
	{
#ifndef LT_KER_26
	struct tq_struct *x_tqueue = (struct tq_struct *) task_x;
	INIT_TQUEUE(x_tqueue, func, data);
#else
	struct work_struct *x_tqueue = (struct work_struct *) task_x;

    INIT_WORK(x_tqueue, (work_func_t)wrap_LXHardwareBottomHalf);
#endif
}

fnatr void x_task_queue_init_usb (x_tq_struct_t *task_x, x_func_ptr func, void *data)
	{
#ifndef LT_KER_26
	struct tq_struct *x_tqueue = (struct tq_struct *) task_x;
	INIT_TQUEUE(x_tqueue, func, data);
#else
	struct work_struct *x_tqueue = (struct work_struct *) task_x;

    INIT_WORK(x_tqueue, (work_func_t)func);   
#endif
}

fnatr void x_queue_task	(x_tq_struct_t *task_x)
	{
#ifndef LT_KER_26
	struct tq_struct *x_tqueue = (struct tq_struct *) task_x;
	queue_task (x_tqueue, &tq_immediate);
	mark_bh (IMMEDIATE_BH);
#else
	struct work_struct *x_tqueue = (struct work_struct *) task_x;
  schedule_work(x_tqueue);
#endif
	}

/************************ Atomic  Operations ***********************/
fnatr int x_atomic_read (int *atomic_x) { return atomic_read ((atomic_t *)atomic_x); }
fnatr void x_atomic_set (int *atomic_x, int value) { atomic_set ((atomic_t *)atomic_x, value); }
fnatr void x_atomic_inc (int *atomic_x) { atomic_inc ((atomic_t *)atomic_x); }
fnatr void x_atomic_dec (int *atomic_x) {atomic_dec ((atomic_t *)atomic_x); }
fnatr int x_atomic_dec_and_test (int *atomic_x) { return atomic_dec_and_test ((atomic_t *)atomic_x)? 1: 0; }

/************************ USB Operations ***************************/
#ifdef USB_MODEM

#define to_uss_dev(d) container_of(d, struct usb_uss, kref)
fnatr void *xkmalloc(size_t d) { return kmalloc(d, GFP_KERNEL); }
fnatr void *kmalloc_dev(void) {
	dev = kmalloc(sizeof(struct usb_uss), GFP_KERNEL);
    	if(dev) memset(dev, 0x00, sizeof(struct usb_uss));
    	return dev;
}

fnatr unsigned char *kmalloc_dev_bulk_in_buffer(int buffer_size) {
	dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
	return dev->bulk_in_buffer;
}


fnatr void *x_to_uss_dev(void *p) 	{return to_uss_dev((struct kref *)p); }
fnatr void x_usb_put_dev(void) 		{if(dev) usb_put_dev(dev->udev);}
fnatr void x_kfree_dev(void) 		{if(dev) {kfree (dev->bulk_in_buffer); kfree (dev);} }
fnatr void set_dev_udev(void *p) 	{dev->udev = p; }
fnatr void set_dev_if(void *p) 		{dev->interface = (struct usb_interface *)p;}
fnatr int  get_dev_speed(void) 		{return (dev->udev->speed); }
fnatr void *get_iface_desc(void *p) 	{return  (((struct usb_interface *)p)->cur_altsetting);}
fnatr int  get_num_endpoints(void *p) 	{return (((struct usb_host_interface *)p)->desc.bNumEndpoints);}
fnatr void x_kref_put(void *p) 		{if (dev)  kref_put(&dev->kref, p); }
fnatr void x_kref_init(void)		{kref_init(&dev->kref);}
fnatr int  IsDirIn(int addr) 		{return (addr & USB_DIR_IN);}
fnatr int  IsBulkXfer(int attr) 	{return ((attr & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK);}
fnatr int  x_le16_to_cpu(int d) 	{return le16_to_cpu(d);}
fnatr void x_unlock_kernel(void) 	{}; //{unlock_kernel();}
fnatr void x_lock_kernel(void) 		{}; //{lock_kernel();}
fnatr void *x_usb_get_intfdata(void *p) { return usb_get_intfdata((struct usb_interface *)p);}
fnatr int  get_urb_status(void *p) 	{return (((struct urb *)p)->status);}
fnatr void *get_urb_context(void *p) 	{return (((struct urb *)p)->context);}
fnatr void *get_dev(void) 		{return dev;}
fnatr void *x_usb_alloc_urb_kernel(void) 	{return usb_alloc_urb(0, GFP_KERNEL);}
fnatr void *x_usb_alloc_urb_atomic(void) 	{return usb_alloc_urb(0, GFP_ATOMIC);}
fnatr void x_udelay(unsigned long d) 	{udelay(d);}
fnatr void *get_dev_control_urb(void) 	{return dev->control_urb;}
fnatr int  usb_req_clear_feature(void) 	{return USB_REQ_CLEAR_FEATURE;}
fnatr int  usb_recip_endpoint(void) 	{ return USB_RECIP_ENDPOINT; }
fnatr int  usb_endpoint_halt(void) 	{ return USB_ENDPOINT_HALT; }
fnatr int  negative_enonet(void) 	{ return -ENONET; }
fnatr int  negative_econnreset(void) 	{ return -ECONNRESET; }
fnatr int  negative_eshutdown(void) 	{ return -ESHUTDOWN; }
fnatr int  negative_enomem(void) 	{return -ENOMEM;}
fnatr int  get_urb_transfer_dma(void *p) 	{return (((struct urb *)p)->transfer_dma);}
fnatr void x_usb_free_urb(void *p) 		{usb_free_urb((struct urb *)p);}
fnatr void set_dev_write_urb(int i, void *p) 	{dev->write_urb[i]=(struct urb *)p;}
fnatr void set_dev_read_urb(int i, void *p) 	{dev->read_urb[i]= (struct urb *)p;}
fnatr void set_urb_no_transfer_dma_map(void *p) {(((struct urb *)p)->transfer_flags) |= URB_NO_TRANSFER_DMA_MAP; }
fnatr int  x_usb_submit_urb_atomic(void *p) 	{return usb_submit_urb((struct urb *)p, GFP_ATOMIC);}
fnatr int  x_usb_submit_urb_kernel(void *p) 	{return usb_submit_urb((struct urb *)p, GFP_KERNEL);}
fnatr void set_dev_bulk_in_endpointAddr(int addr) {dev->bulk_in_endpointAddr = addr;}
fnatr void set_dev_bulk_out_endpointAddr(int addr) {dev->bulk_out_endpointAddr = addr;}
fnatr void set_dev_bulk_in_size(int size) 	{dev->bulk_in_size = size;}
fnatr void set_dev_control_urb(void *p) 	{ dev->control_urb = p; }
fnatr unsigned char get_dev_bulk_in_endpointAddr(void) {return dev->bulk_in_endpointAddr; }
fnatr unsigned char get_dev_bulk_out_endpointAddr(void) {return dev->bulk_out_endpointAddr;}
fnatr int x_usb_set_interface(int intf, int alter) 	{return (usb_set_interface(dev->udev, intf, alter));}
fnatr int set_transfer_in_progress(int d) 	{ dev->transfer_in_progress = d; return dev->transfer_in_progress; }
fnatr int get_transfer_in_progress(void) 	{ return dev->transfer_in_progress;}

fnatr void *x_usb_get_dev(void *p){ 
struct usb_interface *intf = p; 
return usb_get_dev(interface_to_usbdev(intf));
}

fnatr int x_usb_control_msg(int reqtype,int request,int value,int index,char *pchar,int size,int timeout) {
return (usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev,0), reqtype, request,value,index,pchar,size,timeout));
}

fnatr int get_endpoint_address(void *p, int i) {
struct usb_host_interface *iface_desc = p;
struct usb_endpoint_descriptor *endpoint;
endpoint = &iface_desc->endpoint[i].desc;
return (endpoint->bEndpointAddress);
}

fnatr int get_endpoint_attrib(void *p, int i) {
struct usb_host_interface *iface_desc = p;
struct usb_endpoint_descriptor *endpoint;
endpoint = &iface_desc->endpoint[i].desc;
return (endpoint->bmAttributes);
}

fnatr int get_endpoint_packsize(void *p, int i) {
struct usb_host_interface *iface_desc = p;
struct usb_endpoint_descriptor *endpoint;
endpoint = &iface_desc->endpoint[i].desc;
return (endpoint->wMaxPacketSize);
}

fnatr void x_usb_set_intfdata(void *p, void *q) {
struct usb_interface *interface = p;
struct usb_uss *device = q;
usb_set_intfdata(interface, device);
}

fnatr void *x_usb_buffer_alloc_atomic( int count, void *p) {	
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
  return (usb_buffer_alloc(dev->udev, count, GFP_ATOMIC, &(((struct urb *)p)->transfer_dma) )); 
#else
  return (usb_alloc_coherent(dev->udev, count, GFP_ATOMIC, &(((struct urb *)p)->transfer_dma) )); 
#endif
}

fnatr void *x_usb_buffer_alloc_kernel( int count, void *p) {	
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
	return (usb_buffer_alloc(dev->udev, count, GFP_KERNEL, &(((struct urb *)p)->transfer_dma))); 
#else
	return (usb_alloc_coherent(dev->udev, count, GFP_KERNEL, &(((struct urb *)p)->transfer_dma))); 
#endif 
  
}

fnatr void x_usb_buffer_free(void *p) {
	struct urb *urb = p;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
	usb_buffer_free(urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
#else
	usb_free_coherent(urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
#endif
  
}

fnatr void x_usb_buffer_free_ext( int count, char *buf, void *p) {
	struct urb *urb = p;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
	usb_buffer_free(urb->dev, count, buf, urb->transfer_dma);
#else
	usb_free_coherent(urb->dev, count, buf, urb->transfer_dma);
#endif
}
 
fnatr void x_usb_fill_bulk_urb_rx(void *urb,char *buf,int count,void *uss_bulk_callback,
	void *xfer_object){
	usb_fill_bulk_urb (urb, dev->udev, usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
	buf, count, uss_bulk_callback, xfer_object);
}

fnatr void x_usb_fill_bulk_urb_tx(void *urb,char *buf,int count,void *uss_bulk_callback,
	void *xfer_object) {
	usb_fill_bulk_urb (urb, dev->udev, usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
		buf, count, uss_bulk_callback, xfer_object);
}

fnatr void x_usb_fill_control_urb_rx(void *urb,	unsigned char *setup_packet,char *buf,int wLength,
	void *ctrl_callback) {
	usb_fill_control_urb (urb, dev->udev, usb_rcvctrlpipe(dev->udev, 0), 
	setup_packet, buf, wLength, ctrl_callback, NULL);
}

fnatr void x_usb_fill_control_urb_tx(void *urb,	unsigned char *setup_packet,char *buf,int wLength,
	void *ctrl_callback) {
	usb_fill_control_urb (urb, dev->udev, usb_sndctrlpipe(dev->udev, 0), 
	setup_packet, buf, wLength, ctrl_callback, NULL);
}


int uss_probe(struct usb_interface *interface, const struct usb_device_id *id) {		
	printk("USB vid 0x%x, pid 0x%x\n", id->idVendor, id->idProduct);	
	return wrap_uss_probe(interface, id);
}

void uss_disconnect(struct usb_interface *p){	
	wrap_uss_disconnect(p);	
}

void uss_read_bulk_callback(void *p , void *q){
	struct urb *urb = p;
	wrap_uss_read_bulk_callback(urb);
}

void uss_write_bulk_callback(void *p , void *q){
	struct urb *urb = p;	
	wrap_uss_write_bulk_callback(urb);
}

void uss_control_callback(void *p , void *q){
	struct urb *urb = p;
	if(urb->status!=0) printk("ERROR status %d\n",urb->status);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
   	usb_buffer_free(urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
#else
	usb_free_coherent(urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
#endif
	IsCmdCompleted=TRUE;
}

#endif

