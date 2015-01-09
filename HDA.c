 /****************************************************************************
 *
 * HDA.c -- Enable LSI modem using ALSA sound driver through HDA interface
 *
 * Description:
 *		This file contains low level functions programming HDA chipset and ALSA
 *
 * Authors: Ting Ma
 * Created 05/30/2007
 * 
 *****************************************************************************/

// #include <sound/driver.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <sound/core.h>
#include "hda_codec.h"
#include <sound/asoundef.h>
//#include <sound/tlv.h>
#include <sound/initval.h>

#define HDA_MODEM

#define fnatr __attribute__((regparm(0)))

#define AZX_MAX_AUDIO_PCMS	6
#define AZX_MAX_MODEM_PCMS	2
#define AZX_MAX_PCMS		(AZX_MAX_AUDIO_PCMS + AZX_MAX_MODEM_PCMS)

//#define HDAdbg 
struct azx_dev {
	struct snd_dma_buffer bdl; /* BDL buffer */
	u32 *posbuf;            /* position buffer pointer */

	unsigned int bufsize;   /* size of the play buffer in bytes */
	unsigned int period_bytes; /* size of the period in bytes */
	unsigned int frags;     /* number for period in the play buffer */
	unsigned int fifo_size; /* FIFO size */
	unsigned long start_jiffies;    /* start + minimum jiffies */
	unsigned long min_jiffies;      /* minimum jiffies before position is valid */

	void __iomem *sd_addr;  /* stream descriptor pointer */

	u32 sd_int_sta_mask;    /* stream int status mask */

	/* pcm support */
	struct snd_pcm_substream *substream;    /* assigned substream,
						 * set in PCM open
						 */
	unsigned int format_val;        /* format value to be set in the
					 * controller and the codec
					 */
	unsigned char stream_tag;       /* assigned stream */
	unsigned char index;            /* stream index */

	unsigned int opened :1;
	unsigned int running :1;
	unsigned int irq_pending :1;
	unsigned int start_flag: 1;     /* stream full start flag */
	/*
	 * For VIA:
	 *  A flag to ensure DMA position is 0
	 *  when link position is not greater than FIFO size
	 */
	unsigned int insufficient :1;
};

/* CORB/RIRB */
struct azx_rb {
	u32 *buf;               /* CORB/RIRB buffer
				 * Each CORB entry is 4byte, RIRB is 8byte
				 *                                  */
	dma_addr_t addr;        /* physical address of CORB/RIRB buffer */
	/* for RIRB */
	unsigned short rp, wp;  /* read/write pointers */
	int cmds;               /* number of pending requests */
	u32 res;                /* last read value */
};

struct azx {
	struct snd_card *card;
	struct pci_dev *pci;
	int dev_index;

	/* chip type specific */
	int driver_type;
	int playback_streams;
	int playback_index_offset;
	int capture_streams;
	int capture_index_offset;
	int num_streams;

	/* pci resources */
	unsigned long addr;
	void __iomem *remap_addr;
	int irq;

	/* locks */
	spinlock_t reg_lock;
	struct mutex open_mutex;

	/* streams (x num_streams) */
	struct azx_dev *azx_dev;

	/* PCM */
	struct snd_pcm *pcm[AZX_MAX_PCMS];

	/* HD codec */
	unsigned short codec_mask;
	int  codec_probe_mask; /* copied from probe_mask option */
	struct hda_bus *bus;

	/* CORB/RIRB */
	struct azx_rb corb;
	struct azx_rb rirb;

	/* CORB/RIRB and position buffers */
	struct snd_dma_buffer rb;
	struct snd_dma_buffer posbuf;

	/* flags */
	int position_fix;
	unsigned int running :1;
	unsigned int initialized :1;
	unsigned int single_cmd :1;
	unsigned int polling_mode :1;
	unsigned int msi :1;
	unsigned int irq_pending_warned :1;
	unsigned int via_dmapos_patch :1; /* enable DMA-position fix for VIA */
	unsigned int probing :1; /* codec probing phase */

	/* for debugging */
	unsigned int last_cmd;  /* last issued command (to sync) */

	/* for pending irqs */
	struct work_struct irq_pending_work;

	/* reboot notifier (for mysterious hangup problem at power-down) */
	struct notifier_block reboot_notifier;
};

extern void snd_hda_get_codec_name(struct hda_codec *, char *, int);
extern struct snd_card *snd_cards[SNDRV_CARDS];
extern unsigned int snd_hda_codec_read(struct hda_codec *codec, hda_nid_t nid, int direct,
				unsigned int verb, unsigned int parm);
extern  int HDA_resume(void);
extern  int HDA_suspend(void);

int *(ResumePt);
int *(SuspendPt);
struct hda_codec *AgrHDACodec;

/* assign a stream for the PCM */
static inline struct azx_dev *azx_assign_device(struct azx *chip, int stream)
{
	int dev, i, nums;
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dev = chip->playback_index_offset;
		nums = chip->playback_streams;
	} else {
		dev = chip->capture_index_offset;
		nums = chip->capture_streams;
	}
	for (i = 0; i < nums; i++, dev++)
		if (! chip->azx_dev[dev].opened) {
			chip->azx_dev[dev].opened = 1;
#ifdef HDAdbg 
			if (stream == SNDRV_PCM_STREAM_PLAYBACK)
				snd_printk(KERN_WARNING "AGR_azx_assign_device SNDRV_PCM_STREAM_PLAYBACK %x %x %p\n",i,dev,&chip->azx_dev[dev]);
			else
				snd_printk(KERN_WARNING "AGR_azx_assign_device SNDRV_PCM_STREAM_receiveBACK %x %x %p\n",i,dev,&chip->azx_dev[dev]);
#endif
	
			return &chip->azx_dev[dev];
		}
	snd_printk(KERN_WARNING "AGR_azx_assign_device    no stream\n");

	return NULL;
}

void fnatr  HDARunDma(struct azx_dev *azx_dev)
{
	azx_dev->running = 1;
}
void fnatr  HDAStopDma(struct azx_dev *azx_dev)
{
	azx_dev->running = 0;
}

/* release the assigned stream */
static inline void azx_release_device(struct azx_dev *azx_dev)
{
#ifdef HDAdbg 
snd_printk(KERN_WARNING "AGR_azx_release_device %x\n",azx_dev->opened );
#endif
	azx_dev->opened = 0;
}

int fnatr releaseHDAStream(void **pDev)
{
#ifdef HDAdbg 
snd_printk(KERN_WARNING"releaseHDAStream %p\n",*pDev);
#endif
azx_release_device(*pDev);
return 1;

}

int fnatr getHDAStream( unsigned int streamType, unsigned char *StreamId, void **pRegMap, void **pDev)
{
struct azx_dev  * azx_dev_temp;

azx_dev_temp = azx_assign_device(snd_cards[0]->private_data, streamType);
if (azx_dev_temp == 0)
	return 0;

*pDev = azx_dev_temp;
*pRegMap = (unsigned int *)azx_dev_temp->sd_addr;
*StreamId  = (unsigned int) azx_dev_temp->stream_tag;
#ifdef HDAdbg 
printk(KERN_ALERT"getHdaStream  dev = %p stream id = %x\n", azx_dev_temp, *StreamId);
#endif
return 1;
}


unsigned long * GetHDABaseAddress(void)
{
struct azx *chip_temp;
struct hda_bus *bus_temp;
unsigned long *temp;

chip_temp = snd_cards[0]->private_data;
bus_temp = chip_temp->bus;
#ifdef HDAdbg 
snd_printk(KERN_ERR"HDA chip address %p \n",chip_temp->remap_addr);
#endif
temp = (unsigned long *) chip_temp->remap_addr;

return(temp);
}


void HDAdump(void)
{
int i;
unsigned long *temp;
struct azx *chip_temp;
struct hda_bus *bus_temp;

chip_temp = snd_cards[0]->private_data;  //probe
bus_temp = chip_temp->bus;
//hda register dump
snd_printk(KERN_ERR"HDA chip address %p \n",chip_temp->remap_addr);
temp = (unsigned long *) chip_temp->remap_addr;
for (i=0; i<(0x1A0/4);)
{
	snd_printk(KERN_ERR"HDA register %04X %08lX %08lX %08lX %08lX\n",i*4,temp[i],temp[i+1],temp[i+2],temp[i+3]);
	i = i+4;
}
}

char findHDACodec(void)
{
	int i;
	struct azx *chip_temp;
	struct hda_bus *bus_temp;
	struct hda_codec *codec_temp;

	chip_temp = snd_cards[0]->private_data;  //probe
	bus_temp = chip_temp->bus;

	for (i=0; i<16; i++)
	{
		codec_temp = bus_temp->caddr_tbl[i];
		if (codec_temp == 0)
			return 0;
		if ((codec_temp->vendor_id >> 16) == 0x11C1)
		{
#ifdef HDAdbg 
			snd_printk(KERN_ERR"found agr codec vendor id %x %x\n",codec_temp->vendor_id,codec_temp->addr & 0x0f);
#endif
			AgrHDACodec = codec_temp;
			return (codec_temp->addr & 0x0F);
		}
	}
	return -ESRCH;
}

void fnatr azlGetResourceInformation(unsigned char *codecAddress,  unsigned char* functionGroupStartNode)
{
#ifdef HDAdbg
	HDAdump();
#endif
	codecAddress[0] =  findHDACodec();
}

int fnatr setResumeCallBack(int *ResumeCallBack)
{
	ResumePt = (int *)AgrHDACodec->patch_ops.resume;
	AgrHDACodec->patch_ops.resume=(void *)ResumeCallBack;
	return(1);
}
int fnatr setSuspendCallBack(int *SuspendCallBack)
{
	SuspendPt = (int *)AgrHDACodec->patch_ops.suspend;
	AgrHDACodec->patch_ops.suspend=(void *)SuspendCallBack;
	return(1);
}
int fnatr resetResumeCallBack(void)
{
	AgrHDACodec->patch_ops.resume=(void *)ResumePt;
	return(1);
}
int fnatr resetSuspendCallBack(void)
{
	AgrHDACodec->patch_ops.suspend=(void *)SuspendPt;
	return(1);
}

unsigned long fnatr LnxTransferCodecVerbs(unsigned long* cmd)
{
unsigned int nid,direct, verb, para, rvalue;
//snd_printk(KERN_ERR"azlTransferCodecVerbs  %x\n", *cmd);
nid = (*cmd >> 20) & 0x7F;
direct = (*cmd >> 27) & 1;
verb = (*cmd >> 8) & 0xFFF;
para = *cmd & 0xFF;
#ifdef HDAdbg 
snd_printk(KERN_ERR"azlTransferCodecVerbs %p %x %x %x %x\n",AgrHDACodec, nid, direct, verb, para);
#endif 
if (AgrHDACodec == 0)
	return 0;
#ifdef HDA_MODEM
rvalue = snd_hda_codec_read(AgrHDACodec, nid, direct, verb, para);
#endif
#ifdef HDAdbg 
snd_printk(KERN_ERR"return %x\n", rvalue);
#endif
return rvalue;
}
unsigned long fnatr LnxTransferCodecVerbsWrite(unsigned long* cmd)
{
unsigned int nid,direct, verb, para, rvalue;
//snd_printk(KERN_ERR"azlTransferCodecVerbsWrite  %x\n", *cmd);
nid = (*cmd >> 20) & 0x7F;
direct = (*cmd >> 27) & 1;
verb = (*cmd >> 8) & 0xFFF;
para = *cmd & 0xFF;
#ifdef HDAdbg 
snd_printk(KERN_ERR"azlTransferCodecVerbsWrite %p %x %x %x %x\n",AgrHDACodec, nid, direct, verb, para);
#endif 
if (AgrHDACodec == 0)
	return 0;
#ifdef HDA_MODEM
rvalue = snd_hda_codec_write(AgrHDACodec, nid, direct, verb, para);
#endif
#ifdef HDAdbg 
snd_printk(KERN_ERR"return %x\n", rvalue);
#endif
return rvalue;
}
