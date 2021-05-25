/*
 * Davicom DM9620 USB 2.0 10/100Mbps ethernet devices
 *
 * Peter Korsgaard <jacmet@sunsite.dk>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 * V1.0 - ftp fail fixed
 * V1.1 - model name checking, & ether plug function enhancement [0x4f, 0x20]
 * V1.2 - init tx/rx checksum
 *      - fix dm_write_shared_word(), bug fix
 *      - fix 10 Mbps link at power saving mode fail  
 * V1.3 - Support kernel 2.6.31
 * V1.4 - Support eeprom write of ethtool 
 *        Support DM9685
 *        Transmit Check Sum Control by Optopn (Source Code Default: Disable)
 *        Recieve Drop Check Sum Error Packet Disable as chip default
 * V1.5 - Support RK2818 (Debug the Register Function)
 * V1.6 - Solve compiler issue for Linux 2.6.35
 * V1.7 - Enable MAC Layer Flow Control and define debug_message for linux version update.
 * V1.8 - Enable PHY Layer Flow Control, clear debug code, setup default phy_id value is 1.
 *        Update dm9620_mdio_read and dm9620_mdio_write.
 *        Fix bug of ethtool eeprom write       
 * V1.9 - Fixed "deverr" line 367 error in Linux 2.6.38
 * V2.0 - Fixed "dm9620_set_multicast" function CRC bug.
 * V2.1 - Add 802.3az for dm9621a
 * V2.2 - Add PID=0x1269 support CDC mode.
 * V2.3 - Add PID=0x0269 support CDC mode.       
 * V2.41 - Support Linux 3.6.9    
 * V2.42 - Work to V2.42 according to "DM9620 BulkOut ¸É¤B¤À¸Ñ.doc"
 * V2.43 - Special suport for DM9621A in the table 'products'
 * V2.45 - Fix the function TxStyle(), correct to be (len%2) from (len%1). 20131211.
 * V2.47 - Special suport for DM9621A by increase the table 'products', 20140617.
 * V2.49 - EEPROM_Utility_support (Report EEPROM_LEN from 256 to 128), 20140919
 * V2.51 - Suitable to DM9620/21 E3, there is no extra VID/PID list insided.
 * V2.59 - EEPROM_Utility_support (Report EEPROM_LEN from 256 to 128), 20141008
 * v2.59.1 - Add "DM962XA EXT MII" 
 *           "DM962XA EXT MII" only PID, others by board circuit (0x0268,0x1268)
 *           "DM962XA Fiber" only PID, others tb.492 (0x0267,0x1267)
 *           Check reg[0].bit[7] and reply the modified bmcr & bmsr by refer to reg[0]&reg[1].
 * v2.59.2 - Add "DM962XA EXT MII" 
 *           correct the report of "ifconfig  eth1 up" ( loc==4: res= 0x05e1;  loc==5: res= 0x45e1;)
 * v2.59.3t7 - Ehance_link_support use defer_kevent for DEFER RESET
 *             operation IP_toggle_count= 36
 *             dm9620_bind() DIRECT config to DM_SMIREG
 *             dump device descriptors
 *             render eeprom if need
 * v2.59.3t12-6p -  USBEr(CLEAN.DBGMOD)
 * v2.59.3t12-6ppp-fix112-carrier-mode -  USBEr(CLEAN.DBGMOD) (5-Times-design)
 * V2.59.3t_6ppp.JJ3.2(FIX126-crr_open_5_lnk) - Add processing WinToAndroid_20160801
 * V2.59.3t_6ppp.JJ3.3(FIX126-crr_open_5_lnk)
 *           - Tune WinToAndroid Descriptors_20160802
 *           - Interrupt Interval to be 0x07 (WD12_EP3I_6ppp)
 * V2.59.3_EXTPHY_STATUS - Correct EXTPHY_STATUS Clean Debug Messages
 *                         When EXTPHY, must define 'DM9620_PHY_ID' to be [1 ~ 3] correspond the board's schematic. 
 *                         Note: For the internal phy, 'DM9620_PHY_ID' is 1, 
 *													So co-usage by phy_id 1 for external phy's default phy_id.
 * V2.59.3_2021_EXTPHY_STATUS - CleanALL, 20210506 
 * V2.60.0c - External phy completely with 'LNX_DRV_EXT_PHY_HAS_MDIO', 20210511
 * V2.60.1 - Fewer debug message for cable disconnect state, set 'WORK_TOGG_CTLP' to 1, 20210511
 */
#define LNX_DM9620_VER_STR  "V2.60.1 WITH EXTPHY 2021 CLEAN_AND_CTLPRINT"
#define LNX_KERNEL_v511	1
#define LNX_DRV_SIMPLE_PHY_AND_EXT_PHY	1 // 1 for simple internal phy and real_external_phy, 0 for simulate as external phy's control flow
#define	LNX_DRV_EXT_PHY_HAS_MDIO	1 // 1 real_external_phy with DMC/MDIO connection, 0 force external phy always link_up
 
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/usb/usbnet.h>
#include <linux/ctype.h>
#include <linux/skbuff.h>   
#include <linux/version.h> // new v1.3

/* datasheet:
 http://www.davicom.com.tw
*/

/* Configuration */
//#define WORK_MII_MEDIA			1	//[mondatory]
#define WORK_INFINIT_TOGG			1	//Toggle Infinite enable, otherwise only 'TOTAL_CNT_TOGG' times (TOTAL_CNT_TOGG = 36)
#define	WORK_NB_CTLP				1	//Control_printing for new buffer
#define	WORK_TOGG_CTLP				1 //1: fewer toogle print, 0: normal toggle print(good "priv->IP_dc += 63;" toggle, normal toggle print is OK.)
										//Control_printing for toggle-process [recommanded]
#define	WORK_LNK_OKP				0	//DBG print for during LInk-up
//#define WORK_CARRI_ON				0
//#define WORK_CARRI_OFF			0

/* control requests */
#define DM_READ_REGS	0x00
#define DM_WRITE_REGS	0x01
#define DM_READ_MEMS	0x02
#define DM_WRITE_REG	0x03
#define DM_WRITE_MEMS	0x05
#define DM_WRITE_MEM	0x07

/* registers */
#define DM_NET_CTRL	0x00
#define DM_RX_CTRL	0x05
#define DM_FLOW_CTRL	0x0a
#define DM_SHARED_CTRL	0x0b
#define DM_SHARED_ADDR	0x0c
#define DM_SHARED_DATA	0x0d	/* low + high */
#define DM_EE_PHY_L	0x0d
#define DM_EE_PHY_H	0x0e
#define DM_WAKEUP_CTRL  0x0f
#define DM_PHY_ADDR	0x10	/* 6 bytes */
#define DM_MCAST_ADDR	0x16	/* 8 bytes */
#define DM_GPR_CTRL	0x1e
#define DM_GPR_DATA	0x1f
#define DM_PID      0x2a
#define DM_XPHY_CTRL	0x2e
#define DM_TX_CRC_CTRL	0x31
#define DM_RX_CRC_CTRL	0x32 
#define DM_SMIREG       0x91
#define USB_CTRL	0xf4
#define PHY_SPEC_CFG	20
#define DM_TXRX_M       0x5C

#define DMSC_WEP	0x10
#define DMSC_ERPRW	0x02
#define DMSC_ERRE	0x01

#define MD96XX_EEPROM_MAGIC	0x9620
#define DM_MAX_MCAST	64
#define DM_MCAST_SIZE	8
#define DM_EEPROM_LEN	128
#define DM_TX_OVERHEAD	2	/* 2 byte header */
#define DM_RX_OVERHEAD_9601	7	/* 3 byte header + 4 byte crc tail */
#define DM_RX_OVERHEAD		8	/* 4 byte header + 4 byte crc tail */
#define DM_TIMEOUT	1000
#define DM_MODE9620     0x80
#define DM_TX_CS_EN	0        /* Transmit Check Sum Control */
#define DM9620_PHY_ID 1      /* Stone add For kernel read phy register */

/* Macro */
#define dm9620_err(dev, format, args...) printk(format, ##args)

//#if WORK_MII_MEDIA //[mondatory]
//#define	MII_CHK_MEDIA_OKTOPRINT(usbdevmii, okp, initm, carrier)	dm9620_check_media(usbdevmii, okp, initm, carrier) //'mii_check_media'
//#endif

#if WORK_NB_CTLP //Control_printing for new buffer [recommanded]
#define	printnb_init()				printnb_bind_clean()
#define printnb(format, args...)	printnb_process(format, ##args)
#endif

#if WORK_TOGG_CTLP //Control_printing [recommanded]
#define	CTLPRINT_OPEN()				open_printk_processes()				// group counters initialization
#define CTLPRINT(format, args...)	printk_process(format, ##args)		// individual when is to linking up (increasement)
#define TGGLPRINTX(format, args...)	printk_process_grp(format, ##args)	// group raw_item (not increasment)
#define TGGLPRINT(format, args...)	printk_process_grpe(format, ##args)	// group end_item (increasement)
#else
#define	CTLPRINT_OPEN()				// no need init
#define	CTLPRINT		printk		// always print
#define	TGGLPRINTX		printk		// always print
#define	TGGLPRINT		printk		// always print
#endif

/* Constants */
#define TOTAL_CNT_TOGG             36
#define DEFER_ON_CONTROL		  150

/* Flags */
#define FORCE_MODE_MAGIC	0xbd

#define	TRIGGER_IDLE			0
#define	TRIGGER_TO_TOGGLE		1
#define	TRIGGER_LOOPING_150		2
#define	TRIGGER_LINKUP_FOUND	3

#define STR_IDLE	"IDLE"
#define STR_TOGGLE	"TOGGLE"
#define STR_LOOPING	"LOOPING"
#define STR_FOUND	"FOUND"

struct dm96xx_priv {
    u16 sav_lpa;
	
	int opkey_input[3];
	int opnum_op;
	
	int block_input[2];
	int save_input;
	
	int IP_conf_track_bas[2];
	int IP_conf_rxdef; // def is want to (2 stage design by JJ)
	int IP_conf_rxmatch;
  //int IP_conf_txmatch;
	int IP_conf_rxplus;
	
    int IP_conf_inc;
    int IP_save_inc;
    
  //int	flag_fail_count; // EVER RX-DBG
    int flg_txdbg; // NOW TX-DBG
    int IP_dc;
    int IP_di;
    
    int  defer_on_looping;
	int  coming_lnk_defer_on_ct;
	int  summUsbErr;
	int  summPwOnTgl;
//printnb.s //[nb: new buffer]
struct {
  int enab;
  int n;
  char bff[100];
} nb;
#define TGL_EXTRA_WRITE 6 // Relaese: want > 6, 18, [25], 250, 500

	//int  summCarrierTgl;
	
	u8  coming_link_defer_on; /* int/u8 */
	
	u8  mode_9620;	
	u8	tx_fix_mod;	
	u8  Force_mode_magic;
	
	int  IP_toggle_count;
	u8  IP_TOGG;
  //u8  IP_done=
    u8  allow_defer_happen;
    
    u8  desc_ep3i; /* store data for print */
    u8  desc_ep3i_pntonce;
    
    u8  defer_count;
	u8  mode_ext_phy;	
	
	int	mode_ext_link;
	
	u16	mode_ext_phyid1;			
	u16	status_on_trigger;
	
	bool status_carrier_ok;
	bool linkrst_carrier_ok;
	
	u8  mdat[512];
};

#define DM_LINKEN  (1<<5)
#define DM_MAGICEN (1<<3)
#define DM_LINKST  (1<<2)
#define DM_MAGICST (1<<0)

/*#define printnb(format, args...)	printnb_process(format, ##args)
void printnb_process(const char *format, ...)
{
  struct va_format vaf;
  va_list args;
  //if (!nb.enab)
  //  return;
  
  va_start(args, format);
  vaf.fmt= format;
  vaf.va= &args;
  nb.n += sprintf(&nb.bff[nb.n], "%pV", &vaf); 
  va_end(args);
  
  if (nb.bff[nb.n -1]=='\n') {
    printk(nb.bff);
    nb.n = 0;
  }
}*/

#if WORK_NB_CTLP
//[nb: new buffer]
struct {
  //int enab;
  int n;
  char bff[100];
} nb;

void printnb_bind_clean(void) // called 'printnb_init()' (or called directly by 'printnb_bind_clean()')
{
  //param(int enab)
  //nb.enab = enab;
  nb.n = 0; //printnb's initialization-reset.
}

void printnb_process(const char *format, ...) // called by 'printnb(format, args...)'
{
  struct va_format vaf;
  va_list args;
  //if (!nb.enab)
  //  return;
  
  va_start(args, format);
  vaf.fmt= format;
  vaf.va= &args;
  nb.n += sprintf(&nb.bff[nb.n], "%pV", &vaf); 
  va_end(args);
  
  if (nb.bff[nb.n -1]=='\n') {
    printk(nb.bff);
    nb.n = 0;
  }
}
#endif

#if WORK_TOGG_CTLP
u8 counter_count, counter_grp_count; //Global varibles for Control_printing

static void open_printk_processes(void)
{
	counter_count = counter_grp_count = 0;
}

static void printk_process(const char *format, ...) //'CTLPRINT'
{
	struct va_format vaf;
	va_list args;
  
	if (counter_count == 0) {
		//printk();=
  		va_start(args, format);
  		vaf.fmt= format;
		vaf.va= &args;
		printk("%pV", &vaf);
		va_end(args);
	}
	counter_count++;
	counter_count = counter_count % 60; //30; //60; //30
}

static void printk_process_grp(const char *format, ...)
{
	struct va_format vaf;
	va_list args;
	if (counter_grp_count == 0) {
  		va_start(args, format);
  		vaf.fmt= format;
		vaf.va= &args;
		printk("%pV", &vaf);
		va_end(args);
	}
}

static void printk_process_grpe(const char *format, ...)
{
	struct va_format vaf;
	va_list args;
	if (counter_grp_count == 0) {
  		va_start(args, format);
  		vaf.fmt= format;
		vaf.va= &args;
		printk("%pV", &vaf);
		va_end(args);
	}
	counter_grp_count++;
	counter_grp_count = counter_grp_count % 151; //150 //30 (odd counter number for 810h/830h)
}
#endif //WORK_TOGG_CTLP

#define udelayEx(n)		\
	udelay(n);			\
	udelay(n);			\
	udelay(n);			\
	udelay(n);			\
	udelay(n);			\
	udelay(n);			\
	udelay(n)

static int dm_read(struct usbnet *dev, u8 reg, u16 length, void *data)
{
	return usb_control_msg(dev->udev,
			       usb_rcvctrlpipe(dev->udev, 0),
			       DM_READ_REGS,
			       USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			       0, reg, data, length, USB_CTRL_SET_TIMEOUT); //USB_CTRL_SET_TIMEOUT V.S. USB_CTRL_GET_TIMEOUT
}

static int dm_read_reg(struct usbnet *dev, u8 reg, u8 *value)
{
	u16 *tmpwPtr;
	int ret;
	tmpwPtr= kmalloc (2, GFP_ATOMIC);
	if (!tmpwPtr){
		printk("+++++++++++ JJ5 dm_read_reg() Error: can not kmalloc!\n"); //usbnet_suspend (intf, message);
		return 0; 
	}
	ret = dm_read(dev, reg, 2, tmpwPtr);  // usb_submit_urb v.s. usb_control_msg
	*value= (u8)(*tmpwPtr & 0xff);
	kfree (tmpwPtr);
	return ret;
}

static int dm_read_regs(struct usbnet *dev, u8 reg, u16 length, void *data)
{
	int ret;
	u16 *tmpwPtr = kmalloc (length, GFP_ATOMIC);
	if (!tmpwPtr){
		printk("+++++++++++ dm9 JJ5 dm_read_reg() Error: can not kmalloc!\n"); //usbnet_suspend (intf, message);
		return 0; 
	}
	ret = dm_read(dev, reg, length, tmpwPtr);  // usb_submit_urb v.s. usb_control_msg
	memcpy(data, tmpwPtr, length);
	kfree (tmpwPtr);
	return ret;
}

static int dm_write(struct usbnet *dev, u8 reg, u16 length, void *data)
{
	return usb_control_msg(dev->udev,
			       usb_sndctrlpipe(dev->udev, 0),
			       DM_WRITE_REGS,
			       USB_DIR_OUT | USB_TYPE_VENDOR |USB_RECIP_DEVICE,
			       0, reg, data, length, USB_CTRL_SET_TIMEOUT);
}

static int dm_write_reg(struct usbnet *dev, u8 reg, u8 value)
{
	return usb_control_msg(dev->udev,
			       usb_sndctrlpipe(dev->udev, 0),
			       DM_WRITE_REG,
			       USB_DIR_OUT | USB_TYPE_VENDOR |USB_RECIP_DEVICE,
			       value, reg, NULL, 0, USB_CTRL_SET_TIMEOUT);
}

static void dm_write_async_callback(struct urb *urb)
{
	struct usb_ctrlrequest *req = (struct usb_ctrlrequest *)urb->context;
	if (urb->status < 0)
		printk(KERN_DEBUG "dm_write_async_callback() failed with %d\n",
		       urb->status);
	kfree(req);
	usb_free_urb(urb);
}

static void dm_write_async_helper(struct usbnet *dev, u8 reg, u8 value,
				  u16 length, void *data)
{
	struct usb_ctrlrequest *req;
	struct urb *urb;
	int status;
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		dm9620_err(dev, "Error allocating URB in dm_write_async_helper!");
		return;
	}
	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC);
	if (!req) {
		dm9620_err(dev, "Failed to allocate memory for control request");
		usb_free_urb(urb);
		return;
	}
	req->bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	req->bRequest = length ? DM_WRITE_REGS : DM_WRITE_REG;
	req->wValue = cpu_to_le16(value);
	req->wIndex = cpu_to_le16(reg);
	req->wLength = cpu_to_le16(length);
	usb_fill_control_urb(urb, dev->udev,
			     usb_sndctrlpipe(dev->udev, 0),
			     (void *)req, data, length,
			     dm_write_async_callback, req);
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status < 0) {
		dm9620_err(dev, "Error submitting the control message: status=%d",
		       status);      
		kfree(req);
		usb_free_urb(urb);
	}
}

static void dm_write_async(struct usbnet *dev, u8 reg, u16 length, void *data)
{
	dm_write_async_helper(dev, reg, 0, length, data);
}

static void dm_write_reg_async(struct usbnet *dev, u8 reg, u8 value)
{
	dm_write_async_helper(dev, reg, value, 0, NULL);
}

static int dm_read_shared_word(struct usbnet *dev, int phy, u8 reg, __le16 *value)
{
	int ret, i;
  u16 *tmpwPtr1;
	u8 phy_id_bits;
	
	mutex_lock(&dev->phy_mutex);

	phy_id_bits = (u8) phy; //instead as param, (here not from 'dev->mii.phy_id')
	phy_id_bits = phy_id_bits << 6;

	//dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | phy_id_bits) : reg); //also suitable for external phy, allow 'phy' = phy_id [1~3]
	dm_write_reg(dev, DM_SHARED_CTRL, phy ? 0xc : 0x4);

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp;

		udelayEx(1);
		ret = dm_read_reg(dev, DM_SHARED_CTRL, &tmp);
		if (ret < 0) {
			if (phy)
			  printk("++++++phy id= %d [rdPHY %d]+++++  ret<0 out return\n", phy, reg);
			else
			  printk("++++++phy id= %d [rdWORD %d]+++++  ret<0 out return\n", phy, reg);
			goto out;
		}

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		dm9620_err(dev, "%s read timed out!", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	dm_write_reg(dev, DM_SHARED_CTRL, 0x0);
//	ret = dm_read(dev, DM_SHARED_DATA, 2, value); 
//Stone add
	tmpwPtr1= kmalloc (2, GFP_ATOMIC);
	if (!tmpwPtr1)
	{
		printk("+++++++++++ JJ5 dm_read_reg() Error: can not kmalloc!\n"); //usbnet_suspend (intf, message);
		return -EIO; //-1; 
	}
	
	ret = dm_read(dev, DM_SHARED_DATA, 2, tmpwPtr1);  // usb_submit_urb v.s. usb_control_msg
	*value= (u16)(*tmpwPtr1 & 0xffff);
	
	kfree (tmpwPtr1); 
 out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static int dm_write_shared_word(struct usbnet *dev, int phy, u8 reg, __le16 value)
{
	int ret, i;
	u8 phy_id_bits;

	mutex_lock(&dev->phy_mutex);

	phy_id_bits = (u8) phy; //instead as param, (here not from 'dev->mii.phy_id')
	phy_id_bits = phy_id_bits << 6;

	ret = dm_write(dev, DM_SHARED_DATA, 2, &value);
	if (ret < 0)
		goto out;

	//dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | phy_id_bits) : reg); //also suitable for external phy, allow 'phy' = phy_id [1~3]
	if (!phy) dm_write_reg(dev, DM_SHARED_CTRL, 0x10);
	dm_write_reg(dev, DM_SHARED_CTRL, phy ? 0x0a : 0x12);
	dm_write_reg(dev, DM_SHARED_CTRL, 0x10);

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp;

		udelayEx(1);
		ret = dm_read_reg(dev, DM_SHARED_CTRL, &tmp);
		if (ret < 0)
			goto out;

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		dm9620_err(dev,"%s write timed out!", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	dm_write_reg(dev, DM_SHARED_CTRL, 0x0);

out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static void device_polling(struct usbnet *dev, u8 reg, u8 dmsc_bit, 
          u8 uexpected)
{
	int i,ret;
	u8 tmp= 0;
	for (i = 0; i < DM_TIMEOUT; i++) {
		udelayEx(1);
		ret = dm_read_reg(dev, reg, &tmp);
		if (ret < 0){
		        dm9620_err(dev,
				"[dm962 read reg] (reg: 0x%02x) error!\n", 
                reg);
			break;
        }
		if ((tmp & dmsc_bit) == uexpected) /* ready */
			break;
	}
	if (i == DM_TIMEOUT)
		dm9620_err(dev, "[dm962 time out] on polling bit:0x%x\n",
          dmsc_bit);
}

static void dm_write_eeprom_word(struct usbnet *dev, u8 offset, u8 *data)
{
	mutex_lock(&dev->phy_mutex);
    dm_write_reg(dev, DM_SHARED_ADDR, offset);
	dm_write_reg(dev, DM_EE_PHY_H, data[1]);
	dm_write_reg(dev, DM_EE_PHY_L, data[0]);
    dm_write_reg(dev, DM_SHARED_CTRL, DMSC_WEP | DMSC_ERPRW);
        device_polling(dev, DM_SHARED_CTRL, DMSC_ERRE, 0x00);			
	dm_write_reg(dev, DM_SHARED_CTRL, 0);
	mutex_unlock(&dev->phy_mutex);
}

static int dm_read_eeprom_word(struct usbnet *dev, u8 offset, void *value)
{
	return dm_read_shared_word(dev, 0, offset, value);
}

static int dm9620_set_eeprom(struct net_device *net,
        struct ethtool_eeprom *eeprom, u8 *data)
{
	struct usbnet *dev = netdev_priv(net);
	int offset = eeprom->offset;
	int len = eeprom->len;
	int done;
	if (eeprom->magic != MD96XX_EEPROM_MAGIC) {
		printk("dm9 EEPROM: magic value mismatch, magic = 0x%x\n",
			eeprom->magic);	
		return -EINVAL;
	}
	while (len > 0) {
		if (len & 1 || offset & 1) {
			int which = offset & 1;
			u8 tmp[2];
            dm_read_eeprom_word(dev, offset / 2, tmp);
			tmp[which] = *data;
            dm_write_eeprom_word(dev, offset / 2, tmp); 
			mdelay(10);
			done = 1;
		} else {
		dm_write_eeprom_word(dev, offset / 2, data);  
			done = 2;
		}
		data += done;
		offset += done;
		len -= done;
	}
	return 0;
}

static int dm9620_get_eeprom_len(struct net_device *dev)
{
	return DM_EEPROM_LEN;
}

static int dm9620_get_eeprom(struct net_device *net,
			     struct ethtool_eeprom *eeprom, u8 * data)
{
	struct usbnet *dev = netdev_priv(net);
	__le16 *ebuf = (__le16 *) data;
	int i;

	/* access is 16bit */
	if ((eeprom->offset % 2) || (eeprom->len % 2))
		return -EINVAL;

	for (i = 0; i < eeprom->len / 2; i++) {
		if (dm_read_eeprom_word(dev, eeprom->offset / 2 + i,
					&ebuf[i]) < 0)
			return -EINVAL;
	}
	return 0;
}

static int dm9620_mdio_read(struct net_device *netdev, int phy_id, int loc)
{
	struct usbnet *dev = netdev_priv(netdev);
	__le16 res;
	u8 tmp;

	// Support 'EXT MII'
	// Since REG DM_NET_CTRL/DM_NET_STATUS is the final result,
	// No matter internal PHY or EXT MII.
	#if LNX_DRV_EXT_PHY_HAS_MDIO == 0
	dm_read_reg(dev, DM_NET_CTRL, &tmp);
	if (tmp&0x80) //'EXT MII'
	{
		if (loc==0) //bmcr
		{
			res= 0x0000;
			if (tmp&0x08) // duplex mode
				res |= 0x100;
			dm_read_reg(dev, 0x01, &tmp); //= DM_NET_STATUS
			if (!(tmp&0x80)) // speed 10/100
				res |= 0x2000;
			return le16_to_cpu(res);
		}
		if (loc==1) //bmsr
		{
			res= 0x7849;
			dm_read_reg(dev, 0x01, &tmp); //= DM_NET_STATUS
			if (tmp&0x40) // linl status
				res= 0x784D;
			return le16_to_cpu(res);
		}
		if (loc==4) //advertise
		{
			res= 0x05e1;
			return le16_to_cpu(res);
		}
		if (loc==5) //lpa
		{
			res= 0x45e1;
			return le16_to_cpu(res);
		}
	}
	#endif

	dm_read_shared_word(dev, phy_id, loc, &res);
	return le16_to_cpu(res);
}

static void dm9620_mdio_write(struct net_device *netdev, int phy_id, int loc,
			      int val)
{
	struct usbnet *dev = netdev_priv(netdev);
	__le16 res = cpu_to_le16(val);
	int mdio_val;  
	dm_write_shared_word(dev, phy_id, loc, res);
	mdelay(1);
	mdio_val = dm9620_mdio_read(netdev, phy_id, loc);
}

static void dm9620_get_drvinfo(struct net_device *net,
			       struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
	info->eedump_len = DM_EEPROM_LEN;
}

static u32 dm9620_get_link(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return mii_link_ok(&dev->mii);
}

static int dm9620_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

static void
dm9620_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 opt;

	if (dm_read_reg(dev, DM_WAKEUP_CTRL, &opt) < 0) {
		wolinfo->supported = 0;
		wolinfo->wolopts = 0;
		return;
	}
	wolinfo->supported = WAKE_PHY | WAKE_MAGIC;
	wolinfo->wolopts = 0;

	if (opt & DM_LINKEN)
		wolinfo->wolopts |= WAKE_PHY;
	if (opt & DM_MAGICEN)
		wolinfo->wolopts |= WAKE_MAGIC;
}

static int
dm9620_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 opt = 0;

	if (wolinfo->wolopts & WAKE_PHY)
		opt |= DM_LINKEN;
	if (wolinfo->wolopts & WAKE_MAGIC)
		opt |= DM_MAGICEN;

	dm_write_reg(dev, DM_NET_CTRL, 0x48);  // enable WAKEEN 
//	dm_write_reg(dev, 0x92, 0x3f); //keep clock on Hank Jun 30
	return dm_write_reg(dev, DM_WAKEUP_CTRL, opt);
}   

#if LNX_KERNEL_v511
static int dm9620_get_link_ksettings(struct net_device * dev,
  struct ethtool_link_ksettings * cmd) {
  struct usbnet *udev = netdev_priv(dev);
  mii_ethtool_get_link_ksettings( & udev->mii, cmd);
  return 0;
}
static int dm9620_set_link_ksettings(struct net_device * dev,
  const struct ethtool_link_ksettings * cmd) {
  struct usbnet *udev = netdev_priv(dev);
  return mii_ethtool_set_link_ksettings( & udev->mii, cmd);
}
#endif

static struct ethtool_ops dm9620_ethtool_ops = {
	.get_drvinfo	= dm9620_get_drvinfo,
	.get_link		= dm9620_get_link,
	.get_msglevel	= usbnet_get_msglevel,
	.set_msglevel	= usbnet_set_msglevel,
	.get_eeprom_len	= dm9620_get_eeprom_len,
	.get_eeprom		= dm9620_get_eeprom,
	.set_eeprom		= dm9620_set_eeprom,
#if LNX_KERNEL_v511
	.get_link_ksettings = dm9620_get_link_ksettings,
	.set_link_ksettings = dm9620_set_link_ksettings,
#else 
	.get_settings	= usbnet_get_settings,
	.set_settings	= usbnet_set_settings,
#endif
	.nway_reset		= usbnet_nway_reset,
	.get_wol		= dm9620_get_wol,
	.set_wol		= dm9620_set_wol,
};

/*static void dm9620_start_carriering(struct dm96xx_priv *priv)
{
	priv->summCarrierTgl= 0; //CARRIER_TGL_CTRL_S; //0
}
static void dm9620_stop_carriering(struct dm96xx_priv *priv)
{
	priv->summCarrierTgl= 5; //CARRIER_TGL_CTRL_E; //CARRIER_TGL_CONTROL;
}
static void dm9620_step_carriering(struct dm96xx_priv *priv)
{
	priv->summCarrierTgl++;
}*/

 static int dm9620_open(struct net_device *net)
 {
    struct usbnet *dev = netdev_priv(net);
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	
    printk("\n");
	printk("+[dm96 open.s]- (TRIP %d on-ticks %04d)\n", priv->IP_toggle_count, priv->IP_dc); 
	
	/* work-around for 9601 mode, 9620 mode */
	//dm9620_start_carriering(priv); //priv->summCarrierTgl= 0;
	
  //priv->IP_done= 0;
 	priv->IP_TOGG= 0;
 	priv->IP_toggle_count= 0;
	priv->IP_dc= 0;
 	priv->allow_defer_happen= 1;
 	CTLPRINT_OPEN(); //Control_printing
 	
 	/* Need EP3I */
 	printk("+[dm96 open]-   (play get link_up & carrier-on)\n"); 
 	
 	//.NET_CARRIER_ON(net); //'netif_carrier_on'
 	//printk("-[dm96 open]- (play get link_down & carrier-off)\n"); 
 	//.NET_CARRIER_OFF(net);
 	
	printk("+[dm96 open.e]- (TRIP %d on-ticks %04d)\n", priv->IP_toggle_count, priv->IP_dc); 
    printk("\n");
 	return usbnet_open(net);
 } 
 static int dm9620_stop (struct net_device *net)
 {
 	int ret= usbnet_stop(net);
 	netif_carrier_off(net); //'NET_CARRIER_OFF'
 	printk("+[dm96 close.s]- (now)\n");
 	printk("+[dm96 close.e]- (carrier off)\n"); 
 	return ret;
 } // chiphd-work: 

static void dm9620_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	/* We use the 20 byte dev->data for our 8 byte filter buffer
	 * to avoid allocating memory that is tricky to free later */
	u8 *hashes = (u8 *) & dev->data;
	u8 rx_ctl = 0x31;

	memset(hashes, 0x00, DM_MCAST_SIZE);
	hashes[DM_MCAST_SIZE - 1] |= 0x80;	/* broadcast address */

	if (net->flags & IFF_PROMISC) {
		rx_ctl |= 0x02;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,33)	
	} else if (net->flags & IFF_ALLMULTI ||  netdev_mc_count(net) > DM_MAX_MCAST) {
		rx_ctl |= 0x8;
	} else if (!netdev_mc_empty(net)) {
            struct netdev_hw_addr *ha;
 
         netdev_for_each_mc_addr(ha, net) {
              u32 crc = crc32_le(~0, ha->addr, ETH_ALEN) & 0x3f;
              hashes[crc>>3] |= 1 << (crc & 0x7);
		}
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,33)
  	} else if (net->flags & IFF_ALLMULTI || net->mc_count > DM_MAX_MCAST) {
		rx_ctl |= 0x08;
	} else if (net->mc_count) {
		struct dev_mc_list *mc_list = net->mc_list;
		int i;

		for (i = 0; i < net->mc_count; i++, mc_list = mc_list->next) {
			u32 crc = crc32_le(~0, mc_list->dmi_addr, ETH_ALEN) & 0x3f;
                        hashes[crc>>3] |= 1 << (crc & 0x7);
		} 
#endif		
	}

	dm_write_async(dev, DM_MCAST_ADDR, DM_MCAST_SIZE, hashes);
	dm_write_reg_async(dev, DM_RX_CTRL, rx_ctl);
}

 static void __dm9620_set_mac_address(struct usbnet *dev)
 {
	dm_write_async(dev, DM_PHY_ADDR, ETH_ALEN, dev->net->dev_addr);
 }
 
 static int dm9620_set_mac_address(struct net_device *net, void *p)
 {
	 struct sockaddr *addr = p;
	 struct usbnet *dev = netdev_priv(net);
	 int i;
 
#if 1
	 printk("[dm96] Set mac addr %pM\n", addr->sa_data);  // %x:%x:...
	 printk("[dm96] ");
	 for (i=0; i<net->addr_len; i++)
	 	printk("[%02x] ", addr->sa_data[i]);
	 printk("\n");
 #endif
 
     if (!is_valid_ether_addr(addr->sa_data)) {
             dev_err(&net->dev, "not setting invalid mac address %pM\n",
                                                             addr->sa_data);
             return -EINVAL;
     }

     memcpy(net->dev_addr, addr->sa_data, net->addr_len);
     __dm9620_set_mac_address(dev);

     return 0;
 }

static const struct net_device_ops vm_netdev_ops= {
        .ndo_open               = dm9620_open, // usbnet_open,  
        .ndo_stop               = dm9620_stop, //usbnet_stop,  
        .ndo_start_xmit         = usbnet_start_xmit, 
        .ndo_tx_timeout         = usbnet_tx_timeout, 
        .ndo_change_mtu         = usbnet_change_mtu, 
        .ndo_validate_addr      = eth_validate_addr, 
	    .ndo_do_ioctl	    	= dm9620_ioctl,   
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	    .ndo_set_rx_mode        = dm9620_set_multicast,   
#else
	    .ndo_set_multicast_list = dm9620_set_multicast,   
#endif
	    .ndo_set_mac_address    = dm9620_set_mac_address,  
};

//= usbnet_get_endpoints(struct usbnet *dev, struct usb_interface *intf)
int dm9620_get_endpoints(struct usbnet *dev, struct usb_interface *intf)
{
	return usbnet_get_endpoints(dev, intf);
}

static int dm9620_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret,mdio_val,i;
	struct dm96xx_priv* priv;
	u8 temp;
	u8 tmp;
	u8 mac[ETH_ALEN];

	ret = dm9620_get_endpoints(dev, intf);	
	if (ret)
		goto out;

    /* bind version info.s */
    printk("\n");
	printk("[dm962] Linux Driver = %s\n", LNX_DM9620_VER_STR);
	printk("\n");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
#endif
	dev->net->netdev_ops = &vm_netdev_ops; // new kernel 2.6.31  (20091217JJ)
	dev->net->ethtool_ops = &dm9620_ethtool_ops;
	dev->net->hard_header_len += DM_TX_OVERHEAD;
	dev->hard_mtu = dev->net->mtu + dev->net->hard_header_len;
	
	dev->rx_urb_size = dev->net->mtu + ETH_HLEN + DM_RX_OVERHEAD+1; // ftp fail fixed

	dev->mii.dev = dev->net;
	dev->mii.mdio_read = dm9620_mdio_read;
	dev->mii.mdio_write = dm9620_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;
	dev->mii.phy_id = DM9620_PHY_ID;
	
	/* JJ1 */
	if ( (ret= dm_read_reg(dev, 0x29, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0x29 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0x29 fail-func-return %d\n", ret);
		
	if ( (ret= dm_read_reg(dev, 0x28, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0x28 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0x28 fail-func-return %d\n", ret);
		
	if ( (ret= dm_read_reg(dev, 0x2b, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0x2b 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0x2b fail-func-return %d\n", ret);
	if ( (ret= dm_read_reg(dev, 0x2a, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0x2a 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0x2a fail-func-return %d\n", ret);

	/* reset */
	dm_write_reg(dev, DM_NET_CTRL, 1);
	udelayEx(20);
	
	//Stone add Enable "MAC layer" Flow Control, TX Pause Packet Enable and 
	dm_write_reg(dev, DM_FLOW_CTRL, 0x29);
	//Stone add Enable "PHY layer" Flow Control support (phy register 0x04 bit 10)
	temp = dm9620_mdio_read(dev->net, dev->mii.phy_id, 0x04);  // re-do later
	dm9620_mdio_write(dev->net, dev->mii.phy_id, 0x04, temp | 0x400);  // re-do later
	
	/* WinToAndroid_20160801 */
	dm_write_reg(dev, DM_SMIREG, 0x80);
	
	/* Add V1.1, Enable auto link while plug in RJ45, Hank July 20, 2009*/
	dm_write_reg(dev, USB_CTRL, 0x20);
	
	/* read MAC */
	if (dm_read_regs(dev, DM_PHY_ADDR, ETH_ALEN, mac) < 0) { //'dev->net->dev_addr'
		printk(KERN_ERR "Error reading MAC address\n");
		ret = -ENODEV;
		goto out;
	}
#if 1
	if (is_valid_ether_addr(mac))
		memcpy(dev->net->dev_addr, mac, ETH_ALEN);
	else {
		printk(KERN_WARNING "No valid MAC address in EEPROM, using %pM\n", dev->net->dev_addr);
		__dm9620_set_mac_address(dev); //'dev->net->dev_addr'
	}
#endif

#if 1
	 printk("[dm96] Chk mac addr %pM\n", dev->net->dev_addr);  // %x:%x...
	 
	 printnb_init();
	 printnb("[dm96] ");
	 for (i=0; i<ETH_ALEN; i++)
	 	printnb("[%02x] ", dev->net->dev_addr[i]);
	 printnb("\n");
#endif

	/* read SMI mode register */
	 priv = dev->driver_priv = kmalloc(sizeof(struct dm96xx_priv), GFP_ATOMIC);
	 if (!priv) {
		dm9620_err(dev,"Failed to allocate memory for dm96xx_priv");
		ret = -ENOMEM;
		goto out;
	 }

    #if 1
    for (i=0; i<64; i++)
      dm_read_eeprom_word(dev, i, &priv->mdat[i<<1]);
    
    printnb_init(); // more once do this init is OK.
    for (i=0; i<64; i++){
      if (!(i%8)) {
		  printnb("\n[dm96] ");
	  }
	  printnb("%02x %02x ", priv->mdat[i<<1], priv->mdat[(i<<1)+1]);
    }
    printnb("\n");
    #endif

    /* v2.59.3 (render eeprom if need) WORD3 render-II */
    /* v2.59.3 (render eeprom if need) WORD7 render-I */
    /* v2.59.3 (render eeprom if need) WORD11 render-II */
    /* v2.59.3 (render eeprom if need) WORD12 render-II */

	//#if 1
    //priv->opnum_op= 0;
    //priv->opkey_input[0]= priv->opkey_input[1]= priv->opkey_input[2]= 0;
    //priv->save_input= 0;
    //priv->block_input[0]= priv->block_input[1]= 0;
    //priv->IP_conf_track_bas[0]= priv->IP_conf_track_bas[1]= 0;
	//#endif
    
    #if 1
    printnb_init(); // more once do this init is OK.
    for (i=0; i<16; i++){ //13
      if (!(i%8)) {
		printnb("\n[dm96] ");
      }
      dm_read_eeprom_word(dev, i, priv->mdat);
	  printnb("%02x %02x ", priv->mdat[0], priv->mdat[1]);
    }
    printnb("\n");
    #endif

    printk("\n");
    
	/* work-around for 9620 mode */
	dm_read_reg(dev, 0x5c, &temp); 
	priv->tx_fix_mod = temp;
	priv->IP_TOGG= 0;
	priv->IP_toggle_count= 0;
	priv->IP_dc= 0;
	priv->IP_di= 0;
	priv->Force_mode_magic= 0;
	priv->defer_on_looping= priv->coming_lnk_defer_on_ct= 0;
	priv->coming_link_defer_on= priv->summUsbErr= 0;
	priv->summPwOnTgl= 0; //1;
	priv->sav_lpa= 0;
	
	printk(KERN_WARNING "[dm96] DM9_NREV= %d\n", priv->tx_fix_mod);

  /*printk("[dm96] Fixme: work around for 9620 mode\n");*/
  /*printk("[dm96] Add tx_fixup() debug...\n");*/
	
	dm_write_reg(dev, DM_MCAST_ADDR, 0);     // clear data bus to 0s
	dm_read_reg(dev, DM_MCAST_ADDR, &temp);  // clear data bus to 0s
	ret = dm_read_reg(dev, DM_SMIREG, &temp);   // Must clear data bus before we can read the 'MODE9620' bit

	priv->flg_txdbg= 0; //->flag_fail_count= 0;

	if (ret>=0) ret= 0; //JJ.CHK
	if (ret<0)
		printk(KERN_ERR "[dm96] Error read SMI register\n");
	else 
		priv->mode_9620 = temp & DM_MODE9620;

	printk(KERN_WARNING "[dm96] 9620 Mode = %d\n", priv->mode_9620);
	
	priv->mode_ext_phy = 0;
	priv->mode_ext_link = 0;
	priv->mode_ext_phyid1 = 0xffff;
	priv->status_on_trigger = TRIGGER_IDLE;
	priv->status_carrier_ok = 0;
	priv->linkrst_carrier_ok = 0;
	
#if LNX_DRV_SIMPLE_PHY_AND_EXT_PHY
	dm_read_reg(dev, DM_NET_CTRL, &temp);
	if (temp&0x80) //'EXT MII'
		priv->mode_ext_phy = 1;
#else //[Simulate ext.phy]
	priv->mode_ext_phy = 1;
#endif
	printk(KERN_WARNING "[dm96] 9620 EXT_PHY = %d\n", priv->mode_ext_phy);
	if (priv->mode_ext_phy)	{
		printk(KERN_WARNING "[dm96] 9620 EXT_LINK = %d\n", priv->mode_ext_link);
	}
	
	dm_read_reg(dev, DM_TXRX_M, &temp);  // Need to check the Chipset version (register 0x5c is 0x02?)
	if (temp == 0x02)
	{
	 dm_read_reg(dev, 0x3f, &temp);
	 temp |= 0x80; 
	 dm_write_reg(dev, 0x3f, temp);
   }
  
	 /*
	 // cdc-control
	 dm_read_reg(dev, DM_SMIREG, &temp);
	  printk(KERN_WARNING "[dm96] 9620 REG91H (DM_SMIREG= Get 0x%02x, Xet xo 0xHH)\n", temp);
	 //dm_write_reg(dev, DM_SMIREG, temp &0xef);
      //printk(KERN_WARNING "[dm96] 9620 REG91H (DM_SMIREG= Get 0x%02x, Set to 0x%02x)\n", temp, temp &0xef);
	 dm_read_reg(dev, DM_SMIREG, &temp);
      printk(KERN_WARNING "[dm96] 9620 REG91H (DM_SMIREG= Get 0x%02x)\n", temp);*/

    /* bind version info.e */
     printk("\n");
	 printk("([dm962] Linux Driver = %s)\n", LNX_DM9620_VER_STR);
	 //printk("([dm962] Linux Driver OEM %s)\n", CONF_OEM);
     //printk("([dm962] Linux Driver INC %d EP3-INTVL 0x%02X)\n", priv->IP_conf_inc, WD12_EP3I); // 'CONF_INC'
	 printk("\n");
	 
	/* power up phy */
	dm_write_reg(dev, DM_GPR_CTRL, 1);
	dm_write_reg(dev, DM_GPR_DATA, 0);

	/* Init tx/rx checksum */
#if DM_TX_CS_EN
	dm_write_reg(dev, DM_TX_CRC_CTRL, 7);
#endif 
	dm_write_reg(dev, DM_RX_CRC_CTRL, 2);

	/* receive broadcast packets */
	dm9620_set_multicast(dev->net);
	dm9620_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR, BMCR_RESET);

	/* Hank add, work for comapubility issue (10M Power control) */	
	dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, 0x800); //0x0800,0x0820 are the same which is auto-mdix
	mdio_val = dm9620_mdio_read(dev->net, dev->mii.phy_id, PHY_SPEC_CFG);
	
	dm9620_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE,
			  ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
			  
	dm_read_reg(dev, USB_CTRL, &temp);
	printk(KERN_WARNING "[dm96] 9620 REGF4H (USB_CTRL= Get 0x%02x) (need 0x20 to have USB INT)\n", temp);
      
	mii_nway_restart(&dev->mii); 
	
out:
	return ret;
}

void dm9620_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	struct dm96xx_priv* priv= dev->driver_priv;

    /* unbind version info.e */
	printk("([dm962] Linux Driver = %s)\n", LNX_DM9620_VER_STR);
	
	//printk("([dm962] Linux Driver OEM %s)\n", CONF_OEM);
    //printk("([dm962] Linux Driver INC %d EP3-INTVL 0x%02X)\n", priv->IP_conf_inc, WD12_EP3I); //'CONF_INC'
    
    //printk("EXIT defr [because %d/(%d)] /%d\n", priv->defer_on_looping, 1, DEFER_ON_CONTROL);
    //printk("EXIT TGGL [because %d/(%d)] /%d\n", priv->IP_dc, priv->IP_conf_inc, 2250);
    
   //printk("flag_fail_count  %lu\n", (long unsigned int)priv->flag_fail_count);
	printk("flg_txdbg  %lu\n", (long unsigned int)priv->flg_txdbg);
	kfree(dev->driver_priv); // displayed dev->.. above, then can free dev 

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
	printk("rx_length_errors %lu\n",dev->net->stats.rx_length_errors);
	printk("rx_over_errors   %lu\n",dev->net->stats.rx_over_errors  );
	printk("rx_crc_errors    %lu\n",dev->net->stats.rx_crc_errors   );
	printk("rx_frame_errors  %lu\n",dev->net->stats.rx_frame_errors );
	printk("rx_fifo_errors   %lu\n",dev->net->stats.rx_fifo_errors  );
	printk("rx_missed_errors %lu\n",dev->net->stats.rx_missed_errors);	
#else
	printk("rx_length_errors %lu\n",dev->stats.rx_length_errors);
	printk("rx_over_errors   %lu\n",dev->stats.rx_over_errors  );
	printk("rx_crc_errors    %lu\n",dev->stats.rx_crc_errors   );
	printk("rx_frame_errors  %lu\n",dev->stats.rx_frame_errors );
	printk("rx_fifo_errors   %lu\n",dev->stats.rx_fifo_errors  );
	printk("rx_missed_errors %lu\n",dev->stats.rx_missed_errors);	
#endif
	printk("\n");
}

static int dm9620_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	u8 status;
	int len;
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;

	/* 9620 format:
	   b0: rx status
	   b1: packet length (incl crc) low
	   b2: packet length (incl crc) high
	   b3..n-4: packet data
	   bn-3..bn: ethernet crc
	 */

	/* 9620 format:
	   one additional byte then 9620 : 
	   rx_flag in the first pos
	 */

	if (unlikely(skb->len < DM_RX_OVERHEAD_9601)) {   // 20090623
		dev_err(&dev->udev->dev, "unexpected tiny rx frame\n");
		return 0;
	}

	if (priv->mode_9620) {
		/* mode 9620 */

		if (unlikely(skb->len < DM_RX_OVERHEAD)) {  // 20090623
			dev_err(&dev->udev->dev, "unexpected tiny rx frame\n");
			return 0;
		}
	
		status = skb->data[1];
		len = (skb->data[2] | (skb->data[3] << 8)) - 4;
		
		if (unlikely(status & 0xbf)) {
			if (status & 0x01) dev->net->stats.rx_fifo_errors++;
			if (status & 0x02) dev->net->stats.rx_crc_errors++;
			if (status & 0x04) dev->net->stats.rx_frame_errors++;
			if (status & 0x20) dev->net->stats.rx_missed_errors++;
			if (status & 0x90) dev->net->stats.rx_length_errors++;
			return 0;
		}

		skb_pull(skb, 4);
		skb_trim(skb, len);
	}
	else { /* mode 9620 (original driver code) */
		status = skb->data[0];
		len = (skb->data[1] | (skb->data[2] << 8)) - 4;
		
		if (unlikely(status & 0xbf)) {
			if (status & 0x01) dev->net->stats.rx_fifo_errors++;
			if (status & 0x02) dev->net->stats.rx_crc_errors++;
			if (status & 0x04) dev->net->stats.rx_frame_errors++;
			if (status & 0x20) dev->net->stats.rx_missed_errors++;
			if (status & 0x90) dev->net->stats.rx_length_errors++;
			return 0;
		}

		skb_pull(skb, 3);
		skb_trim(skb, len);
	}

	return 1;
}

#define TX_LEN_E  (1<<0)  //EVEN, No action
#define TX_LEN_O  (1<<1)  //ODD, Odd to even workaround
#define TX_LEN_F  (1<<2)  //FULL, Full payload workaround
u8 TxStyle(int len, unsigned full_payload){
  u8 s= (len%2)? TX_LEN_O: TX_LEN_E;
  len= ((len+1)/2)*2;
  len += 2;
  if ((len % full_payload)==0)
    s |= TX_LEN_F;
  return s;
}
struct sk_buff *TxExpend(struct dm96xx_priv* priv, u8 ts, struct sk_buff *skb, gfp_t flags)
{
    int newheadroom= 2, newtailroom= 0;  
    if (ts&TX_LEN_O) newtailroom++;
    if (ts&TX_LEN_F) newtailroom += 2;
    if (skb_headroom(skb) >= newheadroom) newheadroom= 0; // head no need expend
    if (skb_tailroom(skb) >= newtailroom) newtailroom= 0; // tail no need expend
    if (newheadroom || newtailroom){
		struct sk_buff *skb2;
		skb2 = skb_copy_expand(skb, newheadroom, newtailroom, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb){
			printk("[dm96-TxRound].%d expend copy fail, for head, tail= %d, %d\n", priv->flg_txdbg++, newheadroom, newtailroom);
			return NULL;
		}
		printk("[dm96-TxRound].%d expend copy OK, for head, tail= %d, %d\n", priv->flg_txdbg++, newheadroom, newtailroom);
    }
    return skb;
}
static struct sk_buff *dm9620_tx_fixup(struct usbnet *dev, struct sk_buff *skb,
				       gfp_t flags)
{
	int len;
    int newheadroom, newtailroom;  
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;

	/* format:
	   b0: packet length low
	   b1: packet length high
	   b3..n: packet data
	*/

	len = skb->len;

	  if (priv->tx_fix_mod<3)
	  {
	    //;DM9620-E4,E5, and E6
		/* usbnet adds padding 1 byte if odd len */
		/* usbnet adds padding 2 bytes if length is a multiple of packet size
		   if so, adjust length value in header */
	     u8 TS= TxStyle(len, dev->maxpacket);
	     if (!(skb= TxExpend(priv, TS, skb, flags))) return NULL;
	
	     if (TS & TX_LEN_F) len += 2;
	
	     newheadroom= 2; //2
	     newtailroom= 0; //0, 1, 2, or 3
	     if (TS & TX_LEN_O) newtailroom++;
	     if (TS & TX_LEN_F) newtailroom += 2;
	     
	   //if (TS & TX_LEN_O) printk("[dm96-TxRound].%d for LEN_ODD tail_room +1, rslt add %d\n", priv->flg_txdbg, newtailroom);
	   //if (TS & TX_LEN_F) printk("[dm96-TxRound].%d for LEN_PLOAD tail_room +2, rslt add %d\n", priv->flg_txdbg, newtailroom);
	   //if (TS & TX_LEN_F) printk("[dm96-TxRound].%d for LEN_PLOAD data_len +2, len from %d to %d\n", priv->flg_txdbg, len-2, len);
	     if (TS & (TX_LEN_O|TX_LEN_F)) priv->flg_txdbg++;
	
		__skb_push(skb, newheadroom); //2 bytes,for data[0],data[1]
	    __skb_put(skb, newtailroom); //0, 1, 2, or 3 bytes (for tailer), 
	                                 //Note: 0, NOTHING
	                                 //Note: 1, Odd to even WORKAROUND.
	                                 //Note: 2 or 3, the condition is full payload,
	                                 // This is the add more two bytes WORKAROUND
	                                 // for bulkout and buffLen.
	  }
	  else 
	  {
	    //;DM9620-E7
		if (skb_headroom(skb) < DM_TX_OVERHEAD) {
			struct sk_buff *skb2;
			skb2 = skb_copy_expand(skb, DM_TX_OVERHEAD, 0, flags);
			dev_kfree_skb_any(skb);
			skb = skb2;
			if (!skb)
				return NULL;
		}
	
		newheadroom= 2; //2
		__skb_push(skb, newheadroom);  //2 bytes, for data[0],data[1]
	  }

	skb->data[0] = len;
	skb->data[1] = len >> 8;

	/* hank, recalcute checksum of TCP */
	return skb;
}

static u16 Spec_cfg_value(struct usbnet *dev)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	u16 mdio_val= 0x0800;
	//also [priv->status_on_trigger == TRIGGER_TO_TOGGLE]
	if (priv->Force_mode_magic==FORCE_MODE_MAGIC){ /* Defer Case for TOGGLE */
		if (priv->IP_TOGG==0) // toggle.0
		{	
			//mdio_val &= 0xffdf;
			//mdio_val |= 0x10;                    // 0x0890.0x0810
			mdio_val= 0x0810;					 // final-to-be
		}
		else  // toggle.1
		{
			//mdio_val |= 0x20;
			//mdio_val |= 0x10;                    // 0x0830
			mdio_val= 0x0830;					 // final-to-be
		}
		priv->IP_dc= 0;
		priv->IP_di= 0;
		priv->IP_TOGG= 1 - priv->IP_TOGG;
		if (priv->summPwOnTgl)
		 priv->summPwOnTgl++;
	}
	return mdio_val;
}

static int dm9620_toggle(struct usbnet *dev)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;

	/* (1) */
	priv->coming_lnk_defer_on_ct= priv->summUsbErr= 0;
	
	/* (2) */
	#if WORK_INFINIT_TOGG
	if (priv->IP_toggle_count >= TOTAL_CNT_TOGG) {  //;  //last to do
		priv->IP_toggle_count= 0;
		priv->IP_dc= 0;
		priv->IP_di= 0;
	}
	#endif
	
	/* (3) Toggle tasks */
	if (priv->IP_toggle_count < TOTAL_CNT_TOGG){
		//(priv->IP_conf_inc >> 1); //'CONF_INC >> 1';
		// when priv->IP_conf_inc is 126, (priv->IP_conf_inc >> 1) is 63.
		priv->IP_dc += 63; 
		 //from_2200
		 //from 2250
		 //chiphd(Intel ¬O) 7875
		if (priv->IP_dc > 7875){ 
			priv->IP_toggle_count++;
			
			priv->IP_dc= 0; // TEMP.WAIT.PURPOSE
			priv->IP_di= 0;
		    //priv->IP_done= 1 - priv->IP_done;
		    return 1;
		}
	}
	return 0;
}

static void dm9620_status_on(struct usbnet *dev, struct dm96xx_priv *priv, u8 nsr)
{
	int link= !!(nsr & 0x40);
	
	/* reset-oppsite-varibles */
	if (!link)
		priv->defer_on_looping= 0;
	
	/* togg */
	if (!link){
		
		/* [ Before this always return (no cable connected operate power-on) & carrier_on in xxx_open ] */
		if (1) //(!dm9620_icon_carriering(priv))
		{
		 if (priv->status_carrier_ok){
		  //.NET_CARRIER_OFF(dev->net);
		  //printk("++++++[dm962]+++++ [drv_carrier-off togg-early]"); 
		  //if (!CONF_OK_PNT_PLUS1)
		  //  printk(" <CLEAN DBGMOD>"); 
          //printk("\n"); 
		 }
		}

		if (dm9620_toggle(dev)){
		  priv->allow_defer_happen= 0;
		  priv->Force_mode_magic= FORCE_MODE_MAGIC;
		  
		  priv->status_on_trigger = TRIGGER_TO_TOGGLE;
		  if (priv->sav_lpa==0) { //'priv->sav_lpa' was read every previous "dm_link_reset"
			if (priv->mode_ext_phy) //Only print this for external phy mode 
				TGGLPRINTX("[EVENT %d][%s][%d] dm9 ID1 %04x\n", TRIGGER_TO_TOGGLE, STR_TOGGLE, // Control_printing
					priv->status_carrier_ok, priv->mode_ext_phyid1);
		  }
		  usbnet_defer_kevent (dev, EVENT_LINK_RESET); /* Defer Case for TOGGLE */
		  return;
		}
	}
	
    /* --- +carrier-task+ --- */
	/* two: one for keep-link, one for change state actions */
	
	if ( priv->coming_link_defer_on == 0){
		
		/* mantain keep-link-up */
		if (priv->status_carrier_ok == link) {
			/* be 'link' */
			if (link) {
			//#if WORK_INFINIT_REG82H
				if (priv->coming_lnk_defer_on_ct >= 500) {  //;  //last to do
					priv->coming_lnk_defer_on_ct= 0;
					//if (CONF_OK_PNT_PLUS1)
					//  printk("~~~~~ ~~~~~ ~~~~~ [dm96.LNK]- [infinite] re.peat ~~~~~ ~~~~~ ~~~~~\n"); 
				}
			//#endif			
			
				if (priv->coming_lnk_defer_on_ct<500){
					
					priv->defer_on_looping++;
					
					if (priv->defer_on_looping >= DEFER_ON_CONTROL)
					{
						#if WORK_LNK_OKP
						//<dm9620_print(dev, "MAKE defer [because %d/(%d)/%d]\n", priv->defer_on_looping, 1, DEFER_ON_CONTROL); >

						//#define CONF_OK_PNT_PLUS1 (oem[CONF_N].cnf.ok_to_print && ((priv->IP_conf_inc&1)))
						//printk("[U]priv->coming_lnk_defer_on_ct %d\n", priv->coming_lnk_defer_on_ct);
						//printk("[U]oem[CONF_N].cnf.ok_to_print && ((priv->IP_conf_inc&1)): [CONF_N] %d, .cnf.ok_to_print %d ->IP_conf_inc %d\r\n", 
						//	CONF_N, oem[CONF_N].cnf.ok_to_print, priv->IP_conf_inc);
						printk("[U] dm9\n");
						#endif
						
						 /* defer for 'dbg' show REG82H acc */
						 //	priv->defer_on_looping= 0;
						
				  		priv->coming_lnk_defer_on_ct++;
						priv->allow_defer_happen= 0;
						
						priv->status_on_trigger = TRIGGER_LOOPING_150;
						#if WORK_LNK_OKP
						printk("[EVENT %d][%s][%d] dm9 [LoopingGT150]\n", TRIGGER_LOOPING_150, STR_LOOPING, priv->status_carrier_ok); //" for link= %d on", link
						#endif
						usbnet_defer_kevent (dev, EVENT_LINK_RESET); /* Defer Case for DBG REG82H acc */
					}
				}
			}
			/* no-change, link-state, return */
			return;
		}
		
		/* (2/2) change state actions */ 
		if (priv->status_carrier_ok != link) {
			
			/* be 'link' */
			if (link) {
				//printk("[D] dm9\n");
				priv->coming_link_defer_on= 1; 
				priv->allow_defer_happen= 0;
			  //_Netif_carrier_on(dev->net); (to do later)
				priv->status_on_trigger = TRIGGER_LINKUP_FOUND;
			  //printk("[defer_drv_carrier-off-to-on] dm9\n"); 
				printk("[EVENT %d][%s][carrier-off] dm9 link up found\n", TRIGGER_LINKUP_FOUND, STR_FOUND); //" for link= %d on", link
				usbnet_defer_kevent (dev, EVENT_LINK_RESET); /* Defer Case for Link-up carrier-on */
			}	
			return;
		}
	}
	else
	{
	    /* should be no-way */	
		priv->coming_link_defer_on++;
		if (!(priv->coming_link_defer_on % 8)) //64
		  printk("++++++[dm962].DBG+++++ Curr_EP3-Intvl 0x%02X [coming_link_defer_on]= %d\n", priv->desc_ep3i, priv->coming_link_defer_on); 
	}
}

static void dm9620_status_new(struct usbnet *dev, struct urb *urb)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	u8 *buf;
	int link;

	/* format:
	   b0: net status
	   b1: tx status 1
	   b2: tx status 2
	   b3: rx status
	   b4: rx overflow
	   b5: rx count
	   b6: tx count
	   b7: gpr
	*/
	
	if ((urb->actual_length < 8) || (priv->allow_defer_happen==0))
	{	  
	  if (priv->allow_defer_happen==0)
	  {
	  	priv->defer_count++;
	  	if (priv->defer_count > 768)
	  	{
	  		printk("dm9620_status_new: priv->allow_defer_happen == %d for (%d times)\n", priv->allow_defer_happen, priv->defer_count);
	  		priv->defer_count= 0;
	  	}
	  }
	}

	if (urb->actual_length < 8)
		return;
		
	/* This is for some device linux system 
	   which is very very 1ms ep3  
	 */
	if (priv->allow_defer_happen==0)	
		return;

	priv->status_carrier_ok = netif_carrier_ok(dev->net);
	
	buf = urb->transfer_buffer;
	if (priv->mode_ext_phy)
		buf[0] = priv->mode_ext_link ? 0x40 : 0x00;
	link= !!(buf[0] & 0x40);	
	 
	if (link == (int) priv->status_carrier_ok)
	{
	  if (priv->desc_ep3i_pntonce)
	  {
	  	printk("dm9: #PHYEXT %d PHYID %d phyid1 %02x \n", priv->mode_ext_phy, dev->mii.phy_id, priv->mode_ext_phyid1);
	  	printk("dm9: #PHYEXT %d PHYID %d phyid1 %02x, %s \n", priv->mode_ext_phy, dev->mii.phy_id, priv->mode_ext_phyid1,
	  		LNX_DM9620_VER_STR);
	  	printk("dm9: status ===[ status %d, netif_carrier %d ]===\n", link, priv->status_carrier_ok);
	  	priv->desc_ep3i_pntonce= 0;
	  }
	}
	      
	dm9620_status_on(dev, priv, buf[0]); //...//priv->status_on_trigger = TRIGGER_IDLE;
	
	//E3.3(1) @ chiphd
	/*if (1) //(!dm9620_icon_carriering(priv))
	{
		if (link) //if (link != (int) _netif_carrier_ok(dev->net))
		{
			if (!priv->status_carrier_ok) 
			{
				printk("dm9: %s #status ===[ Direct_Netif_carrier_off ]===\n", LNX_DM9620_VER_STR);
				.NET_CARRIER_OFF(dev->net);
			}
		}
		else
		{
			if (priv->status_carrier_ok)
			{
				printk("dm9: %s #status ===[ Direct_Netif_carrier_on ]===\n", LNX_DM9620_VER_STR);
				.NET_CARRIER_ON(dev->net);
			}
		}
	}*/
}

static void Trigg_Print(struct usbnet *dev, u16 mdio_val, u16 bmsr_val, int OKP)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv; 
	u16 lpa_val = priv->sav_lpa;
	char *str;
	str = STR_IDLE;
	if (priv->status_on_trigger == TRIGGER_TO_TOGGLE)
	str = STR_TOGGLE;               
	if (priv->status_on_trigger == TRIGGER_LOOPING_150)
	str = STR_LOOPING;              
	if (priv->status_on_trigger == TRIGGER_LINKUP_FOUND)
	str = STR_FOUND;
	    
	//also [priv->status_on_trigger == TRIGGER_TO_TOGGLE]
	if (!priv->linkrst_carrier_ok) { //'USED_IN_LINK_RESET_netif_carrier_ok(dev->net)'
		//[Temp Check code].s  
		if (lpa_val && priv->summPwOnTgl <= TGL_EXTRA_WRITE)
			CTLPRINT(" TRIGG %d dm9 lpa %04x\n", priv->status_on_trigger, lpa_val); // Control_printing
		//[Temp Check code]  .e
		
		if (mdio_val != 0x0800) {   
			if (!lpa_val || priv->summPwOnTgl > TGL_EXTRA_WRITE)
				TGGLPRINTX("[TRIGG %d][%s][-] dm9 lpa %04x\n", priv->status_on_trigger, str, lpa_val); // Control_printing
		
			//_CORRECT-TOGGLE-SET-TASK-JOB
			if (!lpa_val){
				TGGLPRINTX("[TRIGG %d][%s][x] dm9 bmr %04x SPEC_CFG %04x\n", priv->status_on_trigger, str, bmsr_val, mdio_val); // Control_printing
			}
			//_dm9620_EXTRA_TOGGLE
			if (priv->summPwOnTgl > TGL_EXTRA_WRITE){
				TGGLPRINTX("[TRIGG %d][%s][x] dm9 bmr %04x SPEC_CFG %04x (sumPwOnTgl %d > TGL_EXTRA_WR %d)\n", // Control_printing
					priv->status_on_trigger, str, bmsr_val, mdio_val, priv->summPwOnTgl, TGL_EXTRA_WRITE);
			}
			
			if (!lpa_val || priv->summPwOnTgl > TGL_EXTRA_WRITE)
				TGGLPRINT(".\n"); // Control_printing
		}
	}
	else if (OKP) {
		printk("[TRIGG %d][%s][u] dm9 lpa %04x bmsr %04x\n", priv->status_on_trigger, str, lpa_val, bmsr_val);
		printk(".\n");
	}
}

static void dm9620_EXTRA_TOGGLE(struct usbnet *dev, u16 mdio_val)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	//u16 mdio_val= (priv->IP_TOGG==0) ? 0x0810 : 0x0830;
	
    //Func: EXTRA-WRITE
	// Joseph Debug-code 2016-02-19
	// We need know if this condition, We can solve it or not.
	// This is a little conflict to the rule we ever get and like : "When lpa w/ value stop toggle."
	// Joseph Debug-code 2016-02-22
    // Fine-tune...(some logic wrong!!)
	if (priv->summPwOnTgl > TGL_EXTRA_WRITE) 
	{
		/* [Tell the chip]: 'TOGGLE-TASK' */
		dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, (int) mdio_val); /* [no carrier-(write mdi/mdix)] for every times. */
        printk(" DM9x: EXTRA-WRITE on not-zero lpa= %04x !!\n", priv->sav_lpa);	
		printk(" NSRr: (MDIREG %04x tgl %d > %d .We do-Write) !!\n", mdio_val, priv->summPwOnTgl, TGL_EXTRA_WRITE);	

        // [Clear to 0] So that next time lpa-dead again, should wait again 500 (or 18) FOR enter this EXTRA-WRITE again.
        // This is same effect to (priv->status_flag & SF_LNK) in 'dm9620_dfr_condition_restart()' in link-reset.
        //priv->summPwOnTgl= 0;
	}
}

static void Trigg_Command(struct usbnet *dev, u16 mdio_val)
{
	/* 'CORRECT-TOGGLE-SET-TASK-JOB' */
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv; 
	if (mdio_val!=0x0800){
		if (!priv->sav_lpa)
			dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, (int) mdio_val);
		else
			 dm9620_EXTRA_TOGGLE(dev, mdio_val);
		      /* ----------------------------------------------------
		         [Two times copied to upper code sections] 
		         if (..) printk(" (Det.tgl %d)\n", priv->summPwOnTgl); 
		         ----------------------------------------------------*/
	}
}

#if 0
//[Not well if call to do check media in this.]
static unsigned int dm9620_check_media(struct mii_if_info *mii,
										unsigned int ok_to_print,
										unsigned int init_media,
										bool carrier)
{
	unsigned int ret;
	ret = mii_check_media(mii, ok_to_print, init_media);
	if (carrier) //if (ret) ; 
		CTLPRINT_OPEN(); //Control_printing
	return ret; // returned 1 if the duplex mode changed
}
#endif

static int dm9620_link_reset(struct usbnet *dev)
{
	u16 bmsr_val, mdio_val; //phyid1, lpa_val, 
	unsigned int ok_to_print;
	struct ethtool_cmd ecmd;
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
			
	priv->linkrst_carrier_ok = netif_carrier_ok(dev->net);

	/* read keep */
	bmsr_val = (u16) dm9620_mdio_read(dev->net, dev->mii.phy_id, 0x01);
	bmsr_val = (u16) dm9620_mdio_read(dev->net, dev->mii.phy_id, 0x01);
	priv->mode_ext_link = (bmsr_val & 0x4) ? 1 : 0;
	
	if (priv->mode_ext_phy)
		priv->mode_ext_phyid1 = (u16) dm9620_mdio_read(dev->net, dev->mii.phy_id, 0x02);
	
	priv->sav_lpa = (u16) dm9620_mdio_read(dev->net, dev->mii.phy_id, 0x05);
	
	ok_to_print = (priv->linkrst_carrier_ok != priv->mode_ext_link) ? 1 : 0; //'USED_IN_LINK_RESET_netif_carrier_ok(dev->net)'																			
#if 1
	//MII_CHK_MEDIA_OKTOPRINT(&dev->mii, ok_to_print, 1, priv->linkrst_carrier_ok); //'mii_check_media'
	mii_check_media(&dev->mii, ok_to_print, 1);
	if (priv->linkrst_carrier_ok)
		CTLPRINT_OPEN();
#endif
	mii_ethtool_gset(&dev->mii, &ecmd);
	
	/* Defer to maintain cnf_inc into EEPROM */
	//if (bmsr_val & 0x04)
	//	CheckWrite_cnf_inc(dev);
	
	/* Determine 'mdio_val' */
	mdio_val= Spec_cfg_value(dev);         
	Trigg_Print(dev, mdio_val, bmsr_val, WORK_LNK_OKP);
	priv->status_on_trigger = TRIGGER_IDLE;
	
	if (priv->Force_mode_magic==FORCE_MODE_MAGIC){
		if (!priv->summPwOnTgl && priv->sav_lpa)
		   priv->summPwOnTgl= 1;
	}
	priv->Force_mode_magic= 0;
	
	Trigg_Command(dev, mdio_val);

    if (priv->summPwOnTgl){
		if (mdio_val==0x0800){
	        if (bmsr_val & 0x04){
		     printk("++++++[dm962]+++++ [ On - link_reset - get link_up...] [bmsr %04x lpa %04x]\n",
			  bmsr_val, priv->sav_lpa);  //update-Link Report
		     printk("++++++[dm962]+++++ [ On - Auto-MDIX running - Power_on: %d tgl (Acc)]\n", 
			  priv->summPwOnTgl);  //update-Link Report
	        }else
		     printk("++++++[dm962]+++++ [ On - link_reset - get link_down...] [bmsr %04x lpa %04x]\n",
			  bmsr_val, priv->sav_lpa);  //update-Link Report
		} else {
      		if (!priv->sav_lpa)
	      		printk(" (Det.tgl %d)\n", priv->summPwOnTgl);
	      	if (priv->summPwOnTgl > TGL_EXTRA_WRITE) 
	      		priv->summPwOnTgl= 0;
		}
    }

	if (priv->defer_on_looping>=DEFER_ON_CONTROL) /* defer for 'dbg' show REG82H acc */
	  priv->defer_on_looping= 0; //< printk("ON defer [clear %d/(%d)/%d]\n", priv->defer_on_looping, 1, DEFER_ON_CONTROL); >

	if (priv->coming_link_defer_on){ /* Defer Case for Link-up carrier-on */
	  priv->coming_link_defer_on= 0;
	  if (priv->summPwOnTgl)
		  printk("++++++[dm962]+++++ [OnDFR - deferred carrier-on] %d tgl (Acc) --> 0\n", priv->summPwOnTgl);  //update-Link Report
	  //.NET_CARRIER_ON(dev->net); //(do later)
	}
	
	//[Once set to 1, while FORCE_MODE_MAGIC, Clear to 0, while not above DFR case.]
	priv->summPwOnTgl= 0;

    /* lnk-up version info */
	if (bmsr_val & 0x04) {		
	  if (priv->coming_lnk_defer_on_ct<=(500-3)){
		   u8 nsr, temp;
		   dm_read_reg(dev, 0x01, &nsr);
		   dm_read_reg(dev, 0x82, &temp);
	       dm_write_reg(dev, 0x82, 0xff);
		   priv->summUsbErr += (int) temp;
		   //if (CONF_OK_PNT_PLUS1){
		   //  printk("~[dm96]- [LINK] [nsr 0x%02x lpa %04x] REG82H USBEr= %d, summ %d ", 
		   //  	nsr, priv->sav_lpa, temp, priv->summUsbErr);
		   //  printk("\n");
	       //}
	  }
	}
	
	priv->allow_defer_happen= 1;
	return 0;
}

static const struct driver_info dm9620_info = {
	.description	= "Davicom DM9620 USB Ethernet",
	.flags		= FLAG_ETHER,
	.bind		= dm9620_bind,
	.rx_fixup	= dm9620_rx_fixup,
	.tx_fixup	= dm9620_tx_fixup,
	.status		= dm9620_status_new, //dm9620_status,
	.link_reset	= dm9620_link_reset,
	.reset		= dm9620_link_reset,
	.unbind     = dm9620_unbind,
};

static const struct usb_device_id products[] = {
	{
	USB_DEVICE(0x0a46, 0x9620),     /* Davicom 9620 */
	.driver_info = (unsigned long)&dm9620_info,
	},
	{
	USB_DEVICE(0x0a46, 0x9621),     /* Davicom 9621 */
	.driver_info = (unsigned long)&dm9620_info,
	},
	{
	USB_DEVICE(0x0a46, 0x9622),     /* Davicom 9622 */
	.driver_info = (unsigned long)&dm9620_info,
	},
	{
	USB_DEVICE(0x0a46, 0x0269),     /* Davicom 9620A CDC */
	.driver_info = (unsigned long)&dm9620_info,
	},
	{
	USB_DEVICE(0x0a46, 0x1269),     /* Davicom 9621A CDC */
	.driver_info = (unsigned long)&dm9620_info,
	},
	{
	USB_DEVICE(0x0a46, 0x0268),     /* Davicom 9620A EXT MII */
	.driver_info = (unsigned long)&dm9620_info,
	},
	{
	USB_DEVICE(0x0a46, 0x1268),     /* Davicom 9621A EXT MII */
	.driver_info = (unsigned long)&dm9620_info,
	},
	{
	USB_DEVICE(0x0a46, 0x0267),     /* Davicom 9620A Fiber */
	.driver_info = (unsigned long)&dm9620_info,
	},
	{
	USB_DEVICE(0x0a46, 0x1267),     /* Davicom 9621A Fiber */
	.driver_info = (unsigned long)&dm9620_info,
	},
	{},			// END
};

MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver dm9620_driver = {
	.name = "dm9620",
	.id_table = products,
	.probe = usbnet_probe,
	.disconnect = usbnet_disconnect,
	.suspend = usbnet_suspend,
	.resume = usbnet_resume,
};

static int __init dm9620_init(void)
{
	return usb_register(&dm9620_driver);
}

static void __exit dm9620_exit(void)
{
	usb_deregister(&dm9620_driver);
}

module_init(dm9620_init);
module_exit(dm9620_exit);

MODULE_AUTHOR("Peter Korsgaard <jacmet@sunsite.dk>");
MODULE_DESCRIPTION("Davicom DM9620 USB 2.0 ethernet devices");
MODULE_LICENSE("GPL");
