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
 * V2.42 - Work to V2.42 according to "DM9620 BulkOut 補丁分解.doc"
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
 */

//#define DEBUG
#define LNX_DM9620_VER_STR  "V2593t(6pppJJ5-E3.3(5)-FIX126DIV2-CrrOpn5lnk)"

#define DC_INC_FOR_USBHOST_FAST	   1
#define DC_INC_FOR_USBHOST_SLOW	2200

//::A3X_PANEL_QFAN
//::BCM2836_RPI2
//::V210_FRIEND
typedef struct oem_setting {
  int index;
  struct {
    int inc;
    int ok_to_print;
    char *str;
  } cnf;
} oem_t; 

//::A3X_PANEL_QFAN
//::BCM2836_RPI2
//::V210_FRIEND
#define MACH_LAB_V1      {  11, 1, "LAB_V1"}
#define MACH_LAB_V2      {  92, 1, "LAB_V2"}
#define MACH_LAB_V3      { 126, 0, "LAB_V3"}            //0x66=102, 0x7e=126
#define MACH_REFERENCE   {  12, 0, "CPU_REFERENCE"}
#define MACH_A3X         {  20, 1, "A3X_PANEL"}         //DC_INC_A3X_PANEL_QFAN, DC_OEM_A3X_PANEL_QFAN
#define MACH_A3X_3BOARDS { 101, 1, "A3X_PANEL_3BOARDS"} //0x65 
#define MACH_BCM2836_S   {  20, 1, "BCM2836_RPI2_SLOW"} //DC_INC_BCM2836_RPI2,   DC_OEM_BCM2836_RPI2  (Rev5C= 0).ok / (Rev5C= 2).too.slow
#define MACH_BCM2836_F   { 100, 1, "BCM2836_RPI2_FAST"} //DC_INC_BCM2836_RPI2,   DC_OEM_BCM2836_RPI2
#define MACH_V210        {1100, 1, "V210_FRIEND"}       //DC_INC_V210_FRIEND,    DC_OEM_V210_FRIEND  (Rev5C= 2)

oem_t oem[]= {
  { 0, MACH_REFERENCE, },   //0. default
  { 1, MACH_A3X, },         //1. gemvary.A3X
  { 2, MACH_BCM2836_S, },   //2. rpi2.BCM2836.S
  { 3, MACH_BCM2836_F, },   //3. rpi2.BCM2836.F
  { 4, MACH_V210, },        //4. Friendly.V210
  { 5, MACH_LAB_V1, },      //5. Lab not bad
  { 6, MACH_A3X_3BOARDS, }, //6. gemvary.A3X.3BOARDS
  { 7, MACH_LAB_V2, },      //5. Lab2 not bad
  { 8, MACH_LAB_V3, },      //8. Lab3
};
#define CONF_N  8 // 8,7,5,

//::A3X_PANEL_QFAN
//::BCM2836_RPI2
//::V210_FRIEND
#define CONF_INC  oem[CONF_N].cnf.inc
#define CONF_OEM  oem[CONF_N].cnf.str

/*  We make it both 'CONF_OK_PNT' && ('priv->IP_conf_inc&1') */
/*  to be able to print log */
/*  So can set 'priv->IP_conf_inc' even to coerce to stop print log */ 
#define CONF_OK_PNT 		  (oem[CONF_N].cnf.ok_to_print)
#define CONF_OK_PNT_PLUS1 	  (oem[CONF_N].cnf.ok_to_print && ((priv->IP_conf_inc&1)))
#define CONF_OK_PNT_DURING_OP (oem[CONF_N].cnf.ok_to_print && ((priv->IP_conf_inc&1)))  // ( || priv->opnum_op)

/* Choice infinite or not ! */
#define TOTAL_CNT_TOGG             36
#define WORK_INFINIT_TOGG			1
#define DEFER_ON_CONTROL		  150
#define WORK_INFINIT_REG82H			1

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

#define FORCE_MODE_MAGIC	0xbd

#define NUM_BLOCK			  2
#define OFFSET_FOR_BLOCK	 10
#define OFFSET_FOR_UNBLOCK	100

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
#define TGL_EXTRA_WRITE 6 // Relaese: want > 6, 18, [25], 250, 500
	int  summCarrierTgl;
	
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
  
	u8  mdat[512];
};     
#if defined(DEBUG)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,33)
#define dm9620_print(__dev, format, args...) netdev_dbg((__dev)->net, format, ##args) 
#define dm9620_err(__dev, format, args...) netdev_err((__dev)->net, format, ##args)
#else if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,33)
#define dm9620_print(dev, format, args...) devdbg(dev, format, ##args)
#define dm9620_err(dev, format, args...) deverr(dev, format, ##args)
#endif
#else
#define dm9620_print(dev, format, args...) printk(format, ##args)
#define dm9620_err(dev, format, args...) printk(format, ##args)
#endif

//void udelay7(int loops)
//udelayEx(int loops)
//{
//  udelay7(loops * 7)
//}
#define udelayEx(n)		\
	udelay(n);			\
	udelay(n);			\
	udelay(n);			\
	udelay(n);			\
	udelay(n);			\
	udelay(n);			\
	udelay(n)

static int dm_read_EP3_desc(struct usbnet *dev, void *data, u16 dt, u16 size)
{
	return usb_control_msg(dev->udev,
				   usb_rcvctrlpipe(dev->udev, 0),
			       USB_REQ_GET_DESCRIPTOR,
			       USB_DIR_IN,
			       dt << 8, 0, data, size, USB_CTRL_SET_TIMEOUT); // USB_DT_ENDPOINT=05 EP_DescType, 3 index_EP3, 7 length
}

static int dm_read(struct usbnet *dev, u8 reg, u16 length, void *data)
{
//  	dm9620_print(dev, "dm_read() reg=0x%02x length=%d", reg, length);
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
	if (!tmpwPtr)
	{
		printk("+++++++++++ JJ5 dm_read_reg() Error: can not kmalloc!\n"); //usbnet_suspend (intf, message);
		return 0; 
	}
	
	ret = dm_read(dev, reg, 2, tmpwPtr);  // usb_submit_urb v.s. usb_control_msg
	*value= (u8)(*tmpwPtr & 0xff);
	
	kfree (tmpwPtr);
	return ret;
}

static int dm_write(struct usbnet *dev, u8 reg, u16 length, void *data)
{
//  dm9620_print(dev, "dm_write() reg=0x%02x, length=%d", reg, length);
	return usb_control_msg(dev->udev,
			       usb_sndctrlpipe(dev->udev, 0),
			       DM_WRITE_REGS,
			       USB_DIR_OUT | USB_TYPE_VENDOR |USB_RECIP_DEVICE,
			       0, reg, data, length, USB_CTRL_SET_TIMEOUT);
}

static int dm_write_reg(struct usbnet *dev, u8 reg, u8 value)
{
//	dm9620_print(dev , "dm_write_reg() reg=0x%02x, value=0x%02x", reg, value);
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
//  dm9620_print(dev, "dm_write_async() reg=0x%02x length=%d", reg, length);
	dm_write_async_helper(dev, reg, 0, length, data);
}

static void dm_write_reg_async(struct usbnet *dev, u8 reg, u8 value)
{
//	dm9620_print(dev, "dm_write_reg_async() reg=0x%02x value=0x%02x",
//	       reg, value);      

	dm_write_async_helper(dev, reg, value, 0, NULL);
}

static int dm_read_shared_word(struct usbnet *dev, int phy, u8 reg, __le16 *value)
{
	int ret, i;
  u16 *tmpwPtr1;

	mutex_lock(&dev->phy_mutex);

	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	dm_write_reg(dev, DM_SHARED_CTRL, phy ? 0xc : 0x4);

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp;

		udelayEx(1);
		ret = dm_read_reg(dev, DM_SHARED_CTRL, &tmp);
		if (ret < 0) {
			if (phy)
			  printk("++++++[rdPHY %d]+++++  ret<0 out return\n", reg);
			else
			  printk("++++++[rdWORD %d]+++++  ret<0 out return\n", reg);
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

//	dm9620_print(dev, "read shared %d 0x%02x returned 0x%04x, %d",
//	       phy, reg, *value, ret);      

 out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static int dm_write_shared_word(struct usbnet *dev, int phy, u8 reg, __le16 value)
{
	int ret, i;

	mutex_lock(&dev->phy_mutex);

	ret = dm_write(dev, DM_SHARED_DATA, 2, &value);
	if (ret < 0)
		goto out;

	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
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
    //offset= offset / 2;  /*dm9620_write_eeprom(dev, offset / 2, data);*/
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
	
//	dm9620_print(dev, "EEPROM: magic value, magic = 0x%x offset =0x%x data = 0x%x ",eeprom->magic, eeprom->offset,*data);
	if (eeprom->magic != MD96XX_EEPROM_MAGIC) {
		dm9620_print(dev, "EEPROM: magic value mismatch, magic = 0x%x",
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

	dm_read_shared_word(dev, phy_id, loc, &res);

//  dm9620_print(dev, "dm9620_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x",
//	       phy_id, loc, le16_to_cpu(res));
	return le16_to_cpu(res);
}

static void dm9620_mdio_write(struct net_device *netdev, int phy_id, int loc,
			      int val)
{
	struct usbnet *dev = netdev_priv(netdev);
	__le16 res = cpu_to_le16(val);
	int mdio_val;

//	dm9620_print(dev, "dm9620_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x",
//	       phy_id, loc, val);      

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


#define DM_LINKEN  (1<<5)
#define DM_MAGICEN (1<<3)
#define DM_LINKST  (1<<2)
#define DM_MAGICST (1<<0)

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

static struct ethtool_ops dm9620_ethtool_ops = {
	.get_drvinfo	= dm9620_get_drvinfo,
	.get_link	= dm9620_get_link,
	.get_msglevel	= usbnet_get_msglevel,
	.set_msglevel	= usbnet_set_msglevel,
	.get_eeprom_len	= dm9620_get_eeprom_len,
	.get_eeprom	= dm9620_get_eeprom,
	.set_eeprom	= dm9620_set_eeprom,
	.get_settings	= usbnet_get_settings,
	.set_settings	= usbnet_set_settings,
	.nway_reset	= usbnet_nway_reset,
	.get_wol	= dm9620_get_wol,
	.set_wol	= dm9620_set_wol,
};

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

static void dm9620_start_carriering(struct dm96xx_priv *priv)
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
}

 static int dm9620_open(struct net_device *net)
 {
    struct usbnet *dev = netdev_priv(net);
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	
    printk("\n");
	printk("+[dm96 open.s]- (TRIP %d on-ticks %04d)\n", priv->IP_toggle_count, priv->IP_dc); 
	
	/* work-around for 9601 mode, 9620 mode */
	dm9620_start_carriering(priv); //priv->summCarrierTgl= 0;
	
  //priv->IP_done= 0;
 	priv->IP_TOGG= 0;
 	priv->IP_toggle_count= 0;
 	priv->allow_defer_happen= 1;
 	
 	/* Need EP3I */
 	printk("+[dm96 open]-   (play get link_up & carrier-on)\n"); 
 	netif_carrier_on(net);
 	//printk("-[dm96 open]- (play get link_down & carrier-off)\n"); 
 	//netif_carrier_off(net);
 	
	printk("+[dm96 open.e]- (TRIP %d on-ticks %04d)\n", priv->IP_toggle_count, priv->IP_dc); 
    printk("\n");
 	return usbnet_open(net);
 } 
 static int dm9620_stop (struct net_device *net)
 {
 	int ret= usbnet_stop(net);
 	netif_carrier_off(net);
 	printk("+[dm96 close.s]- (now)\n");
 	printk("+[dm96 close.e]- (carrier off)\n"); 
 	return ret;
 } // chiphd-work: 

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 

static const struct net_device_ops vm_netdev_ops= { // new kernel 2.6.31  (20091217JJ)

            .ndo_open               = dm9620_open, // usbnet_open,  
            .ndo_stop               = dm9620_stop, //usbnet_stop,  
            .ndo_start_xmit         = usbnet_start_xmit, 
            .ndo_tx_timeout         = usbnet_tx_timeout, 
            .ndo_change_mtu         = usbnet_change_mtu, 
            .ndo_validate_addr      = eth_validate_addr, 
	    .ndo_do_ioctl	    = dm9620_ioctl,   
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
            .ndo_set_rx_mode        = dm9620_set_multicast,   
#else
	    .ndo_set_multicast_list = dm9620_set_multicast,   
#endif
            .ndo_set_mac_address    = dm9620_set_mac_address,  
};
#endif

static void Save_cnf_inc(struct usbnet *dev)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	priv->IP_save_inc= priv->IP_conf_inc;
}

static void Write_cnf_inc(struct usbnet *dev, u8 conf_stat)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	
		    if ( (priv->mdat[0]==conf_stat) && (priv->mdat[1]==priv->IP_conf_inc))
			  printk("++++++[INC already %d equal to EEPROM's %d]+++++\n", priv->IP_conf_inc, priv->mdat[1]);
		    else {
		    	
		      //if (priv->mdat[1]==0xff){
			  //  printk("++++++[INC is change from 0xff to %d]+++++\n", priv->IP_conf_inc);
		      //  priv->mdat[1]= CONF_INC;
		      //} else
			    printk("++++++[INC is written (INC cnf-stat 0x%02x)]+++++\n", conf_stat);
			    printk("++++++[INC is change from %d to %d (0x%02x to 0x%02x)]+++++\n", 
			       priv->mdat[1], priv->IP_conf_inc,
			       priv->mdat[1], priv->IP_conf_inc);
			  
			    priv->mdat[0]= conf_stat;
			  //priv->mdat[0]= 0xff;  
			  priv->mdat[1]= priv->IP_conf_inc;
			  
			  dm_write_eeprom_word(dev, 30 / 2, priv->mdat);
			  udelayEx(10);
			  printk("++++++[wrWORD 15]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
			  udelayEx(100);
		      dm_read_eeprom_word(dev, 30 / 2, priv->mdat);
			  printk("++++++[rdWORD 15]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
			  udelayEx(100);
		      dm_read_eeprom_word(dev, 30 / 2, priv->mdat);
			  printk("++++++[rdWORD 15]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
			  udelayEx(100);
		      dm_read_eeprom_word(dev, 30 / 2, priv->mdat);
			  printk("++++++[rdWORD 15]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
			}
}

/*static void InitWrite_cnf_inc(struct usbnet *dev)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	   priv->IP_save_inc= priv->IP_conf_inc;
	   if (dm_read_eeprom_word(dev, 30 / 2, priv->mdat) < 0)
		    printk("++++++[Can not write new INC %d to EEPROM word 15]+++++\n", priv->IP_conf_inc);
	   else {
	   	    // write to EEPROM word 15, conf stat: 0x00 
	   	    Write_cnf_inc(dev, 0x00); // 0x00 is for init-'6ppp_USBEr(FIXED)' 
	   }
}
*/
	
static void CheckWrite_cnf_inc(struct usbnet *dev)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	
	   if (priv->IP_conf_inc==priv->IP_save_inc)
	         return;
	
	   /* SAVE FIRST */
	   priv->IP_save_inc= priv->IP_conf_inc;
	   
	 #if 1
	   /* write is disable */
	   printk("++++++[Warn: Not allow to write new INC %d to EEPROM word 15]+++++\n", priv->IP_conf_inc);
	 #else  
	   if (dm_read_eeprom_word(dev, 30 / 2, priv->mdat) < 0)
		    printk("++++++[Can not write new INC %d to EEPROM word 15]+++++\n", priv->IP_conf_inc);
	   else {
	   	    /* write to EEPROM word 15, conf stat: 0x01 */
	   	    Write_cnf_inc(dev, 0x01); /*priv->mdat[0], is state change */ 
	   }
	 #endif
}

static void New_cnf_inc(struct usbnet *dev)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	
	if (priv->IP_conf_rxmatch==(priv->IP_conf_rxdef+1) &&
	  priv->IP_conf_rxmatch==(priv->IP_conf_rxplus-1))
	{
	   priv->IP_conf_inc= priv->IP_conf_rxmatch;
	   printk("~~~~~ ~~~~~ ~~~~~ [dm96]- [set INC] %d ~~~~~ ~~~~~ ~~~~~\n", priv->IP_conf_inc);

	   /* clean varibes */
	   priv->IP_conf_rxdef= 0;
	   priv->IP_conf_rxmatch= 0;  // clear (reset-it)
	   priv->IP_conf_rxplus= 0;
	}
}

int dm9620_get_endpoints(struct usbnet *dev, struct usb_interface *intf)
{
	int				tmp;
	struct usb_host_interface	*alt = NULL;
	struct usb_host_endpoint	*in = NULL, *out = NULL;
	struct usb_host_endpoint	*status = NULL;

 printk("\n#\n");
 printk("(usbnet)DM961: Interface: num_altsetting= %d\n", intf->num_altsetting);
	for (tmp = 0; tmp < intf->num_altsetting; tmp++) {
		unsigned	ep;

		in = out = status = NULL;
		alt = intf->altsetting + tmp;

 printk("(usbnet)DM961: Interface: altsetting: %d in addr: (struct usb_host_interface *) 0x%x\n", tmp, (unsigned int) alt);
 
		/* take the first altsetting with in-bulk + out-bulk;
		 * remember any status endpoint, just in case;
		 * ignore other endpoints and altsettings.
		 */
		 
		 /* struct usb_interface_descriptor "alt->desc"; */
		 
 printk("(usbnet)DM961: Interface,altsetting: bNumEndpoints= %d\n", alt->desc.bNumEndpoints);
		for (ep = 0; ep < alt->desc.bNumEndpoints; ep++) {
			struct usb_host_endpoint	*e;
			int				intr = 0;

			e = alt->endpoint + ep;
			
		 /* struct usb_host_endpoint *e;              */
		 /* struct usb_endpoint_descriptor "e->desc"; */
		 
 printk("(usbnet)DM961: Interface,altsetting,endpoint: Endpoints-number-%d\n", e->desc.bEndpointAddress & 0x0f); // D[3:0]= Endpoint Number
 
 printk("(usbnet)EP%d: %02x %02x %02x %02x %02x %02x %02x\n", 
   e->desc.bEndpointAddress & 0x0f,
 	e->desc.bLength, 
 	e->desc.bDescriptorType,
 	e->desc.bEndpointAddress,
 	e->desc.bmAttributes,
 	e->desc.wMaxPacketSize & 0xff,
 	e->desc.wMaxPacketSize >> 8,
 	e->desc.bInterval);
 
			switch (e->desc.bmAttributes) {
			case USB_ENDPOINT_XFER_INT:
			
			  printk("(usbnet)DM961: usb_endpoint_dir_in( ) is %d\n", (e->desc.bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN );
			  printk("Because e->desc.bEndpointAddress= %02x, USB_ENDPOINT_DIR_MASK= %02x, USB_DIR_IN= %02x\n",
			    e->desc.bEndpointAddress, USB_ENDPOINT_DIR_MASK, USB_DIR_IN);
			
				if (!usb_endpoint_dir_in(&e->desc))
					continue;					
				intr = 1;
				
				printk("(usbnet)DM961: intr= %d\n", intr);
			    
				/* FALLTHROUGH */
			case USB_ENDPOINT_XFER_BULK:
				break;
			default:
				continue;
			}
			if (usb_endpoint_dir_in(&e->desc)) {
				if (!intr && !in)
					in = e;
				else if (intr && !status)
				{
					status = e;
					printk("(usbnet)DM961: (struct usb_host_endpoint *) 0x%x\n", (unsigned int) status);
					printk("(usbnet)DM961: THis is ----- (usb_host_endpoint).interrupt ------\n");
				}
			} else {
				if (!out)
					out = e;
			}
		}
		if (in && out)
			break;
	}
	
	if (status){
	  dev->status = status;
	  printk("(usbnet)DM961: dev->status = status (= 0x%x)\n", (unsigned int) status);
	}
	
	if (!alt || !in || !out) {
 printk("(usbnet)DM961: Return (!alt || !in || !out)\n");
		return -EINVAL;
	}

	if (alt->desc.bAlternateSetting != 0 ||
	    !(dev->driver_info->flags & FLAG_NO_SETINT)) {
	    	
 /* USB_DT_INTERFACE: Interface descriptor */
 printk("(usbnet)DM961: usb_set_interface (alt->desc.bInterfaceNumber: %d, alt->desc.bAlternateSetting: %d)\n", 
 	alt->desc.bInterfaceNumber, alt->desc.bAlternateSetting); // __u8  bAlternateSetting;
 	
		tmp = usb_set_interface (dev->udev, alt->desc.bInterfaceNumber,
				alt->desc.bAlternateSetting);
		if (tmp < 0)
			return tmp;
	}

 printk("(usbnet)DM961: usb_rcvbulkpipe (In bEndpointAddress: %d)\n", in->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->in = usb_rcvbulkpipe (dev->udev,
			in->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
 printk("(usbnet)DM961: usb_sndbulkpipe (Out bEndpointAddress: %d)\n", out->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->out = usb_sndbulkpipe (dev->udev,
			out->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	//dev->status = status;
	return 0;
}

static int dm9620_bind(struct usbnet *dev, struct usb_interface *intf)
{
  //u16 *tmpwPtr2;
	int ret,mdio_val,i;
	struct dm96xx_priv* priv;
	u8 temp;
	u8 tmp;
	u16 integrate_size;
	int n, m;
/*
    printk("\n");
	printk("[dm962] Linux Driver.pre = %s\n", LNX_DM9620_VER_STR);

	 dm_read_reg(dev, DM_SMIREG, &temp);
	 dm_write_reg(dev, DM_SMIREG, temp &0xef);
      printk(KERN_WARNING "[dm96] 9620 REG91H (DM_SMIREG= Get 0x%02x, Set to 0x%02x)\n", temp, temp &0xef);
	 dm_read_reg(dev, DM_SMIREG, &temp);
      printk(KERN_WARNING "[dm96] 9620 REG91H (DM_SMIREG= Get 0x%02x)\n", temp);
*/
	//ret = usbnet_get_endpoints(dev, intf);
	ret = dm9620_get_endpoints(dev, intf);
	
  // if (dev->status)
  //
  //	
	
	if (ret)
		goto out;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
	dev->net->netdev_ops = &vm_netdev_ops; // new kernel 2.6.31  (20091217JJ)
	dev->net->ethtool_ops = &dm9620_ethtool_ops;
#else
	dev->net->do_ioctl = dm9620_ioctl;  
	dev->net->set_multicast_list = dm9620_set_multicast;
	dev->net->ethtool_ops = &dm9620_ethtool_ops;
#endif
	dev->net->hard_header_len += DM_TX_OVERHEAD;
	dev->hard_mtu = dev->net->mtu + dev->net->hard_header_len;
	dev->rx_urb_size = dev->net->mtu + ETH_HLEN + DM_RX_OVERHEAD+1; // ftp fail fixed

	dev->mii.dev = dev->net;
	dev->mii.mdio_read = dm9620_mdio_read;
	dev->mii.mdio_write = dm9620_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;
	dev->mii.phy_id = DM9620_PHY_ID;

    /* bind version info.s */
    printk("\n");
	printk("[dm962] Linux Driver = %s\n", LNX_DM9620_VER_STR);
	//printk("[dm962] Linux Driver OEM %s\n", CONF_OEM);
    //printk("[dm962] Linux Driver INC %d\n", CONF_INC);
	printk("\n");
	
//JJ1
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
		
//JJ3
	if ( (ret= dm_read_reg(dev, 0xF2, &tmp)) >=0)
		printk("++++++[dm962]+++++ dm_read_reg() 0xF2 0x%02x\n",tmp);
	else
		printk("++++++[dm962]+++++ dm_read_reg() 0xF2 fail-func-return %d\n", ret);
		
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[7] %d %s\n", tmp>>7, (tmp&(1<<7))? "Err: RX Unexpected condition": "OK" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[6] %d %s\n", (tmp>>6)&1, (tmp&(1<<6))? "Err: Host Suspend condition": "OK" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[5] %d %s\n", (tmp>>5)&1, (tmp&(1<<5))? "EP1: Data Ready": "EP1: Empty" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[3] %d %s\n", (tmp>>3)&1, (tmp&(1<<3))? "Err: Bulk out condition": "OK" );
		
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[2] %d %s\n", (tmp>>2)&1, (tmp&(1<<2))? "Err: TX Buffer full": "OK" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[1] %d %s\n", (tmp>>1)&1, (tmp&(1<<1))? "Warn: TX buffer Almost full": "OK" );
		printk("++++++[dm962]+++++  [Analysis.2] 0xF2, D[0] %d %s\n", (tmp>>0)&1, (tmp&(1<<0))? "Status: TX buffer has pkts": "Status: TX buffer 0 pkts" );

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
	if (dm_read(dev, DM_PHY_ADDR, ETH_ALEN, dev->net->dev_addr) < 0) {
		printk(KERN_ERR "Error reading MAC address\n");
		ret = -ENODEV;
		goto out;
	}

#if 1
	 printk("[dm96] Chk mac addr %pM\n", dev->net->dev_addr);  // %x:%x...
	 printk("[dm96] ");
	 for (i=0; i<ETH_ALEN; i++)
	 printk("[%02x] ", dev->net->dev_addr[i]);
	 printk("\n");
#endif

	/* read SMI mode register */
	 priv = dev->driver_priv = kmalloc(sizeof(struct dm96xx_priv), GFP_ATOMIC);
	 if (!priv) {
		dm9620_err(dev,"Failed to allocate memory for dm96xx_priv");
		ret = -ENOMEM;
		goto out;
	 }

#if 1	
	 n= dm_read_EP3_desc(dev, priv->mdat, 1, 18); // DescriptorType: 1 (Device)
	 printk("\n");
	 printk("++++++[Device desc]+++++  Get len= %d\n", n);
	 printk(" %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n", 
		 priv->mdat[0], priv->mdat[1], priv->mdat[2], priv->mdat[3], priv->mdat[4], priv->mdat[5], priv->mdat[6], priv->mdat[7],
		 priv->mdat[8], priv->mdat[9], priv->mdat[10], priv->mdat[11], priv->mdat[12], priv->mdat[13], priv->mdat[14], priv->mdat[15]);
	 printk(" %02x %02x\n",
		 priv->mdat[16], priv->mdat[17]
		 );
	 n= dm_read_EP3_desc(dev, priv->mdat, 2, 9); // DescriptorType: 2 (Configuration)
	 printk("++++++[Config0 desc]+++++  Get len= %d\n", n);
	 for (i=0; i<9; i++)
	 {
		 if (i && (!(i%16)) ) printk("\n");
		 else if (i && (!(i%8)) ) printk(" ");
		 printk(" %02x", priv->mdat[i]);
	 }
	 printk("\n");
#endif
				
	 printk("\n");
	 //E.g. 27 00 (Or 50 00)
	 integrate_size= priv->mdat[2]; 
	 //integrate_size += priv->mdat[3] << 8;
	 printk("++++++[Config0 Desc: %02x %02x]+++++  Total Descriptor Length len= %d\n",
		 priv->mdat[2], priv->mdat[3], integrate_size);
	 printk("\n");
		
#if 1
	 n= dm_read_EP3_desc(dev, priv->mdat, 2, integrate_size); // Total Descriptor Length
	 printk("++++++[Config0 desc]+++++ Get len= ----- %d -----\n", n);
	 m= 0;
	 for (i=0; i<n; i++)
	 {
		 //if (i && (!(i%16)) ) printk("\n");
		 //else if (i && (!(i%8)) ) printk(" ");
		 if (!m) m= priv->mdat[i];
		  
		 printk(" %02x", priv->mdat[i]);
		 m--;
		 if (!m) printk("\n");
	 }
	 printk("\n");
		
	 /* current ep3i, E.g. 27 00 */
	 priv->desc_ep3i= 0;
	 priv->desc_ep3i_pntonce= 1; //0;
	 priv->defer_count= 0;
	 if (n==39)
		priv->desc_ep3i= priv->mdat[39-1];
#endif
		printk("\n");

#if 1
    #if 1
    for (i=0; i<64; i++)
      dm_read_eeprom_word(dev, i, &priv->mdat[i<<1]);
    
    for (i=0; i<64; i++){
      if (!(i%8)) printk("\n");
	  printk("%02x %02x ", priv->mdat[i<<1], priv->mdat[(i<<1)+1]);
    }
    printk("\n");
    #endif

    /* v2.59.3 (render eeprom if need) WORD3 render-II */
    /* v2.59.3 (render eeprom if need) WORD7 render-I */
    /* v2.59.3 (render eeprom if need) WORD11 render-II */
    /* v2.59.3 (render eeprom if need) WORD12 render-II */

#if 1
#if 0
  //dm_eeprom_render(dev, 3, 0x0010, 0xc030);[Test: 1551h]
#endif

  //dm_eeprom_render(dev, 3, 0x4010, 0xc030);[Test: 5551h]
    dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
    if ((priv->mdat[1] != 0x55) || (priv->mdat[0] != 0x51)) //[Test: 5551h]
    {
	  printk("++++++[rdWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  priv->mdat[1] = 0x55;
	  priv->mdat[0] = 0x51;
	  dm_write_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(10);
	  printk("++++++[wrWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  printk("++++++[rdWORD 3].ck1-+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  printk("++++++[rdWORD 3].ck2-+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  printk("++++++[rdWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
    }

#if 0
  //dm_eeprom_render(dev, 3, 0x4010, 0xc030);[Test: 5551h]
    dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
    if ((priv->mdat[1]&0xc0) != 0x40) //[Test: 5551h]
    {
	  printk("++++++[rdWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  priv->mdat[1] &= 0x3f;
	  priv->mdat[1] |= 0x40;
	  dm_write_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(10);
	  printk("++++++[wrWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  printk("++++++[rdWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
    }
#endif
#if 0
  //dm_eeprom_render(dev, 3, 0x0010, 0x0030);[Keep: 1551h or 5551h]
    dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
    if ((priv->mdat[0]&0x30) != 0x10)
    {
	  printk("++++++[rdWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  priv->mdat[0] &= 0xcf;
	  priv->mdat[0] |= 0x10;
	  dm_write_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(10);
	  printk("++++++[wrWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  printk("++++++[rdWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
    }
#endif
#endif

    dm_read_eeprom_word(dev, 14 / 2, priv->mdat);  // set to Normal mode.
    if (priv->mdat[1]&0x04)
    {
	  printk("++++++[rdWORD 7]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  priv->mdat[1] &= 0xfb;
	  dm_write_eeprom_word(dev, 14 / 2, priv->mdat);  
	  udelayEx(10);
	  printk("++++++[wrWORD 7]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 14 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 14 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 14 / 2, priv->mdat);
	  printk("++++++[rdWORD 7]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
    }

    /*
    dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
    if ((priv->mdat[1]&0xc0) != 0x40)
    {
	  printk("++++++[rdWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  priv->mdat[1] &= 0x3f;
	  priv->mdat[1] |= 0x40;
	  dm_write_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(10);
	  printk("++++++[wrWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 6 / 2, priv->mdat);
	  printk("++++++[rdWORD 3]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
    }
    
    dm_read_eeprom_word(dev, 14 / 2, priv->mdat);
    if (priv->mdat[1]&0x04)
    {
	  printk("++++++[rdWORD 7]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  priv->mdat[1] &= 0xfb;
	  dm_write_eeprom_word(dev, 14 / 2, priv->mdat);  
	  udelayEx(10);
	  printk("++++++[wrWORD 7]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 14 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 14 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 14 / 2, priv->mdat);
	  printk("++++++[rdWORD 7]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
    }
    dm_read_eeprom_word(dev, 22 / 2, priv->mdat);
    if (priv->mdat[0]!=0x5a || priv->mdat[1]!=0x00)
    {
	  printk("++++++[rdWORD 11]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  priv->mdat[0]= 0x5a; priv->mdat[1]= 0x00;
	  dm_write_eeprom_word(dev, 22 / 2, priv->mdat);
	  udelayEx(10);
	  printk("++++++[wrWORD 11]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 22 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 22 / 2, priv->mdat);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 22 / 2, priv->mdat);
	  printk("++++++[rdWORD 11]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
    }*/
    
    #define WD12_EP3I_6ppp  	0x07
    #define WD12_EP3I_RELBIN  	0x10
    
    #define WD12_EP3I  WD12_EP3I_6ppp
    //#define WD12_EP3I  WD12_EP3I_RELBIN
    //#define WD12_EP3I  0x07
    //#define WD12_EP3I  0x07 //0x08 // 0x09 faster, 0x0B
    //#define WD12_EP3I  0x0C  [Verr long Time]
    
   #if 1
    dm_read_eeprom_word(dev, 24 / 2, priv->mdat);
    if (priv->mdat[0]!=WD12_EP3I)
    {
	  printk("++++++[rdWORD 12]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  priv->mdat[0]= WD12_EP3I;  priv->mdat[1]= 0;
	  dm_write_eeprom_word(dev, 24 / 2, priv->mdat);
	  udelayEx(10);
	  printk("++++++[wrWORD 12]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 24 / 2, priv->mdat);
	  printk("++++++[rdWORD 12]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 24 / 2, priv->mdat);
	  printk("++++++[rdWORD 12]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
	  udelayEx(100);
      dm_read_eeprom_word(dev, 24 / 2, priv->mdat);
	  printk("++++++[rdWORD 12]+++++  %02x %02x\n", priv->mdat[0], priv->mdat[1]);
    }
   #endif
	
    dm_read_eeprom_word(dev, 30 / 2, priv->mdat);
  /*if (priv->mdat[1]==0xff)*/
    if (priv->mdat[1]!=CONF_INC)
    {
	  priv->IP_conf_rxdef= CONF_INC-1; //priv->IP_conf_rxdef= CONF_INC;
	  priv->IP_conf_rxmatch= CONF_INC;  // INIT CONSTANT INIT
	  priv->IP_conf_rxplus= CONF_INC+1;  // INIT CONSTANT INIT
	  New_cnf_inc(dev);
	  //InitWrite_cnf_inc(dev);
	  //=
	  Save_cnf_inc(dev);
	  Write_cnf_inc(dev, 0x00); // 0x00 is for init-'6ppp_USBEr(FIXED)' 
    }
    else 
    {
	  priv->IP_conf_rxdef= priv->mdat[1]-1; //priv->IP_conf_rxdef= priv->mdat[1];
	  priv->IP_conf_rxmatch= priv->mdat[1];
	  priv->IP_conf_rxplus= priv->mdat[1]+1;
	  New_cnf_inc(dev);
	  Save_cnf_inc(dev);
    }
    
    priv->opnum_op= 0;
    priv->opkey_input[0]= priv->opkey_input[1]= priv->opkey_input[2]= 0;
    priv->save_input= 0;
    priv->block_input[0]= priv->block_input[1]= 0;
    priv->IP_conf_track_bas[0]= priv->IP_conf_track_bas[1]= 0;
    
    #if 1
    for (i=0; i<16; i++){ //13
      if (!(i%8)) printk("\n");
      dm_read_eeprom_word(dev, i, priv->mdat);
	  printk("%02x %02x ", priv->mdat[0], priv->mdat[1]);
    }
    printk("\n");
    #endif
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
	if (ret<0) {
		printk(KERN_ERR "[dm96] Error read SMI register\n");
	}
	else priv->mode_9620 = temp & DM_MODE9620;

	printk(KERN_WARNING "[dm96] 9620 Mode = %d\n", priv->mode_9620);
	
	dm_read_reg(dev, DM_TXRX_M, &temp);  // Need to check the Chipset version (register 0x5c is 0x02?)
	if (temp == 0x02)
	{
	 dm_read_reg(dev, 0x3f, &temp);
	 temp |= 0x80; 
	 dm_write_reg(dev, 0x3f, temp);
   }
  
  //Stone add for check Product ID == 0x1269
  //tmpwPtr2= kmalloc (2, GFP_ATOMIC);
	//if (!tmpwPtr2)
	//{
		//printk("+++++++++++ JJ5 dm_read_reg() Error: can not kmalloc!\n"); //usbnet_suspend (intf, message);
		//return 0; 
	//} 
  //ret =dm_read(dev, DM_PID, 2, tmpwPtr2);

  //if (*tmpwPtr2 == 0x1269)
   //dm_write_reg(dev, DM_SMIREG, 0xa0);
   
  //if (*tmpwPtr2 == 0x0269)
   //dm_write_reg(dev, DM_SMIREG, 0xa0); 
  
  //kfree (tmpwPtr2); 
	 dm_read_reg(dev, DM_SMIREG, &temp);
	  printk(KERN_WARNING "[dm96] 9620 REG91H (DM_SMIREG= Get 0x%02x, Xet xo 0xHH)\n", temp);
	 //dm_write_reg(dev, DM_SMIREG, temp &0xef);
      //printk(KERN_WARNING "[dm96] 9620 REG91H (DM_SMIREG= Get 0x%02x, Set to 0x%02x)\n", temp, temp &0xef);
	 dm_read_reg(dev, DM_SMIREG, &temp);
      printk(KERN_WARNING "[dm96] 9620 REG91H (DM_SMIREG= Get 0x%02x)\n", temp);

    /* bind version info.e */
     printk("\n");
	 printk("([dm962] Linux Driver = %s)\n", LNX_DM9620_VER_STR);
	 printk("([dm962] Linux Driver OEM %s)\n", CONF_OEM);
     printk("([dm962] Linux Driver INC %d EP3-INTVL 0x%02X)\n", priv->IP_conf_inc, WD12_EP3I); // 'CONF_INC'
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
	printk("dm9620_bind().e: return ret= %d\n", ret);
	return ret;
}

void dm9620_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	struct dm96xx_priv* priv= dev->driver_priv;
	printk("dm9620_unbind():\n");

    /* unbind version info.e */
	printk("([dm962] Linux Driver = %s)\n", LNX_DM9620_VER_STR);
	printk("([dm962] Linux Driver OEM %s)\n", CONF_OEM);
    printk("([dm962] Linux Driver INC %d EP3-INTVL 0x%02X)\n", priv->IP_conf_inc, WD12_EP3I); //'CONF_INC'
    
    printk("EXIT defr [because %d/(%d)] /%d\n", priv->defer_on_looping, 1, DEFER_ON_CONTROL);
    printk("EXIT TGGL [because %d/(%d)] /%d\n", priv->IP_dc, priv->IP_conf_inc, 2250);
    
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

struct uip_eth_addr {
  u8 addr[6];
};

struct uip_eth_hdr {
  struct uip_eth_addr dest;
  struct uip_eth_addr src;
  u16 type;
};

struct arp_msg { //= arp_hdr
  struct uip_eth_hdr ethhdr;
  u16 hwtype;
  u16 protocol;
  u8 hwlen;
  u8 protolen;
  u16 opcode;
  struct uip_eth_addr shwaddr;
  u8 sipaddr[4];
  struct uip_eth_addr dhwaddr;
  u8 dipaddr[4];
};

#define BUF ((struct arp_msg *)&skb->data[4+0])
#define TXBUF ((struct arp_msg *)&skb->data[2+0])

/* BLOCK INPUT METHOD */
static int dm9620_block_input(struct usbnet *dev, int ival)
{
  //return 1; //true;
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	int i;
  
    if (ival==priv->block_input[0] || 
      ival==priv->block_input[1])
    {
      if (ival==priv->save_input) // repeat entering..
        return 1;
        
      if (CONF_OK_PNT_DURING_OP)
  	    printk("~~ ~~~ ~~~~~ [dm96]- [BLOCK can-not BAS] %d ~~~~~ ~~~ ~~\n", ival); 
      return 1;
    }
    
    if (ival== (priv->IP_conf_rxdef+OFFSET_FOR_BLOCK) ||
        ival== (priv->IP_conf_rxdef-OFFSET_FOR_BLOCK) )
    {
      for (i=0; i<NUM_BLOCK; i++)
      {
       	 if (priv->IP_conf_rxdef==priv->block_input[i]) 
       	   return 1;
      }
     	
      if (priv->block_input[0] && priv->block_input[1] && CONF_OK_PNT_DURING_OP)
  	  {
  	   printk("~~ ~~~ ~~~~~ [dm96]- [BLOCKED Full] [can-not BLOCK] %d ~~~~~ ~~~ ~~\n", priv->IP_conf_rxdef); 
  	   printk("~~ ~~~ ~~~~~ [dm96]- [BLOCKED List.0] %d \n", priv->block_input[0]); 
  	   printk("~~ ~~~ ~~~~~ [dm96]- [BLOCKED List.1] %d \n", priv->block_input[1]); 
  	  }
  	  else
      {
      	
       for (i=0; i<NUM_BLOCK; i++)
       {
	       if (!priv->block_input[i]) 
	       {
	         priv->block_input[i]= priv->IP_conf_rxdef;
	         if (CONF_OK_PNT_DURING_OP)
	           printk("~~ ~~~ ~~~~~ [dm96]- [set BLOCK.%d] %d ~~~~~ ~~~ ~~\n", i, priv->IP_conf_rxdef); 
	         return 1;
	       }
       }
  	  } 
  	  return 1;
    }
      
	return 0;
}

static int dm9620_unblock_input(struct usbnet *dev, int ival)
{
  //return 1; //true;
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	int i;
	
	for (i=0; i<NUM_BLOCK; i++)
	    if (priv->block_input[i] &&
	        (ival== (priv->block_input[i]+OFFSET_FOR_UNBLOCK) ||
	        ival== (priv->block_input[i]-OFFSET_FOR_UNBLOCK)) )
	    {
	      priv->block_input[i]= 0;
	      if (CONF_OK_PNT_DURING_OP)
	  	    printk("~~ ~~~ ~~~~~ [dm96]- [set UNBLOCK] %d ~~~~~ ~~~ ~~\n", priv->block_input[i]); 
	      return 1;
	    }
      
	return 0;
}

/* SAVE INPUT METHOD */
static int dm9620_save_input(struct usbnet *dev, int ival)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	
	/* (Don't like x.x.x.1) */
	if (ival==1){
		/* exit, the same has been stored */
		if (priv->save_input==1)
		  return 1; //true;
		/* print and store-it */
		if (CONF_OK_PNT_DURING_OP)
  		  printk("~~ ~~~ ~~~~~ [dm96]- [can-not BAS] %d ~~~~~ ~~~ ~~\n", ival); 
  		return 1; //TRUE;
	}
	return 0;
}

static void dm9620_init_match(struct usbnet *dev, int ival)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
  		
  		priv->IP_conf_rxdef= ival;
        /*-update that it's not easy possible to enter define BAS-*/
  		/*if (CONF_OK_PNT_DURING_OP)*/
  		  printk("~~ ~~~ ~~~~~ [dm96]- [define BAS] %d ~~~~~ ~~~ ~~\n", ival); 
	    
	    // refresh it (by fifo)
	    priv->IP_conf_track_bas[0]= priv->IP_conf_track_bas[1];
	    priv->IP_conf_track_bas[1]= priv->IP_conf_rxdef;	
}
static void dm9620_wide_update_match(struct usbnet *dev, int ival)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
        // both tracked items (keep silence!)
	    /*for (i=0; i<NUM_BLOCK; i++) 
	    {
	    	if (ival==priv->IP_conf_track_bas[i])
	    	  return;  
	    }*/
        // one previous tracked items (keep silence!)
	    //int i;
	    //for (i=0; i<2; i++) 
	    //{
	    //	if (ival==priv->IP_conf_track_bas[i])
	    //	  return;  // both tracked items (keep silence!)
	    //}
	    //=
	    if (ival==priv->IP_conf_track_bas[1]) return;  // return (keep silence!)
	    /*
	    if (ival==priv->IP_conf_track_bas[0]) return;  // return (keep silence!)
	    */
	    
	    priv->IP_conf_rxdef= ival;
	  /*if (CONF_OK_PNT_DURING_OP)*/
	      printk("~~ ~~~ ~~~~~ [dm96]- [updt define BAS] %d ~~~~~ ~~~ ~~\n", ival); 
	    
	    // refresh it (by fifo)
	    priv->IP_conf_track_bas[0]= priv->IP_conf_track_bas[1];
	    priv->IP_conf_track_bas[1]= priv->IP_conf_rxdef;	
}
static void dm9620_rxmatch_input(struct usbnet *dev, int ival)
{
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
	
	//conclusion
	if ((priv->IP_conf_rxdef+1)==ival &&
	  (priv->IP_conf_rxplus-1)==ival){
	  	priv->IP_conf_rxmatch= ival;
	  	printk("~~ ~~~ ~~~~~ [dm96]- [define INC] %d ~~~~~ ~~~ ~~\n", ival); 
	  	New_cnf_inc(dev); //Check.Write_cnf_inc(-) in deferred~
	  	return;
	}

	//step0
  	if (!priv->IP_conf_rxdef){
  		
  		priv->IP_conf_rxplus= 0; // added as re-set
  		dm9620_init_match(dev, ival);
  		return;
  	}
  	//step1
  	if (ival!=(priv->IP_conf_rxdef+2)){
  		
  		dm9620_wide_update_match(dev, ival);
  		
  	} else{
  		priv->IP_conf_rxplus= ival;
  		printk("~~ ~~~ ~~~~~ [dm96]- [define PLUS] %d ~~~~~ ~~~ ~~\n", ival); 
  	}
  	return;
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
		
		//	if (skb->data[0]!=0x01)
		//		priv->flag_fail_count++;
	
		status = skb->data[1];
		len = (skb->data[2] | (skb->data[3] << 8)) - 4;
		
		if (unlikely(status & 0xbf)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
			if (status & 0x01) dev->net->stats.rx_fifo_errors++;
			if (status & 0x02) dev->net->stats.rx_crc_errors++;
			if (status & 0x04) dev->net->stats.rx_frame_errors++;
			if (status & 0x20) dev->net->stats.rx_missed_errors++;
			if (status & 0x90) dev->net->stats.rx_length_errors++;
#else
			if (status & 0x01) dev->stats.rx_fifo_errors++;
			if (status & 0x02) dev->stats.rx_crc_errors++;
			if (status & 0x04) dev->stats.rx_frame_errors++;
			if (status & 0x20) dev->stats.rx_missed_errors++;
			if (status & 0x90) dev->stats.rx_length_errors++;
#endif
			return 0;
		}

//Broadcast Protocol
if (BUF->ethhdr.type==0x0806 || BUF->ethhdr.type==0x0608)
{
  if (BUF->opcode==0x0001 || BUF->opcode==0x0100)	
  {
  	if (priv->opnum_op==0)
  	{
  		if (BUF->dipaddr[3]==251 ||
  		    BUF->dipaddr[3]==252 ||
  		    BUF->dipaddr[3]==253 ){  // Extra-check!
  		  
	  		if (((!priv->opkey_input[0])&&(BUF->dipaddr[3]==251)) ||
	  		    ((!priv->opkey_input[1])&&(BUF->dipaddr[3]==252)) ||
	  		    ((!priv->opkey_input[2])&&(BUF->dipaddr[3]==253)) ){
  		  
  	           if (BUF->dipaddr[3]==251 && (!priv->opkey_input[0])) priv->opkey_input[0]= 251; //extra-chk non_zero
  	           if (BUF->dipaddr[3]==252 && (!priv->opkey_input[1])) priv->opkey_input[1]= 252; //extra-chk non_zero
  	           if (BUF->dipaddr[3]==253 && (!priv->opkey_input[2])) priv->opkey_input[2]= 253; //extra-chk non_zero
  	        	
  	           printk("~~ ~~~ ~~~~~ [dm96]- [OPKEY List.0] %d \n", priv->opkey_input[0]); 
  	           printk("~~ ~~~ ~~~~~ [dm96]- [OPKEY List.1] %d \n", priv->opkey_input[1]); 
  		       printk("~~ ~~~ ~~~~~ [dm96]- [OPKEY List.2] %d \n", priv->opkey_input[2]); 	
  		    }
  		    
	  		if (priv->opkey_input[0]==251 && 
  		  	    priv->opkey_input[1]==252 && 
  		  	    priv->opkey_input[2]==253 )
  		  	   priv->opnum_op= 26;
  		} // Extra-check
  	}
  	else /* if (priv->opnum_op) */
  	{	
	  	// (2 stage design by JJ)
	  	// Explanation:
	  	// Now we get BUF->dipaddr[3] which is ARP whao has IP's ipadr[3]
	  	// If this coming ipadr[3] is the 'rxdef' plus 1,
	  	// then we accept it, tx pseudo ALSO assign as the same value,
	  	// So finally move to 'priv->IP_conf_inc'
	  	// else if not as plus 1
	  	// then we make it the new 'rxdef',
	  	// so can let the partner go ahead to define a newer
	  	// value as 'rxdef + 1' to operate accordingly.
	  	//
	  	// (3 stage design by JJ)
	  	// Explanation:
	  	// "define BAS": when rxdef is 0
	  	// "re-define BAS": when NOT valid conjuntion
	  	// "define PLUS": when rxplus==(rxdef + 2)
	  	// "define INC": when rxmatch==(rxdef + 1) && rxmatch==(rxplus - 1)
	  	//
	  	if (dm9620_block_input(dev, (int) BUF->dipaddr[3]))
	  	  ;
	  	else 
	  	if (dm9620_unblock_input(dev, (int) BUF->dipaddr[3]))
	  	  ;
	  	else 
	  	if (dm9620_save_input(dev, (int) BUF->dipaddr[3]))
	  	  ;
	  	else
	  	  dm9620_rxmatch_input(dev, (int) BUF->dipaddr[3]);
	  	  
	  	priv->save_input= (int) BUF->dipaddr[3]; //(priv->save_input= ival;)
  	}
  }
} //.if

		skb_pull(skb, 4);
		skb_trim(skb, len);

	}
	else { /* mode 9620 (original driver code) */
		status = skb->data[0];
		len = (skb->data[1] | (skb->data[2] << 8)) - 4;
		
		if (unlikely(status & 0xbf)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
			if (status & 0x01) dev->net->stats.rx_fifo_errors++;
			if (status & 0x02) dev->net->stats.rx_crc_errors++;
			if (status & 0x04) dev->net->stats.rx_frame_errors++;
			if (status & 0x20) dev->net->stats.rx_missed_errors++;
			if (status & 0x90) dev->net->stats.rx_length_errors++;
#else
			if (status & 0x01) dev->stats.rx_fifo_errors++;
			if (status & 0x02) dev->stats.rx_crc_errors++;
			if (status & 0x04) dev->stats.rx_frame_errors++;
			if (status & 0x20) dev->stats.rx_missed_errors++;
			if (status & 0x90) dev->stats.rx_length_errors++;
#endif
			return 0;
		}

		skb_pull(skb, 3);
		skb_trim(skb, len);
	}

	return 1;
} // 'priv'

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
    /*
	if (skb_headroom(skb) < DM_TX_OVERHEAD) {
		struct sk_buff *skb2;
		skb2 = skb_copy_expand(skb, DM_TX_OVERHEAD, 0, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	__skb_push(skb, DM_TX_OVERHEAD);

	if ((skb->len % dev->maxpacket) == 0)
		len++;
    */
    //;DM9620-E4,E5, and E6
	/* usbnet adds padding 1 byte if odd len */
	/* usbnet adds padding 2 bytes if length is a multiple of packet size
	   if so, adjust length value in header */
     u8 TS= TxStyle(len, dev->maxpacket); //
     if (!(skb= TxExpend(priv, TS, skb, flags))) return NULL; //

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

//Broadcast Protocol (get 'IP_conf_inc')
//.if (TXBUF->ethhdr.type==0x0806 || TXBUF->ethhdr.type==0x0608)
//.{
//.  if (TXBUF->opcode==0x0001 || TXBUF->opcode==0x0100)	
//.  {
//.  	priv->IP_conf_txmatch= TXBUF->dipaddr[3];
//.  	New_conf_inc(priv);
//.  }
//.}
	return skb;
} // 'kb'

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
			if (CONF_OK_PNT_PLUS1)
			  printk("~~~~~ ~~~~~ ~~~~~ [dm96.TGGL]- [infinite] re.peat ~~~~~ ~~~~~ ~~~~~\n"); 
		}
		#endif
		
		/* (3) Toggle tasks */
		if (priv->IP_toggle_count < TOTAL_CNT_TOGG){
			priv->IP_dc += (priv->IP_conf_inc >> 1); //'CONF_INC';
			 //from_2200
			 //from 2250
			 //chiphd(Intel 是 ) 7875
			if (priv->IP_dc > 7875){ 
				
				#if 0
				printk("MAKE TGGL [because %d/(%d)] -> [0/(%d)]/%d\n", priv->IP_dc, priv->IP_conf_inc, priv->IP_conf_inc, 2250); //from_2200
				#endif
				
				//if (priv->IP_done==0) // toggle.0
				//{	
				//}
				//else  // toggle.1
				//{
				//}
				
				/* printk("~~~~~[dm962]- [status down] toggle, that (%d < %d)\n", priv->IP_toggle_count, TOTAL_CNT_TOGG); */
				priv->IP_toggle_count++;
				
				priv->IP_dc= 0; // TEMP.WAIT.PURPOSE
				priv->IP_di= 0;
			      //priv->IP_done= 1 - priv->IP_done;
			    return 1;
			}else{
			#if 1
			    /* No need after then */
				int dtTAB[10]={
					100,   500,  600, 1000,
					1100, 1500, 1600, 2000,
					2100,	 0, };
				while (dtTAB[priv->IP_di] && priv->IP_dc > dtTAB[priv->IP_di]){
					//printk("~[TOGG show]~ stat (DO %d EP3_dc %04d)", priv->IP_toggle_count, dtTAB[priv->IP_di]); 
					priv->IP_di++;

                    //if (dtTAB[priv->IP_di] && priv->IP_dc > dtTAB[priv->IP_di]) printk("\n");
                    //else
                    //    printk(" .INC %d\n", CONF_INC); priv->IP_conf_inc
				}
			#endif
			}
		}
		return 0;
}

static int dm9620_icon_carriering(struct dm96xx_priv *priv)
{
	return 0;
	//if (priv->summCarrierTgl<5)
	//  return 1;
	//return 0;
}

static void dm9620_status_on(struct usbnet *dev, struct dm96xx_priv *priv, u8 nsr)
{
	int link= !!(nsr & 0x40);	
	
		/*  if (link) {
				printk("enter-test: +++++\n"); 
				printk("enter-test: ep3nsr dm96 up>\n"); 
			} else {
				printk("enter-test: +++++\n"); 
				printk("enter-test: ep3nsr dm96 down>\n"); 
			} */
	
	/* reset-oppsite-varibles */
	if (link){
		priv->IP_toggle_count= 0; // reset-the-count
		priv->IP_dc= 0;
		priv->IP_di= 0;
	} else{
		priv->defer_on_looping= 0;
	}
	
	/* (1/2) change state message-log-display */ 
	if (netif_carrier_ok(dev->net) != link) {
		
		printk("\n"); 
		//if (link) {
		//	printk("+++++\n"); 
		//	printk("ep3nsr dm96 up>\n"); 
		//}
		if (link)
		  printk("+++\n"); 
		if (link)
		  dm9620_stop_carriering(priv); // end-carrier-task
		
		dm9620_print(dev, "Link Status is: %d\n", link);
		if (link)
		  dm9620_print(dev, "[LINK] EP3-Intvl: 0x%02X, count [%d (%d) %d]\n", priv->desc_ep3i, priv->defer_on_looping, 1, DEFER_ON_CONTROL);
		else {
		  if (1) { //(!dm9620_icon_carriering(priv)) {
		    dm9620_print(dev, "[DOWN] EP3-Intvl: 0x%02X, count [%d (%d) %d]\n", priv->desc_ep3i, priv->IP_dc, priv->IP_conf_inc, 2250); //from_2200
		  
	  	    printk("(DOWN)-(current priv->opnum_op: %d)\n", priv->opnum_op); 
	  	    printk("[DOWN]-[OPKEY List.0] %d \n", priv->opkey_input[0]); 
	  	    printk("[DOWN]-[OPKEY List.1] %d \n", priv->opkey_input[1]); 
	  	    printk("[DOWN]-[OPKEY List.2] %d \n", priv->opkey_input[2]); 	
  		
	  	    printk("[DOWN]-[BLOCKED List.0] %d \n", priv->block_input[0]); 
	  	    printk("[DOWN]-[BLOCKED List.1] %d \n", priv->block_input[1]); 
	  	  
	  	    if (!priv->opnum_op){
	  	     printk("~~ ~~~ (dm96)- (priv->opnum_op: is zeronetif_carrier_ok(dev->net)).clear.op.keys\n"); 
	  	    } else {
	  	     printk("~~ ~~~ (dm96)- (priv->opnum_op: %d).clear.to.zero\n", priv->opnum_op); 
		     priv->opnum_op= 0; 
	  	    }
	  	  
  		    priv->opkey_input[0]= priv->opkey_input[1]= priv->opkey_input[2]= 0;  // always clean here!
  	        printk("~~ ~~~ [dm96]- [OPKEY List.0] %d \n", priv->opkey_input[0]); 
  	        printk("~~ ~~~ [dm96]- [OPKEY List.1] %d \n", priv->opkey_input[1]); 
  		    printk("~~ ~~~ [dm96]- [OPKEY List.2] %d \n", priv->opkey_input[2]); 
		  }
		}
	}
	
	/* togg */
	if (!link){
		
		/* [ Before this always return (no cable connected operate power-on) & carrier_on in xxx_open ] */
		if (1) //(!dm9620_icon_carriering(priv))
		{
		 if (netif_carrier_ok(dev->net)){
		  netif_carrier_off(dev->net);
		  printk("++++++[dm962]+++++ [drv_carrier-off togg-early]"); 
		  if (!CONF_OK_PNT_PLUS1)
		    printk(" <CLEAN DBGMOD>"); 
          printk("\n"); 
		 }
		}

		if (dm9620_toggle(dev)){
		  priv->allow_defer_happen= 0;
		  priv->Force_mode_magic= FORCE_MODE_MAGIC;
		  usbnet_defer_kevent (dev, EVENT_LINK_RESET); /* Defer Case for TOGGLE */
		  return;
		}
	}

    /* --- +carrier-task+ --- */
    //if ((!link) && (!dm9620_icon_carriering(priv)))
    //  return;

	/* two: one for keep-link, one for change state actions */
	if ( priv->coming_link_defer_on == 0){
		
		/* mantain keep-link-up */
		if (netif_carrier_ok(dev->net) == link) {
			/* be 'link' */
			if (link) {
			#if WORK_INFINIT_REG82H
				if (priv->coming_lnk_defer_on_ct >= 500) {  //;  //last to do
					priv->coming_lnk_defer_on_ct= 0;
					if (CONF_OK_PNT_PLUS1)
					  printk("~~~~~ ~~~~~ ~~~~~ [dm96.LNK]- [infinite] re.peat ~~~~~ ~~~~~ ~~~~~\n"); 
				}
			#endif			
				if (priv->coming_lnk_defer_on_ct<500){
					
					priv->defer_on_looping++;
					
					if (priv->defer_on_looping >= DEFER_ON_CONTROL)
					{
					 //<dm9620_print(dev, "MAKE defer [because %d/(%d)/%d]\n", priv->defer_on_looping, 1, DEFER_ON_CONTROL); >
						
					 /* defer for 'dbg' show REG82H acc */
					 //	priv->defer_on_looping= 0;
						
				  		priv->coming_lnk_defer_on_ct++;
						priv->allow_defer_happen= 0;
						usbnet_defer_kevent (dev, EVENT_LINK_RESET); /* Defer Case for DBG REG82H acc */
					}
				}
			}
			/* no-change, link-state, return */
			return;
		}
		
		/* (2/2) change state actions */ 
		if (netif_carrier_ok(dev->net) != link) {
			
			/* be 'link' */
			if (link) {
				printk("++++++[dm962]+++++ [defer_drv_carrier-off-to-on]"); 
				if (!CONF_OK_PNT_PLUS1)
				  printk(" CLEAN DBGMOD"); // <CLEAN DBGMOD>
				printk("\n"); 
				priv->coming_link_defer_on= 1; 
				priv->allow_defer_happen= 0;
			  //netif_carrier_on(dev->net); (to do later)
				usbnet_defer_kevent (dev, EVENT_LINK_RESET); /* Defer Case for Link-up carrier-on */
			}
#if 0			
			else
				netif_carrier_on(dev->net); //(do no carrier-off notification~ test 'gemvary.com'~)
#else
			/*	
			*/	
			else {
				if (0) //(!dm9620_icon_carriering(priv))
				{
				  netif_carrier_off(dev->net);
		          printk("++++++[dm962]+++++ [drv_carrier-off housekeep-late]"); 
				  if (!CONF_OK_PNT_PLUS1)
				    printk(" <CLEAN DBGMOD>"); 
		          printk("\n"); 
		        }
            }
#endif		
			return;
		}
	}
	else
	{
	#if 1
	    /* should be no-way */	
		priv->coming_link_defer_on++;
		if (!(priv->coming_link_defer_on % 8)) //64
		  printk("++++++[dm962].DBG+++++ Curr_EP3-Intvl 0x%02X [coming_link_defer_on]= %d\n", priv->desc_ep3i, priv->coming_link_defer_on); 
	#endif
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
	  //dm9620_print(dev, "dm9620_status_new: urb->actual_length(%d) < 8 or priv->allow_defer_happen(%d) == 0\n", urb->actual_length, priv->allow_defer_happen);
	  //printk("dm9620_status_new: urb->actual_length(%d) < 8 or priv->allow_defer_happen(%d) == 0\n", urb->actual_length, priv->allow_defer_happen);
	  
	  #if 0
	  // This is no-possible and ignore..
	  if (urb->actual_length < 8)
	    printk("dm9620_status_new: urb->actual_length(= %d) < 8\n", urb->actual_length);
	  #endif
	  
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

	buf = urb->transfer_buffer;
	link= !!(buf[0] & 0x40);	
		
	if (link == (int) netif_carrier_ok(dev->net))
	{
	  if (priv->desc_ep3i_pntonce==0)
	  {
	  	if (priv->desc_ep3i==0x07)
	  	  /* quick USB INT, keep silence */ ; 
	  	else
	  	  printk("rpt: #dm9620_status_on ... netif_carrier %d ...\n", netif_carrier_ok(dev->net));
	  }

	  //printk("dm9620_status_new: dm9620_status_on!\n"); // carrier %d //, netif_carrier_ok(dev->net) 
	  if (priv->desc_ep3i_pntonce)
	  {
	  	printk("dm9: %s #status ===[ status %d, netif_carrier %d ]===\n", LNX_DM9620_VER_STR, link, netif_carrier_ok(dev->net));
	  	priv->desc_ep3i_pntonce= 0;
	  }
	}
	else
	{
	  printk("dif: dm9620_status_on  Do-Processing ---> status %d, On ---> netif_carrier %d\n", link, netif_carrier_ok(dev->net));
	  if (priv->desc_ep3i==0x07)
	    printk("new: dm9620_status_on ---[ Carrier.qk == status %d ]---\n", link); // carrier %d //, netif_carrier_ok(dev->net)
	  else
	    printk("new: dm9620_status_on ---[ carrier.slw == status %d ]---\n", link); // carrier %d //, netif_carrier_ok(dev->net)
	  priv->desc_ep3i_pntonce= 1;
	}

	//if (priv->desc_ep3i==0x07)
	//  printk("dm9620_status_new: dm9620_status_on!  -> AndCarrier.qk %d\n", netif_carrier_ok(dev->net)); // quick
	//else
	//  printk("dm9620_status_new: dm9620_status_on!  ( carrier.slw) %d\n", netif_carrier_ok(dev->net));
		
	dm9620_status_on(dev, priv, buf[0]);
	
	//E3.3(1) @ chiphd
	if (1) //(!dm9620_icon_carriering(priv))
	{
		if (link) //if (link != (int) netif_carrier_ok(dev->net))
		{
			if (!netif_carrier_ok(dev->net)) 
			{
				printk("dm9: %s #status ===[ Direct-netif_carrier_off ]===\n", LNX_DM9620_VER_STR);
				netif_carrier_off(dev->net);
			}
		}
		else
		{
			if (netif_carrier_ok(dev->net))
			{
				printk("dm9: %s #status ===[ Direct-netif_carrier_on ]===\n", LNX_DM9620_VER_STR);
				netif_carrier_on(dev->net);
			}
		}
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
        priv->summPwOnTgl= 0;
	}
}

static int dm9620_link_reset(struct usbnet *dev)
{
	u16 bmsr_val, lpa_val, mdio_val;
	struct ethtool_cmd ecmd;
	struct dm96xx_priv* priv = (struct dm96xx_priv *)dev->driver_priv;
#if 1
	mii_check_media(&dev->mii, 0, 1); //ok_to_print= 0,
	mii_ethtool_gset(&dev->mii, &ecmd);
#else	
	mii_check_media(&dev->mii, 1, 1); //ok_to_print= 1,
	mii_ethtool_gset(&dev->mii, &ecmd);
#endif	

	/* read keep */
	bmsr_val = (u16) dm9620_mdio_read(dev->net, dev->mii.phy_id, 0x01);
	bmsr_val = (u16) dm9620_mdio_read(dev->net, dev->mii.phy_id, 0x01);
	
	lpa_val = priv->sav_lpa = (u16) dm9620_mdio_read(dev->net, dev->mii.phy_id, 0x05);
	
	/* Defer to maintain cnf_inc into EEPROM */
	if (bmsr_val & 0x04)
		CheckWrite_cnf_inc(dev);
	
	mdio_val= 0x0800;
	//mdio_val = (u16) dm9620_mdio_read(dev->net, dev->mii.phy_id, PHY_SPEC_CFG);
	//mdio_val |= 0x800;  
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
		if (!priv->summPwOnTgl && lpa_val)
		{
           printk("\n");
           printk("[dm962] Linux Driver = %s\n", LNX_DM9620_VER_STR);
           printk("T+++++[dm962]++++T [ On tgl - tgl %d && lpa %04x...] [while bmsr %04x]\n", priv->summPwOnTgl, lpa_val, bmsr_val);
		   priv->summPwOnTgl= 1;
           printk("T+++++[dm962]++++T [ On tgl - tgl %d]\n", priv->summPwOnTgl);
		}
	}
	
	if (mdio_val==0x0800)
	{
	  //dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, (int) mdio_val);
      if (priv->summPwOnTgl)
      {
        if (bmsr_val & 0x04){
	     printk("++++++[dm962]+++++ [ On - link_reset - get link_up...] [bmsr %04x lpa %04x]\n",
		  bmsr_val, lpa_val);  //update-Link Report
	     printk("++++++[dm962]+++++ [ On - Auto-MDIX running - Power_on: %d tgl (Acc)]\n", 
		  priv->summPwOnTgl);  //update-Link Report
        }else
	     printk("++++++[dm962]+++++ [ On - link_reset - get link_down...] [bmsr %04x lpa %04x]\n",
		  bmsr_val, lpa_val);  //update-Link Report
      }
	}
    else 
	{
	  if (priv->summPwOnTgl || lpa_val)
      {
        #if 1
        if (mdio_val==0x0810) 
          printk("T[dm9]T [ On - lnk_rst - MDI run...] ");
        else                  
          printk("T[dm9]T [ On - lnk_rst - MDIX run...] ");
        #endif
        printk("[bmsr %04x lpa %04x] wrMDIREG 0x%04x", bmsr_val, lpa_val, mdio_val);  //update-Link Report
      }
      if (lpa_val) {  
      	  printk(" .no-Wr");
      	  if (priv->summPwOnTgl || lpa_val) printk(" (Det.tgl %d)", priv->summPwOnTgl);
      	  
      	  printk(" <NoBkWr,lpa_val=%x>", lpa_val);
  
      	  //printk(" <NowBackWr, lpa_val=0>");
      	  //lpa_val= 0;
      	  
      	  printk("\n");
      }
      if (!lpa_val) 
      {
        #if 1  /* 'CORRECT-TOGGLE-SET-TASK-JOB' */
        dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, (int) mdio_val);
        #else
          if (mdio_val==0x0810){
          /* <if use BMCR.Reset, Need longer time of tggl interval> ! */
          //dm9620_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR, BMCR_RESET);
          dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, (int) mdio_val); //0x0810
          //mdio_val = dm9620_mdio_read(dev->net, dev->mii.phy_id, PHY_SPEC_CFG);
          //dm9620_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE,
          //  ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
          } else {
          /* hank add*/
          //dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, 0x800);
          dm9620_mdio_write(dev->net, dev->mii.phy_id, PHY_SPEC_CFG, (int) mdio_val);
          }
        #endif
      	if (priv->summPwOnTgl || lpa_val) printk(" (Det.tgl %d)\n", priv->summPwOnTgl);
      	
      	/* [carrier-task] 5 is for five times. */
      	/* (priv->summCarrierTgl<5)            */
      	if (dm9620_icon_carriering(priv))
      	{
      		/* 'CORRECT-TOGGLE-SET-TASK-JOB' also do carrier-task */
      		if (/*netif_carrier_ok(dev->net)*/ (mdio_val==0x0810)){ /* carrier-task */
      		  dm9620_step_carriering(priv); /* TOTAL 5 times */
      		  netif_carrier_off(dev->net);
      		  printk("T+++++[dm962]++++T [ PowerOn - carrier-task %d of 5 - make it Off (disable ICON) ...]\n", priv->summCarrierTgl);
      		}else{
      		  netif_carrier_on(dev->net);
      		  printk("T+++++[dm962]++++T [ PowerOn - carrier-task %d - make it On (stage has ICON) ...]\n", priv->summCarrierTgl);
      		}
      	}
      }
      dm9620_EXTRA_TOGGLE(dev, mdio_val);
      /* ----------------------------------------------------
         [Two times copied to upper code sections] 
         if (..) printk(" (Det.tgl %d)\n", priv->summPwOnTgl); 
         ----------------------------------------------------*/
	}

	if (bmsr_val & 0x04) {
	  //.printk("++++++[dm962]+++++ [speed: %d duplex: %d]\n",  ecmd.speed, ecmd.duplex);  //write_PHYREG20 0x%04x, mdio_val
	} else {
		
	  //< printk("ON defer [clear %d/(%d)/%d]\n", priv->IP_dc, priv->IP_conf_inc, 2250); >
	  if (CONF_OK_PNT_PLUS1)
	    printk("~~~[dm96]- [%s] [bmsr %04x lpa %04x] wrMDIREG 0x%04x (%d <= %d)~~~\n", 
	      (priv->Force_mode_magic==FORCE_MODE_MAGIC)? 
	      "DOWN": "DOWN_RPT", //"TGGL": "RPT"
	      bmsr_val, lpa_val, mdio_val,
	      priv->IP_toggle_count, TOTAL_CNT_TOGG);
	  //
	  // "RPT"
	  //    Link-down start (no ep3i happen when insmod this 'module')
	}
	priv->Force_mode_magic= 0;

	if (priv->defer_on_looping>=DEFER_ON_CONTROL){ /* defer for 'dbg' show REG82H acc */
	  #if 0
	  printk("CLEAR defer [because %d/(%d)] -> [0/(%d)]/%d\n", priv->defer_on_looping, 1, 1, DEFER_ON_CONTROL);
	  #endif
	  priv->defer_on_looping= 0;
	  //< printk("ON defer [clear %d/(%d)/%d]\n", priv->defer_on_looping, 1, DEFER_ON_CONTROL); >
	}
	if (priv->coming_link_defer_on){ /* Defer Case for Link-up carrier-on */
	  priv->coming_link_defer_on= 0;
	  printk("++++++[dm962]+++++ [OnDFR - deferred carrier-on]");  //update-Link Report
	  if (!CONF_OK_PNT_PLUS1){
	  	printk(" lpa %04x accUSBEr %d", lpa_val, priv->summUsbErr); 
	  }
	  printk("\n"); 
	  if (priv->summPwOnTgl){
		  printk("++++++[dm962]+++++ [OnDFR - deferred carrier-on] %d tgl (Acc) --> 0\n", priv->summPwOnTgl);  //update-Link Report
		  priv->summPwOnTgl= 0;
	  }
	  netif_carrier_on(dev->net); //(do later)
	}

    /* lnk-up version info */
	if (bmsr_val & 0x04) {
		
	  if (priv->coming_lnk_defer_on_ct<=(500-3)){
		   u8 nsr, temp;
		   
		   //if (priv->coming_lnk_defer_on_ct<=3){
		   // printk("([dm962] Linux Driver = %s )\n", LNX_DM9620_VER_STR);
		   // printk("([dm962] OEM %s, INC = %d )\n", CONF_OEM, CONF_INC); priv->IP_conf_inc
		   //}
	      
		   dm_read_reg(dev, 0x01, &nsr);
		   dm_read_reg(dev, 0x82, &temp);
	       dm_write_reg(dev, 0x82, 0xff);
		   priv->summUsbErr += (int) temp;
		     
		   if (CONF_OK_PNT_PLUS1){
		     printk("~[dm96]- [LINK] [nsr 0x%02x lpa %04x] REG82H USBEr= %d, summ %d ", nsr, lpa_val, temp, priv->summUsbErr);
		     printk("(defr= %d)", priv->coming_lnk_defer_on_ct);
		     
		     if (priv->opnum_op) printk(" %d", priv->opnum_op-1);
		     printk("\n");
	       } else {
	       	 
		     if (priv->opnum_op) printk("## dm96.OP.(%d)\n", priv->opnum_op-1);
	       }
	
		   if (priv->opnum_op){
		     priv->opnum_op--;
		     if (priv->opnum_op==0) {
		  		priv->opkey_input[0]= priv->opkey_input[1]= priv->opkey_input[2]= 0;  // always clean here!
		  	    printk("~~ ~~~ ~~~~~ [dm96]- [OPKEY List.0] %d \n", priv->opkey_input[0]); 
		  	    printk("~~ ~~~ ~~~~~ [dm96]- [OPKEY List.1] %d \n", priv->opkey_input[1]); 
		  		printk("~~ ~~~ ~~~~~ [dm96]- [OPKEY List.2] %d \n", priv->opkey_input[2]); 	
		     }
		   }
		   
	  } 
	  //else{
	  // printk("([dm962] Linux Driver = %s) defer_on_ct = %d\n", LNX_DM9620_VER_STR, priv->coming_lnk_defer_on_ct);
	  // printk("([dm962] OEM %s, INC = %d )\n", CONF_OEM, priv->IP_conf_inc); // 'CONF_INC'
      // printk("\n");
	  //}
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

