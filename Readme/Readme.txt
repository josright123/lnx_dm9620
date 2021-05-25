[Configuration]
Driver Version: 2.60.1:
 #define LNX_DM9620_VER_STR  "V2.60.1 WITH EXTPHY 2021 CLEAN_AND_CTLPRINT"

Kernel select:
 #define LNX_KERNEL_v511	1

 If Linux kernel is new such as v5.11.0 and later, define 'LNX_KERNEL_v511' to 1 

PHY select:
 #define LNX_DRV_SIMPLE_PHY_AND_EXT_PHY	1

 Auto PHY selection, for either internal PHY or external PHY.

 #define LNX_DRV_SIMPLE_PHY_AND_EXT_PHY	0

 Simulate as external phy's control flow
 
 EXT_PHY mdc/mdio select:
  #define LNX_DRV_EXT_PHY_HAS_MDIO		1
  
  External phy mdc/mdio connection
  
  #define LNX_DRV_EXT_PHY_HAS_MDIO		0
  
  External phy no mdc/mdio connection, force to always link_up

[DriverControl]
 #define WORK_INFINIT_TOGG			1	//Toggle Infinite enable, otherwise only 'TOTAL_CNT_TOGG' times (TOTAL_CNT_TOGG = 36)

 Essential for software auto-mdix enhancement

 #define WORK_NB_CTLP				1	//Control_printing for new buffer

 For insurence readable for dm9620_bind() procedure

 #define WORK_TOGG_CTLP				0 //1: fewer toogle print, 0: normal toggle print(good "priv->IP_dc += 63;" toggle, normal toggle print is OK.)
										//Control_printing for toggle-process [recommanded]
 It can huge reduce toggle task printing

 #define WORK_LNK_OKP				0	//DBG print for during LInk-up

 It can help check the EP3-interrupt polling while in link-up status (such as in a stable state). 
