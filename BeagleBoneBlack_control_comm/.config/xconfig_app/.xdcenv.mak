#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/ipc_1_25_00_04/packages;C:/ti/bios_6_35_01_29/packages;C:/ti/xdais_7_21_01_07/packages;C:/ti/xdais_7_21_01_07/examples;C:/ti/uia_1_03_00_02/packages;C:/ti/ccsv5/ccs_base;C:/CSS_prj/BeagleBoneBlack_control_comm/.config
override XDCROOT = C:/ti/xdctools_3_25_00_48
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/ipc_1_25_00_04/packages;C:/ti/bios_6_35_01_29/packages;C:/ti/xdais_7_21_01_07/packages;C:/ti/xdais_7_21_01_07/examples;C:/ti/uia_1_03_00_02/packages;C:/ti/ccsv5/ccs_base;C:/CSS_prj/BeagleBoneBlack_control_comm/.config;C:/ti/xdctools_3_25_00_48/packages;..
HOSTOS = Windows
endif
