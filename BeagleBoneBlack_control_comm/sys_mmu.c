/**
 * \file   sys_mmu.c
 *
 * \brief  SYS/BIOS MMU Driver source file
 *
 * Copyright (c) 2012 Texas Instruments Incorporated ALL RIGHTS RESERVED
 * 
*/

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/arm/a8/Mmu.h>

#include "sys_mmu.h"


int mmuInit(SYS_MMU_ENTRY mmuEntries[])
{
    unsigned short itr = 0;
    Mmu_FirstLevelDescAttrs attrs;

    if(NULL == mmuEntries)
    	return -1;

	Mmu_disable();

	Mmu_initDescAttrs(&attrs);
	
	attrs.type = Mmu_FirstLevelDesc_SECTION;
	attrs.domain = 0;
	attrs.imp = 1; 
	attrs.accPerm = 3;
    
    
    for(itr = 0 ; mmuEntries[itr].physAddr != (void*)0xFFFFFFFF ; itr++)
    {
        attrs.bufferable = (mmuEntries[itr].bufferable)?1:0;
        attrs.cacheable = (mmuEntries[itr].cacheable)?1:0;
        Mmu_setFirstLevelDesc((Ptr)(mmuEntries[itr].virtualAddr), (Ptr)(mmuEntries[itr].physAddr) , &attrs);  // PWM
    }
	Mmu_enable();

    return 0;
}

