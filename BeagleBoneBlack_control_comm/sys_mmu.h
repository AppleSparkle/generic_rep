/*
 * \file   sys_mmu.h
 *
 * Copyright (c) 2012 Texas Instruments Incorporated ALL RIGHTS RESERVED
 * 
*/
#ifndef SYS_MMU_H
#define SYS_MMU_H

typedef struct _sys_mmu_entry
{
    void* virtualAddr;
    void* physAddr;
    unsigned char bufferable;
    unsigned char cacheable;
    
}SYS_MMU_ENTRY;

int mmuInit(SYS_MMU_ENTRY mmuEntries[]);

#endif
