
XOPTS = -I"C:/ti/xdctools_3_25_00_48/packages/" -Dxdc_target_types__=C:/ti/xdctools_3_25_00_48/packages/ti/targets/arm/elf/std.h -Dxdc_target_name__=A8Fnv

vpath % C:/ti/bios_6_35_01_29/packages/ti/sysbios/
vpath %.c C:/ti/xdctools_3_25_00_48/packages/

CCOPTS    = --endian=little -mv7A8 --abi=eabi --neon --float_support=vfpv3 -q -ms --opt_for_speed=2  --program_level_compile -o3 -g --optimize_with_debug 

XDC_ROOT  = C:/ti/xdctools_3_25_00_48/packages/

BIOS_ROOT = C:/ti/bios_6_35_01_29/packages/ti/sysbios/

BIOS_DEFS =  -Dti_sysbios_BIOS_swiEnabled__D=TRUE -Dti_sysbios_BIOS_taskEnabled__D=TRUE -Dti_sysbios_BIOS_clockEnabled__D=TRUE -Dti_sysbios_knl_Swi_numPriorities__D=16 -Dti_sysbios_BIOS_smpEnabled__D=FALSE -Dti_sysbios_Build_useHwiMacros -Dti_sysbios_knl_Task_deleteTerminatedTasks__D=FALSE -Dti_sysbios_knl_Task_numPriorities__D=16 -Dti_sysbios_knl_Clock_TICK_SOURCE=ti_sysbios_knl_Clock_TickSource_TIMER -Dti_sysbios_knl_Clock_TICK_MODE=ti_sysbios_knl_Clock_TickMode_PERIODIC -Dti_sysbios_utils_Load_taskEnabled__D=TRUE -Dti_sysbios_utils_Load_swiEnabled__D=TRUE -Dti_sysbios_family_arm_a8_intcps_Hwi_dispatcherSwiSupport__D=TRUE -Dti_sysbios_family_arm_a8_intcps_Hwi_dispatcherTaskSupport__D=TRUE -Dti_sysbios_family_arm_a8_intcps_Hwi_dispatcherAutoNestingSupport__D=TRUE -Dti_sysbios_family_arm_a8_intcps_Hwi_dispatcherIrpTrackingSupport__D=TRUE -Dti_sysbios_knl_Semaphore_supportsEvents__D=FALSE -Dti_sysbios_family_arm_exc_Exception_enableDecode__D=TRUE

BIOS_INC  = -I"C:/ti/bios_6_35_01_29/packages/"

CC = C:/ti/ccsv5/tools/compiler/arm_5.0.5/bin/armcl -c $(CCOPTS) -I C:/ti/ccsv5/tools/compiler/arm_5.0.5/include
ASM = C:/ti/ccsv5/tools/compiler/arm_5.0.5/bin/armcl -c $(CCOPTS) -I C:/ti/ccsv5/tools/compiler/arm_5.0.5/include
AR = C:/ti/ccsv5/tools/compiler/arm_5.0.5/bin/armar rq

DEL = cmd.exe /q /c del /f
CP = cmd.exe /q /c copy /Y

define RM
	$(if $(wildcard $1),$(DEL) $1,:)
endef

define ASSEMBLE
	@echo asmea8fnv $< ...
	@$(ASM) $(BIOS_DEFS) $(XOPTS) $(BIOS_INC) $<
endef

all: sysbios.aea8fnv

arm_IntrinsicsSupport_asm.obj: family/arm/IntrinsicsSupport_asm.asm makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=arm_IntrinsicsSupport_asm.obj

arm_TaskSupport_asm.obj: family/arm/TaskSupport_asm.asm makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=arm_TaskSupport_asm.obj

exc_Exception_asm.obj: family/arm/exc/Exception_asm.asm makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=exc_Exception_asm.obj

a8_TimestampProvider_asm.obj: family/arm/a8/TimestampProvider_asm.asm makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=a8_TimestampProvider_asm.obj

a8_Mmu_asm.obj: family/arm/a8/Mmu_asm.asm makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=a8_Mmu_asm.obj

a8_Cache_asm.obj: family/arm/a8/Cache_asm.asm makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=a8_Cache_asm.obj

intcps_Hwi_asm.obj: family/arm/a8/intcps/Hwi_asm.s470 makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=intcps_Hwi_asm.obj

intcps_Hwi_asm_switch.obj: family/arm/a8/intcps/Hwi_asm_switch.s470 makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=intcps_Hwi_asm_switch.obj

gptimer_Timer_asm.obj: timers/gptimer/Timer_asm.s470 makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=gptimer_Timer_asm.obj

dmtimer_Timer_asm.obj: timers/dmtimer/Timer_asm.s470 makefile
	@-$(call RM, $@)
	$(ASSEMBLE) --output_file=dmtimer_Timer_asm.obj


BIOS.obj: BIOS.c family/arm/IntrinsicsSupport.c family/arm/TaskSupport.c hal/Hwi.c hal/Hwi_stack.c hal/Hwi_startup.c hal/Timer.c hal/Cache.c knl/Clock.c knl/Idle.c knl/Intrinsics.c knl/Event.c knl/Queue.c knl/Semaphore.c knl/Swi.c knl/Swi_andn.c knl/Task.c family/arm/a8/Cache.c family/arm/a8/Mmu.c family/arm/a8/TimestampProvider.c utils/Load.c gates/GateHwi.c gates/GateMutex.c family/arm/a8/intcps/Hwi.c family/arm/exc/Exception.c timers/dmtimer/Timer.c family/arm/a8/ti81xx/TimerSupport.c heaps/HeapMem.c makefile
	@-$(call RM, $@)
	@echo clea8fnv $< ...
	@$(CC) $(BIOS_DEFS) $(XOPTS) $(BIOS_INC) \
	$(BIOS_ROOT)BIOS.c \
	$(BIOS_ROOT)family/arm/IntrinsicsSupport.c \
	$(BIOS_ROOT)family/arm/TaskSupport.c \
	$(BIOS_ROOT)hal/Hwi.c \
	$(BIOS_ROOT)hal/Hwi_stack.c \
	$(BIOS_ROOT)hal/Hwi_startup.c \
	$(BIOS_ROOT)hal/Timer.c \
	$(BIOS_ROOT)hal/Cache.c \
	$(BIOS_ROOT)knl/Clock.c \
	$(BIOS_ROOT)knl/Idle.c \
	$(BIOS_ROOT)knl/Intrinsics.c \
	$(BIOS_ROOT)knl/Event.c \
	$(BIOS_ROOT)knl/Queue.c \
	$(BIOS_ROOT)knl/Semaphore.c \
	$(BIOS_ROOT)knl/Swi.c \
	$(BIOS_ROOT)knl/Swi_andn.c \
	$(BIOS_ROOT)knl/Task.c \
	$(BIOS_ROOT)family/arm/a8/Cache.c \
	$(BIOS_ROOT)family/arm/a8/Mmu.c \
	$(BIOS_ROOT)family/arm/a8/TimestampProvider.c \
	$(BIOS_ROOT)utils/Load.c \
	$(BIOS_ROOT)gates/GateHwi.c \
	$(BIOS_ROOT)gates/GateMutex.c \
	$(BIOS_ROOT)family/arm/a8/intcps/Hwi.c \
	$(BIOS_ROOT)family/arm/exc/Exception.c \
	$(BIOS_ROOT)timers/dmtimer/Timer.c \
	$(BIOS_ROOT)family/arm/a8/ti81xx/TimerSupport.c \
	$(BIOS_ROOT)heaps/HeapMem.c \

sysbios.aea8fnv: BIOS.obj arm_IntrinsicsSupport_asm.obj arm_TaskSupport_asm.obj exc_Exception_asm.obj a8_TimestampProvider_asm.obj a8_Mmu_asm.obj a8_Cache_asm.obj intcps_Hwi_asm.obj intcps_Hwi_asm_switch.obj gptimer_Timer_asm.obj dmtimer_Timer_asm.obj
	@-$(call RM, $@)
	@echo area8fnv $^ ...
	@$(AR) $@ $^


clean:
	@-$(call RM, sysbios.aea8fnv)
	@-$(call RM, BIOS.obj)
	@-$(call RM, arm_IntrinsicsSupport_asm.obj)
	@-$(call RM, arm_TaskSupport_asm.obj)
	@-$(call RM, exc_Exception_asm.obj)
	@-$(call RM, a8_TimestampProvider_asm.obj)
	@-$(call RM, a8_Mmu_asm.obj)
	@-$(call RM, a8_Cache_asm.obj)
	@-$(call RM, intcps_Hwi_asm.obj)
	@-$(call RM, intcps_Hwi_asm_switch.obj)
	@-$(call RM, gptimer_Timer_asm.obj)
	@-$(call RM, dmtimer_Timer_asm.obj)
