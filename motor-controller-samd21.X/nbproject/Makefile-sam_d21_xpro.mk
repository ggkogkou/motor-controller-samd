#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-sam_d21_xpro.mk)" "nbproject/Makefile-local-sam_d21_xpro.mk"
include nbproject/Makefile-local-sam_d21_xpro.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=sam_d21_xpro
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/config/sam_d21_xpro/peripheral/clock/plib_clock.c ../src/config/sam_d21_xpro/peripheral/evsys/plib_evsys.c ../src/config/sam_d21_xpro/peripheral/nvic/plib_nvic.c ../src/config/sam_d21_xpro/peripheral/nvmctrl/plib_nvmctrl.c ../src/config/sam_d21_xpro/peripheral/port/plib_port.c ../src/config/sam_d21_xpro/peripheral/sercom/usart/plib_sercom3_usart.c ../src/config/sam_d21_xpro/peripheral/systick/plib_systick.c ../src/config/sam_d21_xpro/peripheral/tcc/plib_tcc0.c ../src/config/sam_d21_xpro/initialization.c ../src/config/sam_d21_xpro/interrupts.c ../src/config/sam_d21_xpro/exceptions.c ../src/config/sam_d21_xpro/libc_syscalls.c ../src/config/sam_d21_xpro/startup_gcc.c ../src/main.cpp

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/600686086/plib_clock.o ${OBJECTDIR}/_ext/602835572/plib_evsys.o ${OBJECTDIR}/_ext/2097924074/plib_nvic.o ${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o ${OBJECTDIR}/_ext/2097977225/plib_port.o ${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o ${OBJECTDIR}/_ext/869718558/plib_systick.o ${OBJECTDIR}/_ext/2010529844/plib_tcc0.o ${OBJECTDIR}/_ext/128142748/initialization.o ${OBJECTDIR}/_ext/128142748/interrupts.o ${OBJECTDIR}/_ext/128142748/exceptions.o ${OBJECTDIR}/_ext/128142748/libc_syscalls.o ${OBJECTDIR}/_ext/128142748/startup_gcc.o ${OBJECTDIR}/_ext/1360937237/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/600686086/plib_clock.o.d ${OBJECTDIR}/_ext/602835572/plib_evsys.o.d ${OBJECTDIR}/_ext/2097924074/plib_nvic.o.d ${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o.d ${OBJECTDIR}/_ext/2097977225/plib_port.o.d ${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o.d ${OBJECTDIR}/_ext/869718558/plib_systick.o.d ${OBJECTDIR}/_ext/2010529844/plib_tcc0.o.d ${OBJECTDIR}/_ext/128142748/initialization.o.d ${OBJECTDIR}/_ext/128142748/interrupts.o.d ${OBJECTDIR}/_ext/128142748/exceptions.o.d ${OBJECTDIR}/_ext/128142748/libc_syscalls.o.d ${OBJECTDIR}/_ext/128142748/startup_gcc.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/600686086/plib_clock.o ${OBJECTDIR}/_ext/602835572/plib_evsys.o ${OBJECTDIR}/_ext/2097924074/plib_nvic.o ${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o ${OBJECTDIR}/_ext/2097977225/plib_port.o ${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o ${OBJECTDIR}/_ext/869718558/plib_systick.o ${OBJECTDIR}/_ext/2010529844/plib_tcc0.o ${OBJECTDIR}/_ext/128142748/initialization.o ${OBJECTDIR}/_ext/128142748/interrupts.o ${OBJECTDIR}/_ext/128142748/exceptions.o ${OBJECTDIR}/_ext/128142748/libc_syscalls.o ${OBJECTDIR}/_ext/128142748/startup_gcc.o ${OBJECTDIR}/_ext/1360937237/main.o

# Source Files
SOURCEFILES=../src/config/sam_d21_xpro/peripheral/clock/plib_clock.c ../src/config/sam_d21_xpro/peripheral/evsys/plib_evsys.c ../src/config/sam_d21_xpro/peripheral/nvic/plib_nvic.c ../src/config/sam_d21_xpro/peripheral/nvmctrl/plib_nvmctrl.c ../src/config/sam_d21_xpro/peripheral/port/plib_port.c ../src/config/sam_d21_xpro/peripheral/sercom/usart/plib_sercom3_usart.c ../src/config/sam_d21_xpro/peripheral/systick/plib_systick.c ../src/config/sam_d21_xpro/peripheral/tcc/plib_tcc0.c ../src/config/sam_d21_xpro/initialization.c ../src/config/sam_d21_xpro/interrupts.c ../src/config/sam_d21_xpro/exceptions.c ../src/config/sam_d21_xpro/libc_syscalls.c ../src/config/sam_d21_xpro/startup_gcc.c ../src/main.cpp

# Pack Options 
PACK_COMMON_OPTIONS=-I "${DFP_DIR}/samd21a/include"  -I "${CMSIS_DIR}/CMSIS/Core/Include"



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-sam_d21_xpro.mk ${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=SAMD21J18A
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/600686086/plib_clock.o: ../src/config/sam_d21_xpro/peripheral/clock/plib_clock.c  .generated_files/flags/sam_d21_xpro/5b6b0f92383a911f124962e84d292bb9abc26d90 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/600686086" 
	@${RM} ${OBJECTDIR}/_ext/600686086/plib_clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/600686086/plib_clock.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/600686086/plib_clock.o.d" -o ${OBJECTDIR}/_ext/600686086/plib_clock.o ../src/config/sam_d21_xpro/peripheral/clock/plib_clock.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/602835572/plib_evsys.o: ../src/config/sam_d21_xpro/peripheral/evsys/plib_evsys.c  .generated_files/flags/sam_d21_xpro/b2d0b2171dd908e16264e80259d66c212ce5317c .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/602835572" 
	@${RM} ${OBJECTDIR}/_ext/602835572/plib_evsys.o.d 
	@${RM} ${OBJECTDIR}/_ext/602835572/plib_evsys.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/602835572/plib_evsys.o.d" -o ${OBJECTDIR}/_ext/602835572/plib_evsys.o ../src/config/sam_d21_xpro/peripheral/evsys/plib_evsys.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2097924074/plib_nvic.o: ../src/config/sam_d21_xpro/peripheral/nvic/plib_nvic.c  .generated_files/flags/sam_d21_xpro/389196d8147b445904b984839e5d899a47796789 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2097924074" 
	@${RM} ${OBJECTDIR}/_ext/2097924074/plib_nvic.o.d 
	@${RM} ${OBJECTDIR}/_ext/2097924074/plib_nvic.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2097924074/plib_nvic.o.d" -o ${OBJECTDIR}/_ext/2097924074/plib_nvic.o ../src/config/sam_d21_xpro/peripheral/nvic/plib_nvic.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o: ../src/config/sam_d21_xpro/peripheral/nvmctrl/plib_nvmctrl.c  .generated_files/flags/sam_d21_xpro/6adfaf24de7baabd2c639739fb9193f0f846a219 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1104193656" 
	@${RM} ${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o.d 
	@${RM} ${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o.d" -o ${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o ../src/config/sam_d21_xpro/peripheral/nvmctrl/plib_nvmctrl.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2097977225/plib_port.o: ../src/config/sam_d21_xpro/peripheral/port/plib_port.c  .generated_files/flags/sam_d21_xpro/10ad75ff1272cb034417092277245b639a559261 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2097977225" 
	@${RM} ${OBJECTDIR}/_ext/2097977225/plib_port.o.d 
	@${RM} ${OBJECTDIR}/_ext/2097977225/plib_port.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2097977225/plib_port.o.d" -o ${OBJECTDIR}/_ext/2097977225/plib_port.o ../src/config/sam_d21_xpro/peripheral/port/plib_port.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o: ../src/config/sam_d21_xpro/peripheral/sercom/usart/plib_sercom3_usart.c  .generated_files/flags/sam_d21_xpro/486463560113f4c575f59c3942508120a8f5fc50 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1136425057" 
	@${RM} ${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o.d" -o ${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o ../src/config/sam_d21_xpro/peripheral/sercom/usart/plib_sercom3_usart.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/869718558/plib_systick.o: ../src/config/sam_d21_xpro/peripheral/systick/plib_systick.c  .generated_files/flags/sam_d21_xpro/c6a6dca672a06e7a00cedbc64d8f495d22378b47 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/869718558" 
	@${RM} ${OBJECTDIR}/_ext/869718558/plib_systick.o.d 
	@${RM} ${OBJECTDIR}/_ext/869718558/plib_systick.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/869718558/plib_systick.o.d" -o ${OBJECTDIR}/_ext/869718558/plib_systick.o ../src/config/sam_d21_xpro/peripheral/systick/plib_systick.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2010529844/plib_tcc0.o: ../src/config/sam_d21_xpro/peripheral/tcc/plib_tcc0.c  .generated_files/flags/sam_d21_xpro/6cd561603841eb6419221a60caec413a718b5c0a .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2010529844" 
	@${RM} ${OBJECTDIR}/_ext/2010529844/plib_tcc0.o.d 
	@${RM} ${OBJECTDIR}/_ext/2010529844/plib_tcc0.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2010529844/plib_tcc0.o.d" -o ${OBJECTDIR}/_ext/2010529844/plib_tcc0.o ../src/config/sam_d21_xpro/peripheral/tcc/plib_tcc0.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/initialization.o: ../src/config/sam_d21_xpro/initialization.c  .generated_files/flags/sam_d21_xpro/51a9707ff440a436d9379b618835f6617511baa2 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/initialization.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/initialization.o.d" -o ${OBJECTDIR}/_ext/128142748/initialization.o ../src/config/sam_d21_xpro/initialization.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/interrupts.o: ../src/config/sam_d21_xpro/interrupts.c  .generated_files/flags/sam_d21_xpro/f54ec5e34ab9c87e29a717e7d1b1ab42aea56bfd .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/interrupts.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/interrupts.o.d" -o ${OBJECTDIR}/_ext/128142748/interrupts.o ../src/config/sam_d21_xpro/interrupts.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/exceptions.o: ../src/config/sam_d21_xpro/exceptions.c  .generated_files/flags/sam_d21_xpro/3b5ace83296b2c04cce038331b5df18dd597ffe9 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/exceptions.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/exceptions.o.d" -o ${OBJECTDIR}/_ext/128142748/exceptions.o ../src/config/sam_d21_xpro/exceptions.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/libc_syscalls.o: ../src/config/sam_d21_xpro/libc_syscalls.c  .generated_files/flags/sam_d21_xpro/70cf1a70bc1c768a578f4e49055c4da73cccbe2a .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/libc_syscalls.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/libc_syscalls.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/libc_syscalls.o.d" -o ${OBJECTDIR}/_ext/128142748/libc_syscalls.o ../src/config/sam_d21_xpro/libc_syscalls.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/startup_gcc.o: ../src/config/sam_d21_xpro/startup_gcc.c  .generated_files/flags/sam_d21_xpro/1dbd03089da02ba7216ff21380bea62a4f0c5795 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/startup_gcc.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/startup_gcc.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/startup_gcc.o.d" -o ${OBJECTDIR}/_ext/128142748/startup_gcc.o ../src/config/sam_d21_xpro/startup_gcc.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/600686086/plib_clock.o: ../src/config/sam_d21_xpro/peripheral/clock/plib_clock.c  .generated_files/flags/sam_d21_xpro/a85c433c31305a6bfe56582c6077db912a922e87 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/600686086" 
	@${RM} ${OBJECTDIR}/_ext/600686086/plib_clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/600686086/plib_clock.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/600686086/plib_clock.o.d" -o ${OBJECTDIR}/_ext/600686086/plib_clock.o ../src/config/sam_d21_xpro/peripheral/clock/plib_clock.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/602835572/plib_evsys.o: ../src/config/sam_d21_xpro/peripheral/evsys/plib_evsys.c  .generated_files/flags/sam_d21_xpro/710b71845ef4e72039f76af79e774a5df68a07a5 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/602835572" 
	@${RM} ${OBJECTDIR}/_ext/602835572/plib_evsys.o.d 
	@${RM} ${OBJECTDIR}/_ext/602835572/plib_evsys.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/602835572/plib_evsys.o.d" -o ${OBJECTDIR}/_ext/602835572/plib_evsys.o ../src/config/sam_d21_xpro/peripheral/evsys/plib_evsys.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2097924074/plib_nvic.o: ../src/config/sam_d21_xpro/peripheral/nvic/plib_nvic.c  .generated_files/flags/sam_d21_xpro/4c315aba97eb65985013c212df978e2708d09f1f .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2097924074" 
	@${RM} ${OBJECTDIR}/_ext/2097924074/plib_nvic.o.d 
	@${RM} ${OBJECTDIR}/_ext/2097924074/plib_nvic.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2097924074/plib_nvic.o.d" -o ${OBJECTDIR}/_ext/2097924074/plib_nvic.o ../src/config/sam_d21_xpro/peripheral/nvic/plib_nvic.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o: ../src/config/sam_d21_xpro/peripheral/nvmctrl/plib_nvmctrl.c  .generated_files/flags/sam_d21_xpro/1c7b07ca89adcba3d02c3ee76595b2e6324fce38 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1104193656" 
	@${RM} ${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o.d 
	@${RM} ${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o.d" -o ${OBJECTDIR}/_ext/1104193656/plib_nvmctrl.o ../src/config/sam_d21_xpro/peripheral/nvmctrl/plib_nvmctrl.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2097977225/plib_port.o: ../src/config/sam_d21_xpro/peripheral/port/plib_port.c  .generated_files/flags/sam_d21_xpro/e5d8e37992e7140877eebe3b3a70fcf6d9a1e2c4 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2097977225" 
	@${RM} ${OBJECTDIR}/_ext/2097977225/plib_port.o.d 
	@${RM} ${OBJECTDIR}/_ext/2097977225/plib_port.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2097977225/plib_port.o.d" -o ${OBJECTDIR}/_ext/2097977225/plib_port.o ../src/config/sam_d21_xpro/peripheral/port/plib_port.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o: ../src/config/sam_d21_xpro/peripheral/sercom/usart/plib_sercom3_usart.c  .generated_files/flags/sam_d21_xpro/b9a2722b8a6e062a3840adea7268785633f0cf78 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1136425057" 
	@${RM} ${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o.d" -o ${OBJECTDIR}/_ext/1136425057/plib_sercom3_usart.o ../src/config/sam_d21_xpro/peripheral/sercom/usart/plib_sercom3_usart.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/869718558/plib_systick.o: ../src/config/sam_d21_xpro/peripheral/systick/plib_systick.c  .generated_files/flags/sam_d21_xpro/abfded0fb5571d97f15dff14446c68a660d37c72 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/869718558" 
	@${RM} ${OBJECTDIR}/_ext/869718558/plib_systick.o.d 
	@${RM} ${OBJECTDIR}/_ext/869718558/plib_systick.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/869718558/plib_systick.o.d" -o ${OBJECTDIR}/_ext/869718558/plib_systick.o ../src/config/sam_d21_xpro/peripheral/systick/plib_systick.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2010529844/plib_tcc0.o: ../src/config/sam_d21_xpro/peripheral/tcc/plib_tcc0.c  .generated_files/flags/sam_d21_xpro/b11b501d7acbf97266250ed9d920db51717092fe .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2010529844" 
	@${RM} ${OBJECTDIR}/_ext/2010529844/plib_tcc0.o.d 
	@${RM} ${OBJECTDIR}/_ext/2010529844/plib_tcc0.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2010529844/plib_tcc0.o.d" -o ${OBJECTDIR}/_ext/2010529844/plib_tcc0.o ../src/config/sam_d21_xpro/peripheral/tcc/plib_tcc0.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/initialization.o: ../src/config/sam_d21_xpro/initialization.c  .generated_files/flags/sam_d21_xpro/60201386a44f64028988d2d4fabce91677cd5f5b .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/initialization.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/initialization.o.d" -o ${OBJECTDIR}/_ext/128142748/initialization.o ../src/config/sam_d21_xpro/initialization.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/interrupts.o: ../src/config/sam_d21_xpro/interrupts.c  .generated_files/flags/sam_d21_xpro/d5ac780e40243568934e95d341f2b6c5a62115e2 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/interrupts.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/interrupts.o.d" -o ${OBJECTDIR}/_ext/128142748/interrupts.o ../src/config/sam_d21_xpro/interrupts.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/exceptions.o: ../src/config/sam_d21_xpro/exceptions.c  .generated_files/flags/sam_d21_xpro/6cf7effe02e12f1abfe8fa2fa99ec748c9569542 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/exceptions.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/exceptions.o.d" -o ${OBJECTDIR}/_ext/128142748/exceptions.o ../src/config/sam_d21_xpro/exceptions.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/libc_syscalls.o: ../src/config/sam_d21_xpro/libc_syscalls.c  .generated_files/flags/sam_d21_xpro/59b63f04db16c49ff4a60902a6b3b6c82a90768b .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/libc_syscalls.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/libc_syscalls.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/libc_syscalls.o.d" -o ${OBJECTDIR}/_ext/128142748/libc_syscalls.o ../src/config/sam_d21_xpro/libc_syscalls.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/128142748/startup_gcc.o: ../src/config/sam_d21_xpro/startup_gcc.c  .generated_files/flags/sam_d21_xpro/226301c3d9d721a43ead2ea5207a31a0b0a9712 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/128142748" 
	@${RM} ${OBJECTDIR}/_ext/128142748/startup_gcc.o.d 
	@${RM} ${OBJECTDIR}/_ext/128142748/startup_gcc.o 
	${MP_CPPC}  $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -mlong-calls -I "../src/config/sam_d21_xpro" -I "../src/packs/ATSAMD21J18A_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/128142748/startup_gcc.o.d" -o ${OBJECTDIR}/_ext/128142748/startup_gcc.o ../src/config/sam_d21_xpro/startup_gcc.c  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.cpp  .generated_files/flags/sam_d21_xpro/c9da8125e81cd689f7e884db3224bcd5f2d94b2 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CPPC} $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus -g -D__DEBUG  -gdwarf-2  -x c++ -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -fno-rtti -fno-exceptions -mlong-calls -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.cpp  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.cpp  .generated_files/flags/sam_d21_xpro/aa6e83eebafa408836471a6449e482d011b447a2 .generated_files/flags/sam_d21_xpro/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CPPC} $(MP_EXTRA_CC_PRE) -mcpu=cortex-m0plus  -x c++ -c -D__$(MP_PROCESSOR_OPTION)__  -mthumb ${PACK_COMMON_OPTIONS}  -Os -ffunction-sections -fno-rtti -fno-exceptions -mlong-calls -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.cpp  -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CPPC} $(MP_EXTRA_LD_PRE) -mcpu=cortex-m0plus   -gdwarf-2  -D__$(MP_PROCESSOR_OPTION)__    -mthumb --specs=nosys.specs -Wl,-Map="${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.map"  -o ${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}      -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1 -Wl,--gc-sections  
	
	
	
	
	
	
else
${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CPPC} $(MP_EXTRA_LD_PRE) -mcpu=cortex-m0plus  -D__$(MP_PROCESSOR_OPTION)__    -mthumb --specs=nosys.specs -Wl,-Map="${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.map"  -o ${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}      -DXPRJ_sam_d21_xpro=$(CND_CONF)  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION) -Wl,--gc-sections  
	
	${MP_CC_DIR}/arm-none-eabi-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature "${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" "${DISTDIR}/motor-controller-samd21.X.${IMAGE_TYPE}.hex"
	
	
	
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
