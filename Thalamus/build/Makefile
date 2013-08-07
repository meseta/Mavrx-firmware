###############################################################################
# Copyright (c) 2011, Universal Air Ltd. All rights reserved.                 #
# Source and binaries are released under the BSD 3-Clause license             #
# See readme_forebrain.txt files for the text of the license                  #
###############################################################################

CORTEX = cortex-m3
MPU = LPC13xx

### Location definitions
COMPILER = arm-none-eabi-
CC = $(COMPILER)gcc
AS = $(COMPILER)gcc
LD = $(COMPILER)gcc
SIZE = $(COMPILER)size
OBJCOPY = $(COMPILER)objcopy
OBJDUMP = $(COMPILER)objdump
CHKSM = checksum
PRJPATH = ./build

LD_SCRIPT = $(PRJPATH)/linker.ld
INCLUDE = -I $(PRJPATH)
INCLUDE += -I $(PRJPATH)/../
VPATH = $(PRJPATH)

### Libraries
VPATH += $(PRJPATH)/FatFs

### Compile ALL *.c files found in the directories (don't leave non-project
### *.c files lying around, or this makefile might attempt to include them
### in your project!

SRCS=$(wildcard *.c)
SRCS+=$(wildcard $(PRJPATH)/*.c)
#SRCS+=$(wildcard $(PRJPATH)/FatFs/*.c)

OBJS=$(SRCS:.c=.o)

### Compiler stuff

CFLAGS  = -c -g -O3 $(INCLUDE) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CORTEX) -DTARGET=$(MPU) -fno-builtin
ASFLAGS = -c -g -O3 $(INCLUDE) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CORTEX) -D__ASSEMBLY__ -x assembler-with-cpp
LDFLAGS = -nostartfiles -mthumb -mcpu=$(CORTEX) -Wl,--gc-sections
OCFLAGS = --strip-unneeded

all: firmware

$(PRJPATH)/%.o : %.c
	$(CC) $(CFLAGS) -o $@ $<

$(PRJPATH)/%.o : %.s
	$(AS) $(ASFLAGS) -o $@ $<

firmware: $(OBJS)
	-@echo "*** Compiling... ***"
	$(LD) $(LDFLAGS) -T $(LD_SCRIPT) -o $(PRJPATH)/firmware.elf $(OBJS) -lm
	-@echo ""
	-@echo "*** Converting... ***"
	$(OBJCOPY) $(OCFLAGS) -O binary $(PRJPATH)/firmware.elf firmware.bin
	$(OBJCOPY) $(OCFLAGS) -O ihex $(PRJPATH)/firmware.elf firmware.hex
	-@echo ""
	-@echo "*** Adding Checksum... ***"
	$(CHKSM) firmware.bin
	-@echo ""
	-@echo "*** Getting Size information... ***"
	$(SIZE) $(PRJPATH)/firmware.elf
	-@echo ""
	-@echo "*** Compile complete! ***"
clean:
	rm -f $(OBJS) $(PRJPATH)/firmware.elf firmware.bin firmware.hex
