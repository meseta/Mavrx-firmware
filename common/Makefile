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
PRJPATH = .

LD_SCRIPT = $(PRJPATH)/../common/linker.ld
INCLUDE = -I $(PRJPATH)
INCLUDE += -I $(PRJPATH)/inc
INCLUDE += -I $(PRJPATH)/../common

### Compile ALL *.c files found in the directories (don't leave non-project
### *.c files lying around, or this makefile might attempt to include them
### in your project!

SRCS1=$(wildcard *.c)
SRCS2=$(wildcard $(PRJPATH)/../common/*.c)
DIRS=$(PRJPATH)/obj
DIRS+=$(PRJPATH)/lib
DIRS+=$(PRJPATH)/bin
OBJS=$(SRCS1:%.c=obj/%.o)
OBJS+=$(SRCS2:$(PRJPATH)/../common/%.c=lib/%.o)

#
#LIBS=$(SRCS:.c=.o)

### Compiler stuff

CFLAGS  = -c -g -O3 $(INCLUDE) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CORTEX) -DTARGET=$(MPU) -fno-builtin
ASFLAGS = -c -g -O3 $(INCLUDE) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CORTEX) -D__ASSEMBLY__ -x assembler-with-cpp
LDFLAGS = -nostartfiles -mthumb -mcpu=$(CORTEX) -Wl,--gc-sections
OCFLAGS = --strip-unneeded

all: firmware

$(DIRS):
	mkdir -p $@

$(PRJPATH)/obj/%.o : %.c
	$(CC) $(CFLAGS) -o $@ $<

$(PRJPATH)/obj/%.o : %.s
	$(AS) $(ASFLAGS) -o $@ $<
	
$(PRJPATH)/lib/%.o : $(PRJPATH)/../common/%.c
	$(CC) $(CFLAGS) -o $@ $<

$(PRJPATH)/lib/%.o : $(PRJPATH)/../common/%.s
	$(AS) $(ASFLAGS) -o $@ $<

firmware: $(DIRS) | $(OBJS)
	-@echo "*** Compiling... ***"
	$(LD) $(LDFLAGS) -T $(LD_SCRIPT) -o $(PRJPATH)/bin/firmware.elf $(OBJS) -lm
	-@echo ""
	-@echo "*** Converting... ***"
	$(OBJCOPY) $(OCFLAGS) -O binary $(PRJPATH)/bin/firmware.elf $(PRJPATH)/bin/firmware.bin
	$(OBJCOPY) $(OCFLAGS) -O ihex $(PRJPATH)/bin/firmware.elf $(PRJPATH)/bin/firmware.hex
	-@echo ""
	-@echo "*** Adding Checksum... ***"
	-$(CHKSM) $(PRJPATH)/bin/firmware.bin
	-@echo ""
	-@echo "*** Getting Size information... ***"
	$(SIZE) $(PRJPATH)/bin/firmware.elf
	-@echo ""
	-@echo "*** Compile complete! ***"
clean:
	rm -f $(PRJPATH)/obj/*.o $(PRJPATH)/lib/*.o $(PRJPATH)/bin/firmware.elf $(PRJPATH)/bin/firmware.bin $(PRJPATH)/bin/firmware.hex
