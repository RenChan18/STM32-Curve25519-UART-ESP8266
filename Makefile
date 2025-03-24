BINARY       = $(BUILD_DIR)/main
SRC_DIR      = src
BUILD_DIR    = build
INC_DIR      = include
SRCFILES     := $(wildcard $(SRC_DIR)/*.c)

PREFIX       ?= arm-none-eabi
TOP_DIR      := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
OPENCM3_DIR  := ./libopencm3
LIBNAME      = opencm3_stm32f4
DEFS        += -DSTM32F4

FP_FLAGS    ?= -mfpu=fpv4-sp-d16 -mfloat-abi=hard
ARCH_FLAGS  = -mthumb -mcpu=cortex-m4 $(FP_FLAGS) -mfix-cortex-m3-ldrd
ASFLAGS     = -mthumb -mcpu=cortex-m4

CC          := $(PREFIX)-gcc
LD          := $(PREFIX)-gcc
OBJCOPY     := $(PREFIX)-objcopy
SIZE        := $(PREFIX)-size
OBJDUMP     := $(PREFIX)-objdump
STFLASH     = $(shell which st-flash)

OPT         := -Os -g
CSTD        ?= -std=c99
DEFS       += -DBEGIN_DECLS= -DEND_DECLS=
TGT_CPPFLAGS += -include stdbool.h

OBJS         := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCFILES))
DEPS         := $(OBJS:.o=.d)
LDSCRIPT    ?= $(TOP_DIR)/linker.ld

TGT_CFLAGS   += -I$(INC_DIR)
TGT_CFLAGS   += $(OPT) $(CSTD)
TGT_CFLAGS   += $(ARCH_FLAGS)
TGT_CFLAGS   += -Wextra -Wshadow -Wimplicit-function-declaration
TGT_CFLAGS   += -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
TGT_CFLAGS   += -fno-common -ffunction-sections -fdata-sections
TGT_CFLAGS   += -I$(OPENCM3_DIR)/include

TGT_CPPFLAGS += -MD -Wall -Wundef $(DEFS)
TGT_CPPFLAGS += -I$(OPENCM3_DIR)/include -I$(INC_DIR)

TGT_LDFLAGS   = --static -nostartfiles -T$(LDSCRIPT) $(ARCH_FLAGS) \
                -Wl,-Map=$(BINARY).map -Wl,--gc-sections

LDLIBS      += -specs=nosys.specs
LDLIBS      += -L$(OPENCM3_DIR)/lib -lopencm3_stm32f4

.PHONY: all clean flash bigflash libopencm3

all: libopencm3 $(BINARY).elf

libopencm3:
	$(MAKE) -C $(OPENCM3_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(TGT_CFLAGS) $(TGT_CPPFLAGS) -c $< -o $@

$(BINARY).elf: $(OBJS) $(LDSCRIPT)
	$(LD) $(TGT_LDFLAGS) $(OBJS) $(LDLIBS) -o $@
	$(SIZE) $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.hex: %.elf
	$(OBJCOPY) -Oihex $< $@

%.srec: %.elf
	$(OBJCOPY) -Osrec $< $@

%.list: %.elf
	$(OBJDUMP) -S $< > $@

flash: $(BINARY).bin
	$(STFLASH) write $(BINARY).bin 0x8000000

bigflash: $(BINARY).bin
	$(STFLASH) --flash=512k write $(BINARY).bin 0x8000000

clean:
	rm -rf $(BUILD_DIR) $(BINARY).elf $(BINARY).bin $(BINARY).hex $(BINARY).srec $(BINARY).list
	$(MAKE) -C $(OPENCM3_DIR) clean

-include $(DEPS)

