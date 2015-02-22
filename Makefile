# Output directors to store intermediate compiled files
# relative to the project directory
BUILD_BASE	= build
FW_BASE		= firmware
PATH:=$(PATH):/home/src/esp8266/esp-open-sdk/xtensa-lx106-elf/bin

# base directory of the ESP8266 SDK package, absolute
SDK_BASE	= /home/src/esp8266/esp_iot_rtos_sdk

# name for the target project
TARGET		= espfree-bandwidth

# which modules (subdirectories) of the project to include in compiling
MODULES		= driver user
EXTRA_INCDIR	=

# libraries used in this project, mainly provided by the SDK
#LIBS		= c gcc hal phy net80211 lwip wpa main json ssl upgrade upgrade_ssl
LIBS = gcc hal phy pp net80211 wpa main freertos lwip udhcp 

# compiler flags using during compilation of source files
CFLAGS		= -Os -g -Wpointer-arith -Wundef -Werror -Wl,-EL -fno-inline-functions -nostdlib \
						-mlongcalls -mtext-section-literals  -D__ets__ -DICACHE_FLASH

# linker flags used to generate the main object file
#LDFLAGS		= -nostdlib -Wl,--no-check-sections -u call_user_start -Wl,-static
LDFLAGS		= -nostdlib -Wl,-EL -Wl,--no-check-sections -u call_user_start -Wl,-static -g -O2

# linker script used for the above linkier step
LD_SCRIPT	= eagle.app.v6.ld

# various paths from the SDK used in this project
SDK_LIBDIR	= lib
SDK_LDDIR	= ld
SDK_INCDIR	= include include/json include/espressif include/lwip \
  include/lwip/arch include/lwip/ipv4 include/lwip/ipv6 include/lwip/lwip include/lwip/netif \
	include/lwip/posix \
  extra_include extra_include/xtensa

# path to the esptool used to generate the final binaries
# it assumed to have it somwhere in the main SDK directoy tree
FW_TOOL		= esptool

# actual name of the esptool
FW_TOOLDIR	= esptool

# we create two different files for uploading into the flash
# these are the names and options to generate them
FW_FILE_1	= 0x00000
FW_FILE_1_ARGS	= -bo $@ -bs .text -bs .data -bs .rodata -bc -ec
FW_FILE_2	= 0x40000
FW_FILE_2_ARGS	= -es .irom0.text $@ -ec

# select which tools to use as compiler, librarian and linker
AR = xtensa-lx106-elf-ar
CC = xtensa-lx106-elf-gcc
NM = xtensa-lx106-elf-nm
CPP = xtensa-lx106-elf-cpp
OBJCOPY = xtensa-lx106-elf-objcopy
ESPTOOL = /home/src/esp8266/esptool/esptool.py



####
#### no user configurable options below here
####
FW_TOOL		:= $(addprefix $(SDK_BASE)/,$(addprefix $(FW_TOOLDIR)/,$(FW_TOOL)))
SRC_DIR		:= $(MODULES)
BUILD_DIR	:= $(addprefix $(BUILD_BASE)/,$(MODULES))

SDK_LIBDIR	:= $(addprefix $(SDK_BASE)/,$(SDK_LIBDIR))
SDK_INCDIR	:= $(addprefix -I$(SDK_BASE)/,$(SDK_INCDIR))

SRC		:= $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.c))
OBJ		:= $(patsubst %.c,$(BUILD_BASE)/%.o,$(SRC))
LIBS		:= $(addprefix -l,$(LIBS))
APP_AR		:= $(addprefix $(BUILD_BASE)/,$(TARGET)_app.a)
TARGET_OUT	:= $(addprefix $(BUILD_BASE)/,$(TARGET).out)

LD_SCRIPT	:= $(addprefix -T$(SDK_BASE)/$(SDK_LDDIR)/,$(LD_SCRIPT))

INCDIR	:= $(addprefix -I,$(SRC_DIR))
EXTRA_INCDIR	:= $(addprefix -I,$(EXTRA_INCDIR))
MODULE_INCDIR	:= $(addsuffix /include,$(INCDIR))

FW_FILE_1	:= $(addprefix $(FW_BASE)/,$(FW_FILE_1).bin)
FW_FILE_2	:= $(addprefix $(FW_BASE)/,$(FW_FILE_2).bin)

vpath %.c $(SRC_DIR)

define compile-objects
$1/%.o: %.c
	$(CC) $(INCDIR) $(MODULE_INCDIR) $(EXTRA_INCDIR) $(SDK_INCDIR) $(CFLAGS)  -c $$< -o $$@
endef

.PHONY: all checkdirs clean

all: checkdirs $(TARGET_OUT) $(FW_FILE_1) $(FW_FILE_2)

$(FW_FILE_1) $(FW_FILE_2): $(TARGET_OUT)
	$(ESPTOOL) elf2image $< -o $(FW_BASE)/

#$(FW_FILE_2): $(TARGET_OUT)
#	$(FW_TOOL) -eo $(TARGET_OUT) $(FW_FILE_2_ARGS)

$(TARGET_OUT): $(APP_AR)
	$(CC) -L$(SDK_LIBDIR) $(LD_SCRIPT) $(LDFLAGS) -Wl,--start-group $(LIBS) $(APP_AR) -Wl,--end-group -o $@

$(APP_AR): $(OBJ)
	$(AR) cru $@ $^

checkdirs: $(BUILD_DIR) $(FW_BASE)

$(BUILD_DIR):
	@mkdir -p $@

firmware:
	@mkdir -p $@

clean:
	@rm -f $(APP_AR)
	@rm -f $(TARGET_OUT)
	@rm -rf $(BUILD_DIR)
	@rm -rf $(BUILD_BASE)

	
	@rm -f $(FW_FILE_1)
	@rm -f $(FW_FILE_2)
	@rm -rf $(FW_BASE)

$(foreach bdir,$(BUILD_DIR),$(eval $(call compile-objects,$(bdir))))
