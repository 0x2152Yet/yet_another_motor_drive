################################################################################
#
#  Yet Another Motor Drive
#
#  MIT License
#
#  Copyright (c) 2021 Michael F. Kaufman
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
#
#################################################################################
#  
#  Makefile for the Motor Drive application.
#
#  At present, the following are accepted:
#     make:  Builds the image.
#     make clean: Clears out all of the generated files including build output,
#                 object files, and dependency files
#     make noopt:  Builds the image with no compiler optimization to simplify
#                  debugging
#
#################################################################################

#
#  These are definitions that will be passed via the compiler and linker.
#
MAKE_DEFINES := USE_FULL_LL_DRIVER

#
#  "Phony" commands are make "commands" that do not actually create a file.
#
.PHONY: all noopt clean

OUTPUT_NAME := motor_drive
BUILD_DIR := ./build
SRC_DIR := ./src
EXE_DIR := $(BUILD_DIR)/motor_drive_image
OBJS_DIR := $(BUILD_DIR)/obj
DEPEND_DIR := $(BUILD_DIR)/depend
BUILDCOUNT_DIR := $(SRC_DIR)/main
BUILD_TOOLS_DIR := ./tools

#
#  Basic build targets
#
default: $(EXE_DIR)/$(OUTPUT_NAME).bin
all: $(EXE_DIR)/$(OUTPUT_NAME).bin
noopt: $(EXE_DIR)/$(OUTPUT_NAME).bin

#
#  The VPATH includes all the directories that store our source code.  Make
#  uses this to know where to look for files.
#
VPATH = \
	. \
	$(SRC_DIR) \
	$(SRC_DIR)/CMSIS \
	$(SRC_DIR)/free_rtos_source \
	$(SRC_DIR)/free_rtos_source/include \
	$(SRC_DIR)/free_rtos_source/portable \
	$(SRC_DIR)/free_rtos_source/portable/GCC \
	$(SRC_DIR)/free_rtos_source/portable/GCC/ARM_CM4F \
	$(SRC_DIR)/hw_io \
	$(SRC_DIR)/main \
	$(SRC_DIR)/motor_drive \
	$(SRC_DIR)/processor_specific \
	$(SRC_DIR)/processor_specific/STM32F4xx_HAL_Driver \
	$(SRC_DIR)/util

#
#  These are the source files.  We build our C++, C, and assembly
#  source files with different tools and options so we split them into
#  three different groups.
#
CPP_SRC_FILES := \
adc_conversions.cpp \
adc_interface.cpp \
background_task.cpp \
c_startup.cpp \
dac_interface.cpp \
data_logger.cpp \
debouncer.cpp \
gpio_interface.cpp \
i2c_interface.cpp \
low_pass_filter.cpp \
main.cpp \
motor_angle_n_speed.cpp \
motor_controller.cpp \
motor_transforms.cpp \
motor_state_machine.cpp \
newlib_functions.cpp \
os_interface.cpp \
physical_inputs.cpp \
pi_controller.cpp \
pll_angle_est.cpp \
pwm_interface.cpp \
smo_angle_est.cpp \
tick_timers.cpp \
uart_interface.cpp \
zero_sequence_modulator.cpp

C_SRC_FILES := \
arm_common_tables.c \
arm_cos_f32.c \
arm_sin_f32.c \
croutine.c \
event_groups.c \
heap_1.c \
list.c \
port.c \
queue.c \
stm32f4xx_ll_adc.c \
stm32f4xx_ll_dac.c \
stm32f4xx_ll_dma.c \
stm32f4xx_ll_exti.c \
stm32f4xx_ll_rcc.c \
stm32f4xx_ll_gpio.c \
stm32f4xx_ll_i2c.c \
stm32f4xx_ll_tim.c \
stm32f4xx_ll_usart.c \
stream_buffer.c \
system_stm32f4xx.c \
tasks.c \
timers.c

ASM_SRC_FILES := \
startup_stm32f429xx.s

#
# These are the include paths for compiling and linking.
#
define MAKE_INCLUDES
-I . \
-I $(SRC_DIR) \
-I $(SRC_DIR)/CMSIS \
-I $(SRC_DIR)/free_rtos_source \
-I $(SRC_DIR)/free_rtos_source/include \
-I $(SRC_DIR)/free_rtos_source/portable \
-I $(SRC_DIR)/free_rtos_source/portable/GCC \
-I $(SRC_DIR)/free_rtos_source/portable/GCC/ARM_CM4F \
-I $(SRC_DIR)/hw_io \
-I $(SRC_DIR)/main \
-I $(SRC_DIR)/motor_drive \
-I $(SRC_DIR)/processor_specific \
-I $(SRC_DIR)/processor_specific/STM32F4xx_HAL_Driver \
-I $(SRC_DIR)/util
endef

#
# These are where the various tools and support files live.
#
GCC_DIR := /Volumes/Macintosh\ HD/Applications/ARM
PERL := /usr/bin/perl

#
#  This defines the linker command file that describes the memory layout.
#
LINKER_COMMAND_FILE := $(BUILD_DIR)/linker_cmd.ld

#
#  We select compiler options and output names based on the build
#  type (flash vs. noopt)
#
ifeq ($(MAKECMDGOALS),)
OPT_FLAGS := O1
endif

ifeq ($(MAKECMDGOALS),default)
OPT_FLAGS := O1
endif

ifeq ($(MAKECMDGOALS),all)
OPT_FLAGS := O1
endif

ifeq ($(MAKECMDGOALS),noopt)
OPT_FLAGS := O0
endif

#
#  These take the lists of source files and create lists of build output files
#  based on the source file names.  For example, depending on the directory
#  symbol names, "my_file.cpp" would be used to create "./build/obj/my_file.obj"
#  and "./build/depend/my_file.d".
#
CPP_OBJS  = $(patsubst %.cpp, $(OBJS_DIR)/%.obj, $(CPP_SRC_FILES))
C_OBJS = $(patsubst %.c, $(OBJS_DIR)/%.obj, $(C_SRC_FILES))
ASM_OBJS = $(patsubst %.s, $(OBJS_DIR)/%.obj, $(ASM_SRC_FILES))

CPP_DEPENDS = $(patsubst %.cpp, $(DEPEND_DIR)/%.d, $(CPP_SRC_FILES))
C_DEPENDS = $(patsubst %.c, $(DEPEND_DIR)/%.d, $(C_SRC_FILES))
ASM_DEPENDS = $(patsubst %.s, $(DEPEND_DIR)/%.d, $(ASM_SRC_FILES))

#
# Create the output directories
#
$(EXE_DIR):
	-@echo Create directory $@
	-@mkdir -p $@

$(OBJS_DIR):
	-@echo Create directory $@
	-@mkdir -p $@

$(DEPEND_DIR):
	-@echo Create directory $@
	-@mkdir -p $@

#
# Clean out everything
#
clean:
	@rm -rf $(OBJS_DIR)
	@rm -rf $(EXE_DIR)
	@rm -rf $(DEPEND_DIR)
	-@echo 'Finished cleaning $(OUTPUT_NAME)'

#
#  These are all of the things we link together to create the final image.
#
LINK_OBJS := \
$(ASM_OBJS) \
$(CPP_OBJS) \
$(C_OBJS) \
$(OBJS_DIR)/buildcnt.obj

#
# We need two versions of the C++ compiler command.  Once that is used for
# the occasional case where we give it a file explicitly ("BASE" command) and
# one that is used in the general build rules.
#
define BASE_CPP_COMPILE_COMMAND
$(GCC_DIR)/bin/arm-none-eabi-g++ -c -D$(MAKE_DEFINES) \
-Werror -Wall \
-mthumb -mcpu=cortex-m4 -DARCH=armv7e-m -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -$(OPT_FLAGS) -g3 \
--specs=nano.specs \
-fno-operator-names  -fno-use-cxa-atexit -fno-exceptions \
$(MAKE_INCLUDES)
endef

define CPP_COMPILE_COMMAND
$(BASE_CPP_COMPILE_COMMAND) \
-MT $@ -MMD -MP -MF $(DEPEND_DIR)/$*.d \
$< -o $@
endef

#
#  C files are compiled somewhat differently than C++ files.
#
define C_COMPILE_COMMAND
$(GCC_DIR)/bin/arm-none-eabi-gcc -c -D$(MAKE_DEFINES) \
-Werror -Wall \
-mthumb -mcpu=cortex-m4 -DARCH=armv7e-m -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -$(OPT_FLAGS) -g3  \
--specs=nano.specs \
-MT $@ -MMD -MP -MF $(DEPEND_DIR)/$*.d \
$(MAKE_INCLUDES) \
$< -o $@
endef


define BASE_LINK_COMMAND
$(GCC_DIR)/bin/arm-none-eabi-g++ $(LINK_OBJS) -L$(OBJS_DIR) \
-mcpu=cortex-m3 -mthumb -mfloat-abi=soft -mfpu=auto -$(OPT_FLAGS) -g3 \
-u _printf_float -fno-use-cxa-atexit \
-Xlinker --gc-sections -specs=nano.specs \
-Xlinker -Map=$(EXE_DIR)/$(OUTPUT_NAME).map \
-Xlinker -rpath-link -Xlinker $(GCC_DIR) \
-L$(GCC_DIR) -Xlinker -q
endef

define LINK_COMMAND
$(BASE_LINK_COMMAND) \
-T ./$(LINKER_COMMAND_FILE) \
-o $(EXE_DIR)/$(OUTPUT_NAME).elf
endef

#
#  This is the message we print for each file we build.
#
define ECHO_FILE
@echo $<
endef

#
# Make the objects
#
$(OBJS_DIR)/%.obj: %.cpp $(DEPEND_DIR)/%.d
$(OBJS_DIR)/%.obj: %.cpp
	@$(ECHO_FILE)
	@$(CPP_COMPILE_COMMAND)

$(OBJS_DIR)/%.obj: %.c $(DEPEND_DIR)/%.d
$(OBJS_DIR)/%.obj: %.c
	@$(ECHO_FILE)
	@$(C_COMPILE_COMMAND)

$(OBJS_DIR)/%.obj: %.s $(DEPEND_DIR)/%.d
$(OBJS_DIR)/%.obj: %.s
	@$(ECHO_FILE)
	@$(C_COMPILE_COMMAND)
	@touch $(DEPEND_DIR)/$*.d

$(OBJS_DIR)/buildcnt.obj:
	@touch $(OBJS_DIR)/buildcnt.obj

#
#  This keeps the make utility from deleting the dependency files once the
#  build is complete.
#
.SECONDARY: $(CPP_DEPENDS) $(C_DEPENDS) $(ASM_DEPENDS)

#
#  This builds and links the executable.  Note that this also updates the 
#  build count.
#
$(EXE_DIR)/$(OUTPUT_NAME).bin: $(OBJS_DIR) $(EXE_DIR) $(DEPEND_DIR) $(LINK_OBJS)
	@$(PERL) $(BUILD_DIR)/buildcnt.pl
	@$(BASE_CPP_COMPILE_COMMAND) $(BUILDCOUNT_DIR)/buildcnt.cpp -o $(OBJS_DIR)/buildcnt.obj
	-@echo Linking $(LINKER_COMMAND_FILE)
	@$(LINK_COMMAND)
	@$(GCC_DIR)/bin/arm-none-eabi-objcopy -O binary $(EXE_DIR)/$(OUTPUT_NAME).elf $(EXE_DIR)/$(OUTPUT_NAME).bin
	@$(GCC_DIR)/bin/arm-none-eabi-size --format=berkeley $(EXE_DIR)/$(OUTPUT_NAME).elf

#
#  We include the dependency files.  These are created by the compiler when
#  it builds an object.  Keep this last in the makefile.
#
-include $(CPP_DEPENDS) $(C_DEPENDS) $(ASM_DEPENDS)
