# SPDX-License-Identifier: BSD-2-Clause
# 
# Copyright (c) 2022 Vincent DEFERT. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions 
# are met:
# 
# 1. Redistributions of source code must retain the above copyright 
# notice, this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright 
# notice, this list of conditions and the following disclaimer in the 
# documentation and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.

# Prerequisites --------------------------------------------------------
#
# Besides make, his project requires: 
#
# - sdcc
# - stcgal-patched
# - minicom
# - doxygen

# Usage ----------------------------------------------------------------
#
# Build executable in release mode:
#   make
#
# Build executable in debug mode:
#   make BUILD_MODE=debug
#
# Build documentation:
#   make doc
#
# Upload executable to MCU:
#   make upload
#
# Open serial console in new window:
#   make console
#
# Clean project (remove all build files):
#   make clean

# Target MCU settings --------------------------------------------------

# Note: try lowest speed for power efficiency on STC15W101 series MCU (pg. 1, sec. 1)
#MCU_FREQ := 5990000]
#MCU_FREQ := 5414000

# for STC15W104 door sensor models
MCU_FREQ := 10886000

STACK_SIZE := 16

# 
MEMORY_SIZES = \
	--iram-size 128 \
    --xram-size 0 \
	--stack-size $(STACK_SIZE) \
    --code-size 4096

#FIXME: specify eeprom address selection to a separate macro?
MEMORY_MODEL := --model-small

HAS_DUAL_DPTR := n

# Define UNISTC_DIR, HAL_DIR, DRIVER_DIR, and MAKE_DIR -----------------
include ../../makefiles/0-directories.mk

# Project settings -----------------------------------------------------
PROJECT_NAME := door-reed-rf-demo

SRCS := \
    coding.c \
    flash_dump.c \
    $(HAL_DIR)/delay.c \
    $(HAL_DIR)/gpio-hal.c \
    $(HAL_DIR)/power-hal.c \
    $(HAL_DIR)/timer-hal.c \
	main.c

CONSOLE_BAUDRATE := 57600
CONSOLE_PORT := ttyUSB0

ISP_PORT := ttyUSB0

# Boilerplate rules ----------------------------------------------------
include $(MAKE_DIR)/1-settings.mk
-include $(DEP_FILE)
include $(MAKE_DIR)/2-rules.mk
