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
# for STC15W101 mcu used in door sensor
#MCU_PREQ := 11057000
# for STC15W104 mcu
MCU_FREQ := 10598000
STACK_SIZE := 16

# limit code size to 1 KB to support '101 part
# take care to account for 7 bytes of unique id
# so for example 1024 byte part minus 7 bytes is limited to 1017 flash
MEMORY_SIZES = \
	--iram-size 128 \
    --xram-size 0 \
	--stack-size $(STACK_SIZE) \
    --code-size 1017

#
MEMORY_MODEL := --model-small

HAS_DUAL_DPTR := n

# Define UNISTC_DIR, HAL_DIR, DRIVER_DIR, and MAKE_DIR -----------------
include ../../makefiles/0-directories.mk

# Project settings -----------------------------------------------------
PROJECT_NAME := ReedTripRadio

SRCS := \
    $(HAL_DIR)/delay.c \
	main.c

CONSOLE_BAUDRATE := 19200
CONSOLE_PORT := ttyUSB0

ISP_PORT := COM3

# Boilerplate rules ----------------------------------------------------
include $(MAKE_DIR)/1-mcu-settings.mk
-include $(DEP_FILE)
include $(MAKE_DIR)/2-mcu-rules.mk
