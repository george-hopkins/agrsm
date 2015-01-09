# Makefile for Agere Soft Modem Driver for Linux
#
# Copyright (c) 2002, 2003 Agere Systems, Inc.  All rights reserved.
#
# Description:
# Makes the following modules
#   - Agere Soft Modem Controller driver module 
#   - Modem Serail Interface driver module
#
# Usage:
#   make                 - build the module(s)
#   make modules_install - install the module(s)
#   make clean           - remove generated files in module directory only
#
# Revision History:
#   Name                   Date          Change
#   Soumyendu Sarkar       12/03/2002    Initial
#

PWD := $(shell pwd)
KRELEASE = $(shell uname -r)
KERNEL_DIR := /lib/modules/$(KRELEASE)/build

default:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
