############################################################################
# apps/examples/goldorak_go/Makefile
#
#   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

# GOLDORAK_GO Example.

ASRCS =
CSRCS = goldo_asserv.c\
        goldo_asserv_hal.c\
        goldo_asserv_arch.c\
        goldo_pid_filter.c\
        goldo_odometry.c\
        goldo_odometry_hal.c\
        goldo_command_fifo.c\
        goldo_polyline.c\
        goldo_match_timer.c\
        goldo_main_loop.c\
        goldo_strategy.c\
        goldo_fpga.c\
        robot/robot.c\
        robot/arms.c\
        robot/adversary_detection.c \
        dynamixel.c
        
MAINSRC = goldo_go_main.c

CONFIG_XYZ_PROGNAME ?= goldorak_go$(EXEEXT)
PROGNAME = $(CONFIG_XYZ_PROGNAME)

# GOLDORAK_GO built-in application info

APPNAME = goldorak_go
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = 2048

include $(APPDIR)/Application.mk
