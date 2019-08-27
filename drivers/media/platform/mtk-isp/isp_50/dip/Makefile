# SPDX-License-Identifier: GPL-2.0
#
# Copyright (C) 2019 MediaTek Inc.
#

$(info $(srctree))
ccflags-y += -I$(srctree)/drivers/media/platform/mtk-mdp3

obj-$(CONFIG_VIDEO_MEDIATEK_ISP_DIP) += mtk_dip-v4l2.o

# Utilities to provide frame-based streaming model
# with v4l2 user interfaces and alloc context managing
# memory shared between ISP and coprocessor
mtk_dip_util-objs := \
mtk_dip-dev.o \
mtk_dip-sys.o

obj-$(CONFIG_VIDEO_MEDIATEK_ISP_DIP) += mtk_dip_util.o
