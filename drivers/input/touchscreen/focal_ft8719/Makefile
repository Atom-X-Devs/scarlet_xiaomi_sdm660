#
# Makefile for the focaltech touchscreen drivers.
#

# Each configuration option enables a list of files.
focaltech-objs := focaltech_i2c.o     \
                  focaltech_core.o    \
                  focaltech_flash.o   \
                  focaltech_ex_fun.o  \
                  focaltech_ex_mode.o \
                  focaltech_gesture.o \
                  focaltech_upgrade_ft8719.o

obj-$(CONFIG_TOUCHSCREEN_FTS_FT8719)   +=  focaltech.o
