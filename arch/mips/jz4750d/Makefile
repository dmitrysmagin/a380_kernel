#
# Makefile for the Ingenic JZ4750D.
#

# Object file lists.

# Mod: maddrone@gmail.com

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o i2c.o

obj-$(CONFIG_PROC_FS)		+= proc.o

# board specific support

obj-$(CONFIG_JZ4750D_A380)	+= board-a380.o
obj-$(CONFIG_JZ4750D_RZX50)	+= board-rzx50.o

# PM support

obj-$(CONFIG_PM_LEGACY)         += pm.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)       += cpufreq.o
