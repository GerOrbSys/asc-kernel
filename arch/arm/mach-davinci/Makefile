#
# Makefile for the linux kernel.
#
#

# Common objects
obj-y 			:= time.o clock.o serial.o io.o psc.o \
			   gpio.o dma.o usb.o common.o sram.o aemif.o

obj-$(CONFIG_DAVINCI_MUX)		+= mux.o
obj-$(CONFIG_PCI)			+= pci-generic.o

# Chip specific
obj-$(CONFIG_ARCH_DAVINCI_DM644x)       += dm644x.o devices.o
obj-$(CONFIG_ARCH_DAVINCI_DM355)        += dm355.o devices.o
obj-$(CONFIG_ARCH_DAVINCI_DM646x)       += dm646x.o devices.o
obj-$(CONFIG_ARCH_DAVINCI_DM365)	+= dm365.o devices.o
obj-$(CONFIG_ARCH_DAVINCI_DA830)        += da830.o devices-da8xx.o
obj-$(CONFIG_ARCH_DAVINCI_DA850)        += da850.o devices-da8xx.o

# PCI - Chip specific
ifeq ($(CONFIG_PCI), y)
obj-$(CONFIG_ARCH_DAVINCI_DM646x)	+= pci-dm646x.o
endif

obj-$(CONFIG_AINTC)			+= irq.o
obj-$(CONFIG_CP_INTC)			+= cp_intc.o

# Board specific
obj-$(CONFIG_MACH_DAVINCI_EVM)  	+= board-dm644x-evm.o
obj-$(CONFIG_MACH_SFFSDR)		+= board-sffsdr.o
obj-$(CONFIG_MACH_DAVINCI_DM355_EVM)	+= board-dm355-evm.o
obj-$(CONFIG_MACH_DM355_LEOPARD)	+= board-dm355-leopard.o
obj-$(CONFIG_MACH_DAVINCI_DM368_ASC)    += board-dm368-asc.o
obj-$(CONFIG_MACH_DAVINCI_DM6467_EVM)	+= board-dm646x-evm.o cdce949.o
obj-$(CONFIG_MACH_DAVINCI_DM365_EVM)	+= board-dm365-evm.o
obj-$(CONFIG_MACH_DM368_LEOPARD)	+= board-dm368-leopard.o
obj-$(CONFIG_MACH_DAVINCI_DA830_EVM)	+= board-da830-evm.o
obj-$(CONFIG_MACH_DAVINCI_DA850_EVM)	+= board-da850-evm.o
