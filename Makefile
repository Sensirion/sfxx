#########################################################################
#
# SFxx driver
#
# This makefile works for the Raspberry PI, where SFxx sensor is wired
# to I2C bus with ID==$(I2C_BUSID)
#########################################################################


#########################################################################
# Board Configuration
#########################################################################
# on which bus to instantiate the driver
I2C_BUSID=10
# address on which the sensor is listening
ADDRESS=0x40
# ssh connection string (e.g. for Raspberry Pi)
BOARD_ADDRESS?=pi@192.168.1.10

#########################################################################
# Kernel build directory
#########################################################################
KERNELDIR?=/usr/src/raspberrypi/linux/

#########################################################################
# Tools
#########################################################################
PWD := $(shell pwd)
ADDRESS_NR := $(shell echo $(ADDRESS) | cut -d'x' -f 2)

#########################################################################
# Build setup
#########################################################################
# you should have a source called  $(MODULENAME).c in the same directory
MODULENAME=sfxx
MODULEBINARY=$(MODULENAME).ko
ARCH?=arm
CROSS_COMPILE?=/usr/src/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-

#########################################################################
# Targets
#########################################################################
obj-m := $(MODULENAME).o

$(MODULEBINARY): $(MODULENAME).c
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE="$(CROSS_COMPILE)" modules
clean:
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) clean; rm -f *.symvers


all: $(MODULEBINARY)


# this part allows automated loading,
# unloading relading of module
# this should not be probably here, but it is handy

load: $(MODULEBINARY)
	scp $(MODULEBINARY) $(BOARD_ADDRESS):
	ssh $(BOARD_ADDRESS) 'sudo insmod $(MODULEBINARY)'
	ssh $(BOARD_ADDRESS) 'echo sdp631 $(ADDRESS) | sudo tee /sys/bus/i2c/devices/i2c-$(I2C_BUSID)/new_device'
	@echo "\n driver loaded\n"

unload:
	ssh $(BOARD_ADDRESS) 'echo $(ADDRESS) | sudo tee /sys/bus/i2c/devices/i2c-$(I2C_BUSID)/delete_device' || true
	ssh $(BOARD_ADDRESS) 'sudo rmmod $(MODULENAME)' || true
	@echo "\n driver unloaded [see messages above]\n"

read:
	@echo "scale_factor: $(shell ssh  $(BOARD_ADDRESS) cat /sys/bus/i2c/devices/i2c-$(I2C_BUSID)/$(I2C_BUSID)-00$(ADDRESS_NR)/scale_factor)"
	@echo "offset: $(shell ssh  $(BOARD_ADDRESS) cat /sys/bus/i2c/devices/i2c-$(I2C_BUSID)/$(I2C_BUSID)-00$(ADDRESS_NR)/offset)"
	@echo "unit: $(shell ssh  $(BOARD_ADDRESS) cat /sys/bus/i2c/devices/i2c-$(I2C_BUSID)/$(I2C_BUSID)-00$(ADDRESS_NR)/unit)"
	@echo "measured_value: $(shell ssh  $(BOARD_ADDRESS) cat /sys/bus/i2c/devices/i2c-$(I2C_BUSID)/$(I2C_BUSID)-00$(ADDRESS_NR)/measured_value)"

reload:
	@make unload && make load && make read

dmesg:
	ssh $(BOARD_ADDRESS) dmesg | grep sht
