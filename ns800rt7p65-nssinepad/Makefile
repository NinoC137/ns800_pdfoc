PROJECT := rtthread
BUILD_DIR := .
JLINK_EXE ?= JLinkExe
JLINK_DEVICE ?= NS800RT7P65
JLINK_IF ?= SWD
JLINK_SPEED ?= 4000
JLINK_SCRIPT := flash.jlink

VENV := /Volumes/1TBWDBlue/Embedded_Dev/.venv

SCONS ?= $(shell [ -x $(VENV)/bin/scons ] && printf "$(VENV)/bin/scons" || printf "scons")
PYTHON ?= $(shell [ -x $(VENV)/bin/python3 ] && printf "$(VENV)/bin/python3" || printf "python3")

NSWAVEVIEW := ../tools/waveview/nswaveview.py
PORT ?= /dev/cu.usbmodem*
BAUD ?= 115200
CHANNELS ?= 3

wave:
	$(PYTHON) $(NSWAVEVIEW) \
		--port $(PORT) \
		--baud $(BAUD) \
		--channels $(CHANNELS)

.PHONY: flash flash-jlink

all:
	$(SCONS) -j4

flash: flash-jlink

flash-jlink: $(PROJECT).elf
	# J-Link 烧录：使用 J-Link Commander 下载 ELF 到目标板并复位运行。
	@printf "ExitOnError 1\nr\nhalt\nloadfile %s\nr\ng\nqc\n" "$<" > $(JLINK_SCRIPT)
	$(JLINK_EXE) -device $(JLINK_DEVICE) -if $(JLINK_IF) -speed $(JLINK_SPEED) -CommanderScript $(JLINK_SCRIPT)