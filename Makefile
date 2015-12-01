PORT?=/dev/ttyUSB0
PROJECT=ubberButton
ARDUINO_MODEL=nano
MCU=atmega328p
UPLOAD_RATE=57600
#ARDUINO_DIR=/usr/share/arduino/
ARDUINO_CORE=$(ARDUINO_DIR)/hardware/arduino/cores/arduino

USER_LIBDIR=/home/glager/sketchbook/libraries/
USER_LIBS=RadioHead uberbuttonlib
ARDUINO_LIBS=Wire Arduino LiquidCrystal SPI
#OPT_WARN=-Wall -Wno-missing-declarationsLiquidCrystal
ARDUINO=106
include Makefile.arduino

