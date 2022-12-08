# Edit this file to compile extra C files into their own programs.
SOURCES = main.c common.c LCD_display.c  ultrasonic.c BLE_controller.c watchdog.c
TARGET = droneController

SHARED_DIR = $(HOME)/cmpt433/public/myApps
PUBDIR = .
OUTDIR = $(PUBDIR)
CROSS_TOOL = arm-linux-gnueabihf-
CC_CPP = $(CROSS_TOOL)g++
CC_C = $(CROSS_TOOL)gcc

CFLAGS = -Wall -g -std=c99 -D _POSIX_C_SOURCE=200809L -Werror -Wshadow
# -pg for supporting gprof profiling.
# CFLAGS += -pg

all: $(SOURCES)
	$(CC_C) $(CFLAGS) $(SOURCES) -o $(OUTDIR)/$(TARGET) -lpthread
	cp $(OUTDIR)/$(TARGET) $(SHARED_DIR)

clean:
	rm -f $(OUTDIR)/$(TARGET)
	rm -f $(SHARED_DIR)/$(TARGET)