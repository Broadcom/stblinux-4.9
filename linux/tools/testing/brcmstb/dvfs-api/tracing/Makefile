# arm builds
ifeq ($(BUILD),arm)
CROSS := arm-linux-
TARGET := trace
endif

ifeq ($(BUILD),arm64)
CROSS := aarch64-linux-
TARGET := trace
endif

ifeq ($(BUILD),)
TARGET := trace_host
endif

CC := $(CROSS)gcc
CXX := $(CROSS)g++
STRIP := $(CROSS)strip
export CC CXX STRIP BUILD

CFLAGS := -Wall -Werror

C_FILES := trace.c trace_decode.c trace_decode_clk.c

.PHONY: all
all: $(TARGET)

.PHONY: clean
clean:
	rm -f $(TARGET)

$(TARGET): $(C_FILES)
	$(CC) $(CFLAGS) $(C_FILES) -o $@
