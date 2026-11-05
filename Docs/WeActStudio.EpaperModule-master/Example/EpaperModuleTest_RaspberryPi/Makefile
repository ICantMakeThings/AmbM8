# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -g -O2 -ffunction-sections -fdata-sections
LDFLAGS = -Wl,--gc-sections
LIBS = -lwiringPi -lm

# Directory settings
SRC_DIR = .
EPAPER_LIB_DIR = ./epaper
BUILD_DIR = ./build
BIN_DIR = ./bin

# Target settings
TARGET = $(BIN_DIR)/epd
EPD_TYPE = NULL  # Can be set to EPD420_BW etc.
DEBUG_FLAG = -DDEBUG

# Auto detect source files
SRCS = $(wildcard $(SRC_DIR)/*.c) $(wildcard $(EPAPER_LIB_DIR)/*.c)
OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRCS))

# Create all required directories
BUILD_SUBDIRS = $(sort $(dir $(OBJS)))
$(shell mkdir -p $(BUILD_SUBDIRS) $(BIN_DIR))

# Default target
all: $(TARGET)

# Linking target
$(TARGET): $(OBJS)
	@echo "EPD_TYPE: $(EPD_TYPE)"
	@echo "Linking to generate executable file: $@"
	$(CC) $(LDFLAGS) $^ -o $@ $(LIBS)
	@echo "Executable file generated at: $@"

# Compilation rules
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@echo "Compiling source file: $<"
	$(CC) $(CFLAGS) -D$(EPD_TYPE) $(DEBUG_FLAG) -I$(EPAPER_LIB_DIR) -c $< -o $@

$(BUILD_DIR)/%.o: $(EPAPER_LIB_DIR)/%.c
	@echo "Compiling source file: $<"
	$(CC) $(CFLAGS) -D$(EPD_TYPE) $(DEBUG_FLAG) -I$(EPAPER_LIB_DIR) -c $< -o $@

# Cleanup
clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)

.PHONY: all clean