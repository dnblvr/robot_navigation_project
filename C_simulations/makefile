# Compiler and flags
CC = clang
CFLAGS = -Iinc -Wall -Wextra -Wpedantic -lm

# Directories
SRC_DIR = src
INC_DIR = inc
BUILD_DIR = build

# Source and object files
SRCS = main.c $(SRC_DIR)/coordinate_transform.c $(SRC_DIR)/rrt_star.c $(SRC_DIR)/ransac.c $(SRC_DIR)/matrices.c
OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o)

# Output executable
TARGET = main

# Default target
all: $(TARGET)

# Build target
$(TARGET): $(OBJS)
    $(CC) $(OBJS) -o $@ $(CFLAGS)

# Compile source files into object files
$(BUILD_DIR)/%.o: %.c
    @mkdir -p $(dir $@)
    $(CC) -c $< -o $@ $(CFLAGS)

# Clean build files
clean:
    rm -rf $(BUILD_DIR) $(TARGET)

# Phony targets
.PHONY: all clean