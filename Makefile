CC = gcc
CFLAGS = -Wall -Wextra -O2

BIN_DIR = bin

SRC = parser.c
OBJ = $(BIN_DIR)/parser.o
TARGET = $(BIN_DIR)/parser

all: $(TARGET)

$(TARGET): $(OBJ) | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^

$(BIN_DIR)/parser.o: parser.c parser.h | $(BIN_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

clean:
	rm -rf $(BIN_DIR)

