CC := cc
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -O2
LDFLAGS := -lm
TARGET := drone_collision
SRC := main.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
