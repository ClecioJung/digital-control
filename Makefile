# ----------------------------------------
# Compiler and linker definitions
# ----------------------------------------

# Compiler and linker
CC = gcc
AR = ar

# Flags for compiler
CFLAGS = -W -Wall -Wextra -pedantic -Werror -O2 -std=c99

# ----------------------------------------
# Compilation and linking rules
# ----------------------------------------

all: digitalControl.a

digitalControl.a: digitalControl.o
	$(AR) rcs $@ $^

digitalControl.o : digitalControl.c digitalControl.h
	$(CC) $(CFLAGS) -c $(filter %.c %.s %.o,$^) -o $@

# ----------------------------------------
# Script rules
# ----------------------------------------

clean:
	rm -rf *.d *.o *.a *.so

remade: clean all

.PHONY: all clean remade

# ----------------------------------------
