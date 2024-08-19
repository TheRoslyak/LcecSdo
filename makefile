# Compiler
CC = g++

# Compilation flags
CFLAGS = -std=c++17 -Wall -fPIC -DULAPI

# Paths to search for header files
INCLUDES = -Iinclude \
	   	   -I/usr/include/linuxcnc \
		   -I/usr/local/include

# Link flags
LDFLAGS = 

# Libraries for linking
LIBS = -lyaml-cpp -llinuxcnchal -lethercat -lrt -lyaml-cpp

# Source files
SRCS = main.cpp 

# Output file
OUTPUT = /usr/bin/lcecsdo

# Rule to compile all source files
all:
	$(CC) $(CFLAGS) $(INCLUDES) $(SRCS) $(LDFLAGS) $(LIBS) -o $(OUTPUT)
