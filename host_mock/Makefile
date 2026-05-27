CXX      = g++
CC       = gcc
CXXFLAGS = -DHOST_MOCK -std=c++17 -g -Wall -Wno-unused-variable
CFLAGS   = -DHOST_MOCK -g -Wall

# host_mock first so mock esp_log.h shadows the real one
INCLUDES = -I. -I../main -I../components/ui/include \
           -I../components/ft8_lib -I../components/ft8_lib/ft8

CORE_SRC   = ../main/autoseq.cpp
MOCK_SRC   = host_main.cpp host_mocks.cpp json_parser.cpp
# FT8 encode sources (uncomment to link real encoder):
# FT8_SRC  = ../components/ft8_lib/ft8/encode.c ../components/ft8_lib/ft8/message.c \
#            ../components/ft8_lib/ft8/crc.c ../components/ft8_lib/ft8/pack.c \
#            ../components/ft8_lib/ft8/text.c

ALL_CXX_SRC = $(CORE_SRC) $(MOCK_SRC)
ALL_C_SRC   = $(FT8_SRC)

CXX_OBJ = $(ALL_CXX_SRC:.cpp=.o)
C_OBJ   = $(ALL_C_SRC:.c=.o)
OBJ     = $(CXX_OBJ) $(C_OBJ)

TARGET = host_test

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean

# Phase 1 standalone synth test (no FreeRTOS, no autoseq)
SYNTH_TARGET = host_synth_test
SYNTH_SRC = test_ft8_synth.cpp
SYNTH_OBJ = $(SYNTH_SRC:.cpp=.o)
SYNTH_CXXFLAGS = $(CXXFLAGS) -I../main

$(SYNTH_TARGET): $(SYNTH_OBJ)
	$(CXX) $(SYNTH_CXXFLAGS) -o $@ $^

# Explicit rule wins over %.o: %.cpp — ensures -I../main is used for this file.
test_ft8_synth.o: test_ft8_synth.cpp
	$(CXX) $(SYNTH_CXXFLAGS) -c $< -o $@

synth: $(SYNTH_TARGET)

clean_synth:
	rm -f $(SYNTH_OBJ) $(SYNTH_TARGET)

.PHONY: synth clean_synth
