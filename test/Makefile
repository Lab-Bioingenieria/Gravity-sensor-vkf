CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Wno-unused-parameter

SRC = test_emg.cpp ../EMGFilters.cpp ../EMGProcessor.cpp
BIN = test_emg

.PHONY: all test clean

all: test

test: $(BIN)
	./$(BIN)

$(BIN): $(SRC) ../EMGFilters.h ../EMGProcessor.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) -lm

clean:
	rm -f $(BIN)
