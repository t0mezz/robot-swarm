# tests/Makefile
# Minimal unit tests for pure functions (no framework — plain asserts).

CXX       = g++
CXXFLAGS  = -std=c++17 -Wall -O2
BUILD_DIR = build

.PHONY: all test clean

all: $(BUILD_DIR)/test_protocol

test: $(BUILD_DIR)/test_protocol
	./$(BUILD_DIR)/test_protocol

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/test_protocol: test_protocol.cpp ../lib/SwarmProtocol/protocol.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -rf $(BUILD_DIR)
