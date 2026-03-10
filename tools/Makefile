# tools/Makefile
# Baut die PC-seitigen Tools

CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2

# macOS braucht IOKit für Serial
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    LDFLAGS += -framework IOKit
endif

.PHONY: all clean

all: swarm_terminal

swarm_terminal: swarm_terminal.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

clean:
	rm -f swarm_terminal
