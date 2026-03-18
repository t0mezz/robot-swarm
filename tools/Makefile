# tools/Makefile
# Baut die PC-seitigen Tools

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2

# macOS braucht IOKit nur für swarm_hub (Serial-Zugriff)
# swarm_terminal und latency_plot nutzen nur Sockets — kein IOKit nötig
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    HUB_LDFLAGS        = -framework IOKit
    CONTROLLER_LDFLAGS = -framework CoreGraphics
endif

.PHONY: all clean

all: swarm_hub swarm_terminal latency_plot swarm_controller

swarm_hub: swarm_hub.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(HUB_LDFLAGS)

swarm_terminal: swarm_terminal.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

latency_plot: latency_plot.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

swarm_controller: swarm_controller.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(CONTROLLER_LDFLAGS)

clean:
	rm -f swarm_hub swarm_terminal latency_plot swarm_controller
