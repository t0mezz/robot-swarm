# tools/Makefile
# Baut die PC-seitigen Tools

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2

UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    HUB_LDFLAGS        = -framework IOKit
    CONTROLLER_LDFLAGS = -framework CoreGraphics
endif

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4 2>/dev/null || pkg-config --libs   opencv 2>/dev/null)

.PHONY: all clean

all: swarm_hub swarm_terminal latency_plot swarm_controller vision_controller circle_demo

swarm_hub: swarm_hub.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(HUB_LDFLAGS)

swarm_terminal: swarm_terminal.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

latency_plot: latency_plot.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

swarm_controller: swarm_controller.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(CONTROLLER_LDFLAGS)

vision_controller: vision_controller.cpp aruco_tracker.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $< -o $@ $(OPENCV_LIBS) $(CONTROLLER_LDFLAGS)

circle_demo: circle_demo.cpp aruco_tracker.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $< -o $@ $(OPENCV_LIBS) $(CONTROLLER_LDFLAGS)

clean:
	rm -f swarm_hub swarm_terminal latency_plot swarm_controller vision_controller circle_demo
