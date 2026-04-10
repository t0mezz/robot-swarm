# tools/Makefile

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2

UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    HUB_LDFLAGS        = -framework IOKit
    CONTROLLER_LDFLAGS = -framework CoreGraphics -framework CoreFoundation
endif

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4 2>/dev/null || pkg-config --libs   opencv 2>/dev/null)

.PHONY: all clean calibrate

all: swarm_hub swarm_terminal latency_plot swarm_controller vision_controller circle_demo wingman calibrate

calibrate:
	$(MAKE) -C vision/calibration

swarm_hub: swarm/swarm_hub.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(HUB_LDFLAGS)

swarm_terminal: swarm/swarm_terminal.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

latency_plot: swarm/latency_plot.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

swarm_controller: swarm/swarm_controller.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(CONTROLLER_LDFLAGS)

vision_controller: vision/vision_controller.cpp vision/aruco_tracker.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -Ivision $< -o $@ $(OPENCV_LIBS) $(CONTROLLER_LDFLAGS)

circle_demo: vision/circle_demo.cpp vision/april/april_vision.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -Ivision $< -o $@ $(OPENCV_LIBS) $(CONTROLLER_LDFLAGS)

wingman: vision/wingman.cpp vision/aruco_tracker.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -Ivision $< -o $@ $(OPENCV_LIBS) $(CONTROLLER_LDFLAGS)

clean:
	rm -f swarm_hub swarm_terminal latency_plot swarm_controller vision_controller circle_demo wingman
	$(MAKE) -C vision/calibration clean
