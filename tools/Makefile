# tools/Makefile

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2
LIBDIR   = ../lib

UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    HUB_LDFLAGS        = -framework IOKit
    CONTROLLER_LDFLAGS = -framework CoreGraphics -framework CoreFoundation
    PYLON_DIR         ?= /Library/Frameworks/pylon.framework
    PYLON_CFLAGS       = -F$(dir $(PYLON_DIR)) \
                         -I$(PYLON_DIR)/Headers -I$(PYLON_DIR)/Headers/GenICam \
                         -Wno-overloaded-virtual -Wno-unused-variable
    PYLON_LIBS         = -F$(dir $(PYLON_DIR)) -framework pylon \
                         -Wl,-rpath,$(dir $(PYLON_DIR))
    SFML_CFLAGS       := -I/opt/homebrew/opt/sfml@2/include
    SFML_LIBS         := -L/opt/homebrew/opt/sfml@2/lib -lsfml-graphics -lsfml-window -lsfml-system
else
    # Linux: pylon installs to /opt/pylon and ships a pylon-config query tool
    PYLON_CFLAGS       = $(shell pylon-config --cflags 2>/dev/null) \
                         -Wno-overloaded-virtual -Wno-unused-variable
    PYLON_LIBS         = $(shell pylon-config --libs 2>/dev/null)
    SFML_CFLAGS       := $(shell pkg-config --cflags sfml-all 2>/dev/null)
    SFML_LIBS         := $(shell pkg-config --libs   sfml-all 2>/dev/null)
endif

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4 2>/dev/null || pkg-config --libs   opencv 2>/dev/null)

.PHONY: all clean calibrate

all: swarm_hub swarm_terminal latency_plot swarm_controller game vision_controller circle_demo wingman shape_demo marker_eval calibrate

calibrate:
	$(MAKE) -C vision/calibration

swarm_hub: $(LIBDIR)/swarm/swarm_hub.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(HUB_LDFLAGS)

swarm_terminal: $(LIBDIR)/swarm/swarm_terminal.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

latency_plot: $(LIBDIR)/swarm/latency_plot.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

swarm_controller: $(LIBDIR)/swarm/swarm_controller.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(CONTROLLER_LDFLAGS)

game: game.cpp
	$(CXX) $(CXXFLAGS) $(SFML_CFLAGS) $< -o $@ $(SFML_LIBS)

vision_controller: vision/vision_controller.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) -I$(LIBDIR)/ArucoTracker $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(CONTROLLER_LDFLAGS)

circle_demo: vision/circle_demo.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) -I$(LIBDIR)/ArucoTracker $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(CONTROLLER_LDFLAGS)

wingman: vision/wingman.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) -I$(LIBDIR)/ArucoTracker $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(CONTROLLER_LDFLAGS)

shape_demo: vision/shape_demo.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h $(LIBDIR)/SwarmClient/SwarmClient.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) \
	    -I$(LIBDIR)/ArucoTracker -I$(LIBDIR)/SwarmClient \
	    $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(CONTROLLER_LDFLAGS)

marker_eval: vision/marker_eval.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) -I$(LIBDIR)/ArucoTracker \
	    $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS)

clean:
	rm -f swarm_hub swarm_terminal latency_plot swarm_controller game vision_controller circle_demo wingman shape_demo
	$(MAKE) -C vision/calibration clean
