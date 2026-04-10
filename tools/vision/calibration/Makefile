# tools/calibration/Makefile

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4 2>/dev/null || pkg-config --libs   opencv 2>/dev/null)

UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    EXTRA_LDFLAGS = -framework CoreGraphics -framework CoreFoundation
endif

HEADERS = cmaes.h param_space.h objective.h objective_static.h ../aruco_tracker.h

.PHONY: all clean

all: calibrate

calibrate: calib_main.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $< -o $@ $(OPENCV_LIBS) $(EXTRA_LDFLAGS)

clean:
	rm -f calibrate
