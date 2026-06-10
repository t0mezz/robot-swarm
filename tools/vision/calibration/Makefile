# tools/calibration/Makefile

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2
LIBDIR   = ../../../lib
BUILD_DIR = ../../build

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4 2>/dev/null || pkg-config --libs   opencv 2>/dev/null)

UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    EXTRA_LDFLAGS  = -framework CoreGraphics -framework CoreFoundation
    PYLON_DIR     ?= /Library/Frameworks/pylon.framework
    PYLON_CFLAGS   = -F$(dir $(PYLON_DIR)) \
                     -I$(PYLON_DIR)/Headers -I$(PYLON_DIR)/Headers/GenICam \
                     -Wno-overloaded-virtual -Wno-unused-variable
    PYLON_LIBS     = -F$(dir $(PYLON_DIR)) -framework pylon \
                     -Wl,-rpath,$(dir $(PYLON_DIR))
else
    PYLON_CFLAGS   = $(shell pylon-config --cflags 2>/dev/null) \
                     -Wno-overloaded-virtual -Wno-unused-variable
    PYLON_LIBS     = $(shell pylon-config --libs 2>/dev/null)
endif

LIBHDRS  = $(LIBDIR)/ArucoTracker/aruco_tracker.h \
           $(LIBDIR)/Calibration/cmaes.h \
           $(LIBDIR)/Calibration/param_space.h \
           $(LIBDIR)/Calibration/objective.h \
           $(LIBDIR)/Calibration/objective_static.h

.PHONY: all clean

all: $(BUILD_DIR)/calibrate

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/calibrate: calib_main.cpp $(LIBHDRS) $(LIBDIR)/ArucoTracker/basler_pylon_source.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) \
	    -I$(LIBDIR)/ArucoTracker -I$(LIBDIR)/Calibration \
	    $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(EXTRA_LDFLAGS)

clean:
	rm -f $(BUILD_DIR)/calibrate
