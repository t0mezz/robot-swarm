# tools/Makefile

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2
LIBDIR   = ../lib
SWARMDIR = swarm
BUILD_DIR = build

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
    # game.cpp targets the SFML 3 API; Homebrew's unversioned "sfml" formula
    # is SFML 3 (sfml@2 is the old pinned formula and won't compile this code).
    SFML_CFLAGS       := -I/opt/homebrew/opt/sfml/include
    SFML_LIBS         := -L/opt/homebrew/opt/sfml/lib -lsfml-graphics -lsfml-window -lsfml-system
else
    # Linux: pylon installs to /opt/pylon and ships a pylon-config query tool
    PYLON_CFLAGS       = $(shell pylon-config --cflags 2>/dev/null) \
                         -Wno-overloaded-virtual -Wno-unused-variable
    # pylon-config only emits -L/opt/pylon/lib (a link-time path); without an
    # rpath the binaries can't find libpylonbase.so.10 etc. at runtime since
    # /opt/pylon/lib isn't registered with ldconfig.
    PYLON_LIBS         = $(shell pylon-config --libs 2>/dev/null) -Wl,-rpath,/opt/pylon/lib
    SFML_CFLAGS       := $(shell pkg-config --cflags sfml-all 2>/dev/null)
    SFML_LIBS         := $(shell pkg-config --libs   sfml-all 2>/dev/null)
    # WASD multi-key polling reads /dev/input/eventN directly (see swarm/evdev_keys.h);
    # no extra link flags needed beyond libc.
    CONTROLLER_LDFLAGS =
endif

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4 2>/dev/null || pkg-config --libs   opencv 2>/dev/null)

.PHONY: all clean calibrate

all: $(BUILD_DIR)/swarm_hub $(BUILD_DIR)/swarm_hub_simulation $(BUILD_DIR)/swarm_terminal $(BUILD_DIR)/latency_plot $(BUILD_DIR)/swarm_dashboard $(BUILD_DIR)/swarm_controller $(BUILD_DIR)/game $(BUILD_DIR)/vision_controller $(BUILD_DIR)/circle_demo $(BUILD_DIR)/wingman $(BUILD_DIR)/shape_demo $(BUILD_DIR)/drag_drop_demo $(BUILD_DIR)/marker_eval $(BUILD_DIR)/frame_inspector $(BUILD_DIR)/demo_hud_preview calibrate

$(BUILD_DIR):
	mkdir -p $@

calibrate: | $(BUILD_DIR)
	$(MAKE) -C vision/calibration

$(BUILD_DIR)/swarm_hub: $(SWARMDIR)/swarm_hub.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(HUB_LDFLAGS)

$(BUILD_DIR)/swarm_hub_simulation: $(SWARMDIR)/swarm_hub_simulation.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/swarm_terminal: $(SWARMDIR)/swarm_terminal.cpp $(LIBDIR)/SwarmClient/SwarmClient.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(LIBDIR)/SwarmClient $< -o $@

$(BUILD_DIR)/latency_plot: $(SWARMDIR)/latency_plot.cpp $(LIBDIR)/SwarmClient/SwarmClient.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(LIBDIR)/SwarmClient $< -o $@

$(BUILD_DIR)/swarm_dashboard: $(SWARMDIR)/swarm_dashboard.cpp $(SWARMDIR)/telemetry_history.h $(LIBDIR)/SwarmClient/SwarmClient.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(LIBDIR)/SwarmClient $< -o $@

$(BUILD_DIR)/swarm_controller: $(SWARMDIR)/swarm_controller.cpp $(LIBDIR)/SwarmClient/SwarmClient.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(LIBDIR)/SwarmClient $< -o $@ $(CONTROLLER_LDFLAGS)

$(BUILD_DIR)/game: game.cpp $(LIBDIR)/SwarmClient/SwarmClient.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SFML_CFLAGS) -I$(LIBDIR)/SwarmClient $< -o $@ $(SFML_LIBS)

$(BUILD_DIR)/vision_controller: vision/vision_controller.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h $(SWARMDIR)/evdev_keys.h $(LIBDIR)/SwarmClient/SwarmClient.h $(LIBDIR)/DemoHud/DemoHud.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) -I$(LIBDIR)/ArucoTracker -I$(SWARMDIR) -I$(LIBDIR)/SwarmClient -I$(LIBDIR)/DemoHud $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(CONTROLLER_LDFLAGS)

$(BUILD_DIR)/circle_demo: vision/circle_demo.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h $(LIBDIR)/SwarmClient/SwarmClient.h $(LIBDIR)/DemoHud/DemoHud.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) -I$(LIBDIR)/ArucoTracker -I$(LIBDIR)/SwarmClient -I$(LIBDIR)/DemoHud $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(CONTROLLER_LDFLAGS)

$(BUILD_DIR)/wingman: vision/wingman.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h $(SWARMDIR)/evdev_keys.h $(LIBDIR)/SwarmClient/SwarmClient.h $(LIBDIR)/DemoHud/DemoHud.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) -I$(LIBDIR)/ArucoTracker -I$(SWARMDIR) -I$(LIBDIR)/SwarmClient -I$(LIBDIR)/DemoHud $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(CONTROLLER_LDFLAGS)

$(BUILD_DIR)/shape_demo: vision/shape_demo.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h $(LIBDIR)/SwarmClient/SwarmClient.h $(LIBDIR)/DemoHud/DemoHud.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) \
	    -I$(LIBDIR)/ArucoTracker -I$(LIBDIR)/SwarmClient -I$(LIBDIR)/DemoHud \
	    $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(CONTROLLER_LDFLAGS)

$(BUILD_DIR)/drag_drop_demo: vision/drag_drop_demo.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h $(LIBDIR)/SwarmClient/SwarmClient.h $(LIBDIR)/DemoHud/DemoHud.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) \
	    -I$(LIBDIR)/ArucoTracker -I$(LIBDIR)/SwarmClient -I$(LIBDIR)/DemoHud \
	    $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS) $(CONTROLLER_LDFLAGS)

$(BUILD_DIR)/marker_eval: vision/marker_eval.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h $(LIBDIR)/DemoHud/DemoHud.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) -I$(LIBDIR)/ArucoTracker -I$(LIBDIR)/DemoHud \
	    $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS)

$(BUILD_DIR)/frame_inspector: vision/frame_inspector.cpp $(LIBDIR)/ArucoTracker/aruco_tracker.h $(LIBDIR)/ArucoTracker/basler_pylon_source.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) $(PYLON_CFLAGS) -I$(LIBDIR)/ArucoTracker \
	    $< -o $@ $(OPENCV_LIBS) $(PYLON_LIBS)

# No camera/pylon/hub dependency — pure OpenCV, synthetic data only. See
# tools/vision/demo_hud_preview.cpp for the rationale.
$(BUILD_DIR)/demo_hud_preview: vision/demo_hud_preview.cpp $(LIBDIR)/DemoHud/DemoHud.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -I$(LIBDIR)/DemoHud \
	    $< -o $@ $(OPENCV_LIBS)

clean:
	rm -rf $(BUILD_DIR)
