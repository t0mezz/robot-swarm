# tests/Makefile
# Minimal unit tests for pure functions (no framework — plain asserts).

CXX       = g++
CXXFLAGS  = -std=c++17 -Wall -O2
BUILD_DIR = build

.PHONY: all test clean

all: $(BUILD_DIR)/test_protocol $(BUILD_DIR)/test_pose_hub $(BUILD_DIR)/test_car_following

test: all
	./$(BUILD_DIR)/test_protocol
	./$(BUILD_DIR)/test_pose_hub
	./$(BUILD_DIR)/test_car_following

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/test_protocol: test_protocol.cpp ../lib/SwarmProtocol/protocol.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

# pose_hub.h is deliberately free of OpenCV/pylon, so this needs no extra flags.
$(BUILD_DIR)/test_pose_hub: test_pose_hub.cpp ../lib/ArucoTracker/pose_hub.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -pthread $< -o $@

# car_following.h is pure maths — no OpenCV, no sockets, nothing to link.
$(BUILD_DIR)/test_car_following: test_car_following.cpp ../lib/CarFollowing/car_following.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -rf $(BUILD_DIR)
