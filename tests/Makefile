# tests/Makefile
# Minimal unit tests for pure functions (no framework — plain asserts).

CXX       = g++
CXXFLAGS  = -std=c++17 -Wall -O2
BUILD_DIR = build

ifneq ($(shell uname),Darwin)
    SHM_LIBS = -lrt
endif

.PHONY: all test clean

all: $(BUILD_DIR)/test_protocol $(BUILD_DIR)/test_pose_hub $(BUILD_DIR)/test_mjpeg_server $(BUILD_DIR)/test_frame_shm $(BUILD_DIR)/test_car_following $(BUILD_DIR)/test_car_following_ring $(BUILD_DIR)/test_battery_log $(BUILD_DIR)/test_camera_intrinsics

test: all
	./$(BUILD_DIR)/test_protocol
	./$(BUILD_DIR)/test_pose_hub
	./$(BUILD_DIR)/test_mjpeg_server
	./$(BUILD_DIR)/test_frame_shm
	./$(BUILD_DIR)/test_car_following
	./$(BUILD_DIR)/test_car_following_ring
	./$(BUILD_DIR)/test_battery_log
	./$(BUILD_DIR)/test_camera_intrinsics

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/test_protocol: test_protocol.cpp ../lib/SwarmProtocol/protocol.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

# pose_hub.h is deliberately free of OpenCV/pylon, so this needs no extra flags.
$(BUILD_DIR)/test_pose_hub: test_pose_hub.cpp ../lib/ArucoTracker/pose_hub.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -pthread $< -o $@

# mjpeg_server.h takes finished JPEG bytes, so it too needs no OpenCV.
$(BUILD_DIR)/test_mjpeg_server: test_mjpeg_server.cpp ../lib/ArucoTracker/mjpeg_server.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

# frame_shm.h moves raw bytes through POSIX shared memory — no OpenCV. -pthread for
# the concurrent torn-frame test; -lrt for shm_open on older glibc.
$(BUILD_DIR)/test_frame_shm: test_frame_shm.cpp ../lib/ArucoTracker/frame_shm.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -pthread $< -o $@ $(SHM_LIBS)

# car_following.h is pure maths — no OpenCV, no sockets, nothing to link.
$(BUILD_DIR)/test_car_following: test_car_following.cpp ../lib/CarFollowing/car_following.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

# ring.h is the same: angles, timestamps and bookkeeping, no OpenCV or sockets.
$(BUILD_DIR)/test_car_following_ring: test_car_following_ring.cpp ../lib/CarFollowing/ring.h ../lib/CarFollowing/car_following.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

# battery_log.h is the camera-free half of tools/vision/battery_log.cpp.
$(BUILD_DIR)/test_battery_log: test_battery_log.cpp ../lib/BatteryLog/battery_log.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

# camera_intrinsics.h needs OpenCV (calib3d) but no camera or pylon.
$(BUILD_DIR)/test_camera_intrinsics: test_camera_intrinsics.cpp ../lib/ArucoTracker/camera_intrinsics.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(shell pkg-config --cflags opencv4) $< -o $@ $(shell pkg-config --libs opencv4)

clean:
	rm -rf $(BUILD_DIR)
