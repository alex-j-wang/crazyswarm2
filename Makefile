WORKERS := $(shell expr `nproc` - 2)

.PHONY: all clean

all:
	@echo "Building with $(WORKERS) workers"
	@cd /ros_ws && colcon build --symlink-install --parallel-workers $(WORKERS) --cmake-args -DCMAKE_BUILD_TYPE=Debug

clean:
	@echo "Removing build & log"
	@cd /ros_ws && rm -rf build log
