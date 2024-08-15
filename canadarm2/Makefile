# Define variables
DOCKER_IMAGE = osrf/space-ros:canadarm_demo
WORKSPACE = ${HOME}/canadarm_ws
LOCAL_WORKSPACE = $(shell pwd)
SHELL := /bin/bash
XTERM_CONFIG = -bg black -fg white -fa 'Monospace' -fs 11

# Help target to describe each target
.PHONY: help
help:
	@echo "CanadArm2 Makefile"
	@echo "  make build          - Build the Docker image and the Gazebo workspace locally"
	@echo "  make build-docker   - Build the Docker image"
	@echo "  make build-gazebo   - Build the Gazebo workspace locally"
	@echo "  make run-gazebo     - Run the CanadArm2 Gazebo simulation locally"
	@echo "  make run-demo       - Run the CanadArm2 demo within Docker"
	@echo "  make clean          - Clean the local workspace"
	@echo "  make all            - Build and run everything"

# Build all
.PHONY: build
build: build-docker build-gazebo

# Build the Docker image
.PHONY: build-docker
build-docker:
	docker build -t $(DOCKER_IMAGE) .

# Run the CanadArm Gazebo simulation locally
.PHONY: run-gazebo
run-gazebo:
	xterm $(XTERM_CONFIG) -T 'CanadArm2 Gazebo' -e 'source /opt/ros/${ROS_DISTRO}/setup.bash \
		&& source $(LOCAL_WORKSPACE)/install/setup.bash \
		&& ros2 launch canadarm_gazebo canadarm.launch.py' &

# Build the Gazebo workspace locally
.PHONY: build-gazebo
build-gazebo:
	@source /opt/ros/${ROS_DISTRO}/setup.bash && \
	rosdep install --from-paths canadarm_gazebo --ignore-src -r -y && \
	rosdep install --from-paths canadarm_description --ignore-src -r -y && \
	colcon build --symlink-install --base-paths $(LOCAL_WORKSPACE) --install-base $(LOCAL_WORKSPACE)/install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        --packages-select canadarm_description canadarm_gazebo

# Run the CanadArm demo within Docker
.PHONY: run-docker
run-docker:
	xterm $(XTERM_CONFIG) -T 'CanadArm2 Demo' -e "docker run -it --rm \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		$(DOCKER_IMAGE) \
		bash -c 'source ~/.bashrc && ros2 launch canadarm_demo canadarm.launch.py'" &

.PHONY: clean
clean:
	rm -rf $(LOCAL_WORKSPACE)/install $(LOCAL_WORKSPACE)/log $(LOCAL_WORKSPACE)/build

# Build and run everything
.PHONY: run
run: build run-gazebo run-docker
