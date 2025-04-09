# Installation

The following installation procedures have been tested on **Ubuntu 22.04** and **24.04**.

## If You Already Have ROS2

First, check the official [Kinova API repository](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L) for instructions on connecting to the robot.

`kinova_robust_control` is a ros2 package. To build it:

1. Place the codebase inside the `src/` directory of your ros2 workspace.
2. From your workspace root, run:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

You would also need to run this compile command **every time** you make changes to the code.

3. **Every time** you open a new terminal, remember to source the workspace to load custom ros2 messages:

```bash
source install/setup.bash
```

## Install Through Docker (Strongly Recommended Setup)

We highly recommend using the provided Docker environment.

A pre-configured Docker setup is available in a separate repository:  
👉 [kinova_robust_control_docker](https://github.com/roahmlab/kinova_robust_control_docker/tree/humble)

This includes a [Dockerfile](https://github.com/roahmlab/kinova_robust_control_docker/blob/humble/docker/Dockerfile) that installs all necessary dependencies.

> Need Docker? Follow the [official instructions](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

### 1. Clone the Docker Repository (with Submodules)

```bash
git clone --recurse-submodules https://github.com/roahmlab/kinova_robust_control_docker.git
```

### 2. Update `kinova_robust_control` (Optional)

```bash
git submodule update --init --recursive
```

### 3. Build the Docker Container in VS Code

1. Open VS Code.
2. Press `Ctrl+Shift+P` and search for: `Dev Containers: Rebuild and Reopen Container`.
3. Select it to automatically build the container using the provided Dockerfile.

### 4. Build `kinova_robust_control`

Inside the container, run:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

As before, source the workspace after opening a new terminal:

```bash
source install/setup.bash
```
