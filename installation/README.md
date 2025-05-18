# Installation (humble-with-RAPTOR)

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
👉 [kinova_robust_control_docker](https://github.com/roahmlab/kinova_robust_control_docker/tree/humble-with-RAPTOR)

This includes a [Dockerfile](https://github.com/roahmlab/kinova_robust_control_docker/blob/humble-with-RAPTOR/docker/Dockerfile) that installs all necessary dependencies.

> Need Docker? Follow the [official instructions](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

### 1. Clone the Docker Repository (with Submodules)

```bash
git clone --recurse-submodules https://github.com/roahmlab/kinova_robust_control_docker.git
```

### 2. Update `kinova_robust_control` (Optional)

Go to `kinova_robust_control_docker` first:
```bash
cd kinova_robust_control_docker
```

Update all the submodules, including `kinova_robust_control`:
```bash
git submodule update --init --recursive
```

### 3. HSL
You should complete HSL steps BEFORE you build the docker image otherwise you will have error.

We have selected [HSL](https://www.hsl.rl.ac.uk/) to solve large linear systems in the nonlinear optimization problem. 
Please follow the instructions below to complete the installation.

Besides official HSL code, we used [ThirdParty-HSL](https://github.com/coin-or-tools/ThirdParty-HSL), which is specifically tailored for COIN-OR projects, particularly Ipopt, offering easier integration and installation.
    ```
    git clone https://github.com/coin-or-tools/ThirdParty-HSL.git. 
    ```
1. Download the tarball containing the Coin-HSL source code from its official [website](https://licences.stfc.ac.uk/product/coin-hsl). You will need to apply a license for it. The academic license is free but it could take 1 or 2 days to process the order.
2. Download code from [ThirdParty-HSL](https://github.com/coin-or-tools/ThirdParty-HSL).
3. Unpack the Coin-HSL source code and rename the folder as `coinhsl`.
4. Place the `coinhsl` folder inside [ThirdParty-HSL](https://github.com/coin-or-tools/ThirdParty-HSL) folder, which serves as a wrapper to simplify the compilation and integration of HSL.
5. Rename the [ThirdParty-HSL](https://github.com/coin-or-tools/ThirdParty-HSL) folder as `HSL`,then compress it into a `HSL.zip` file. Note: This zip file will later be 'unziped' and built inside the docker container.

### 4. Build the Docker Container in VS Code

 - Open VS Code.
 - Open `kinova_robust_control_docker` folder in VS Code.
 - Press `Ctrl+Shift+P` and search for: `Dev Containers: Rebuild and Reopen Container`.
 - Select it to automatically build the container using the provided Dockerfile.

### 5. Build `kinova_robust_control`

Inside the container, run:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

As before, source the workspace after opening a new terminal:

```bash
source install/setup.bash
```

## Important Configuration Note

You may need to modify include paths in the following files:
- [dynamics/CMakeLists.txt](../dynamics/CMakeLists.txt) (line 36)
- [kortex/CMakeLists.txt](../kortex/CMakeLists.txt) (line 32)

Both files contain this include directive:
```cmake
include_directories(/workspaces/kinova_robust_control_docker/install/customized_msgs/include)
```

This path is hardcoded to match our default Docker workspace. 
Depending on your build environment and workspace name, you'll need to update this path to point to your `customized_msgs` package location.