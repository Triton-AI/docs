---
title: Building and Compiling
hide_title: True
description: Documentation for the what packages to build to work on what
sidebar_position: 2
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import ImageGallery from "@site/src/components/ImageGallery"
import ImageCard from "@site/src/components/ImageCard"

## Build System and Workflows

### vcstool
[vcstool](https://github.com/dirk-thomas/vcstool) is a command-line tool designed to help developers manage multiple version-controlled repositories (primarily Git) in a structured and repeatable way. It is especially popular in complex software projects such as [ROS 2 (Robot Operating System)](https://docs.ros.org/), where a workspace may consist of dozens of interrelated repositories.

Instead of manually cloning or updating each repository, `vcstool` provides a consistent interface for working with all of them at once.

#### Key Features
- Clone multiple repositories from a configuration file
- Perform operations like `pull`, `status`, and `log` across all repositories
- Easy integration with CI workflows or automated setup scripts

`vcs import` is a subcommand of `vcstool` that reads a **`.repos` file** (YAML format) and checks out the listed repositories into the current workspace.

#### Example Usage
**docs.repos**
```yaml
repositories:
    src/external/tritonai/docs: # Location where you want to place the repository
        type: git
        url: git@github.com:Triton-AI/docs.git
        version: main # branch
```

```bash
vcs import < docs.repos
```
### Makefile
A **Makefile** is a special file used by the `make` build automation tool to define how to compile and link a program. It specifies a set of **rules** to automate tasks such as building binaries, running tests, or installing software.

Makefiles are commonly used in C/C++ projects, but they can be used to automate virtually any command-line task.

#### Basic Structure
```makefile
target: dependencies
	<tab>command
```

#### Example Usage
**Makefile**
```makefile
.PHONY: ucsd-gokart # .PHONY means to always make even if this exists
ucsd-gokart:
	source ./tools/scripts/source_all.sh
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to autonomy_launch basestation_launch tools_launch ucsd_gokart_launch art_lqr_interface racing_lqr
```

## colcon
`colcon` (short for **COmplex Launcher for CONtrollers**) is the **official build tool for ROS2**. It replaces `catkin` and `ament_tools`, providing a more flexible and extensible way to build and manage workspaces that contain multiple interdependent packages.

It supports both **CMake** and **Python** packages, and it's built on top of the `ament` build system, which is specific to ROS2.

#### Key Features
- Builds multiple packages in the correct dependency order
- Parallel builds for improved performance
- Extensible with plugins (e.g., test, lint, documentation)
- Provides output grouping and improved log readability
- Supports mix of CMake, Python, and plain packages

#### Basic Workflow
In ROS2, development is typically done inside a **workspace**, which is a directory that holds your source code and all build-related files. Once you've entered your workspace, you'll interact with various directories during development.

To begin, navigate to your existing workspace and build your packages:
```bash
cd ~/workspace
colcon build # This builds all your packages
colcon build --packages-select <package_name> # Specific package building
```
After running colcon build, four main directories are used or created in the workspace:
```txt
workspace/
├── src/ # Source code of your packages
├── build/ # Intermediate build files
├── install/ # Final built artifacts, libraries, scripts, etc
└── log/ # Build logs
```

:::note
You only keep ```src/``` in version control and not ```build/ install/ log/```
:::

### build
The build/ directory contains intermediate build files produced during compilation. This includes:
- CMake cache and build system files
- Object files (.o)
- Temporary or internal files for building packages


It's used by build systems (like ament_cmake) to compile packages and is not meant to be run directly or deployed. It is an intermediary layer.
Each package gets its own subdirectory:
```txt
build/
├── my_package/
│   ├── CMakeFiles/
│   └── Makefile
```
### install
The install/ directory contains the final installed outputs of your packages. This includes:
- Compiled executables
- Shared libraries
- Python modules
- Launch files
- Any other install targets defined in the package

It acts like a staging area or runtime environment, and when you run ```source install/setup.bash```, it sets up your environment using these final outputs.

### log
The log/ directory stores logs of the build process and other colcon commands:
- Build output for each package
- Error logs
- Summaries of test results

It's useful for debugging build errors and helps identify what failed and why during a build or test.

:::info
You can safely delete all three with ```rm -rf build/ install/ log/``` and is done usually for a complete clean rebuild
:::
