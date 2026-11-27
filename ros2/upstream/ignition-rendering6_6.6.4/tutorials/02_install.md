\page installation Installation

These instructions are for installing only Ignition Rendering.
If you're interested in using all the Ignition libraries, check out this [Ignition installation](https://ignitionrobotics.org/docs/latest/install).

We recommend following the Binary Install instructions to get up and running as quickly and painlessly as possible.

The Source Install instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

# Ubuntu

## Binary Installation

Install dependencies:
```
sudo apt-get update
sudo apt-get -y install wget lsb-release gnupg
```

Setup your computer to accept software from
[packages.osrfoundation.org](http://packages.osrfoundation.org):
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install:
```
sudo apt-get update
sudo apt-get install libignition-rendering<#>-dev
```

Be sure to replace `<#>` with a number value, such as `1` or `2`, depending on which version you need.

## Source Installation

### Prerequisites

Ubuntu Bionic 18.04 or above:

Install dependencies:
```
sudo apt -y install wget lsb-release gnupg
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install -y \
    g++-8 \
    cmake \
    pkg-config \
    git \
    libglew-dev  \
    libfreeimage-dev \
    freeglut3-dev \
    libxmu-dev \
    libxi-dev \
    libignition-cmake2-dev \
    libignition-math6-dev \
    libignition-common4-dev \
    libignition-plugin-dev
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
```

### Supported Rendering Engines

Ignition Rendering will look for rendering libraries installed in the system and
build the relevant plugins if dependencies are found.

**OGRE 1.x**
```
# this installs ogre 1.9. Alternatively, you can install 1.8
sudo apt-get install libogre-1.9-dev
```

**OGRE 2.x (supported in Versions >= ign-rendering1)**

Add OSRF packages if you have not done so already:
```
sudo apt -y install wget lsb-release gnupg
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
```

Install OGRE 2.2 debs
```
sudo apt install libogre-2.2-dev
```

**OptiX (experimental)**

Download and install by following instructions on NVIDIA website

CUDA: http://docs.nvidia.com/cuda

OptiX: https://developer.nvidia.com/optix

Update `LD_LIBRARY_PATH` and add an `OPTIX_INSTALL_DIR` environment variables
so that ign-rendering can find Optix, e.g. if you installed version 4.0.2 in HOME/optix:
```
export LD_LIBRARY_PATH=${HOME}/optix/NVIDIA-OptiX-SDK-4.0.2-linux64/lib64:${LD_LIBRARY_PATH}
export OPTIX_INSTALL_DIR=${HOME}/optix/NVIDIA-OptiX-SDK-4.0.2-linux64
```

Note: If you encounter errors about different exception specifiers in optix math
when building Ign Rendering OptiX plugin, edit
`[optix_install_dir]/include/optixu/optixu_math_namespace.h` and comment
out the section that defines `fminf`, fmaxf, and `copysignf` (for optix
sdk 4.0.2, comment out lines 167-206).

### Build from Source

1. Clone the repository
  ```
  # Optionally, append `-b ign-rendering#` (replace # with a number) to check out a specific version
  git clone http://github.com/ignitionrobotics/ign-rendering
  ```

2. Configure and build
  ```
  cd ign-rendering
  mkdir build
  cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/install/dir
  make
  ```
  Replace `/path/to/install/dir` to whatever directory you want to install this package to

3. Optionally, install
  ```
  make install
  ```

# Windows

On Windows, only OGRE 1 is currently supported.

## Prerequisites

First, follow the [ign-cmake](https://github.com/ignitionrobotics/ign-cmake) tutorial for installing Conda, Visual Studio, CMake, and other prerequisites, and also for creating a Conda environment.

Navigate to ``condabin`` if necessary to use the ``conda`` command (i.e., if Conda is not in your `PATH` environment variable. You can find the location of ``condabin`` in Anaconda Prompt, ``where conda``).

Create if necessary, and activate a Conda environment:
```
conda create -n ign-ws
conda activate ign-ws
```

## Binary Installation

`libignition-rendering<#>` Conda feedstock is not yet available, pending [conda-forge/staged-recipes#13551](https://github.com/conda-forge/staged-recipes/issues/13551).

## Source Installation

This assumes you have created and activated a Conda environment while installing the Prerequisites.

1. Install dependencies:
  ```
  conda install ogre --channel conda-forge
  ```

2. Install Ignition dependencies:

  You can view available versions and their dependencies:
  ```
  conda search libignition-rendering* --channel conda-forge --info
  ```

  Install dependencies, replacing `<#>` with the desired versions:
  ```
  conda install libignition-cmake<#> libignition-common<#> libignition-math<#> libignition-plugin<#> --channel conda-forge
  ```

3. Navigate to where you would like to build the library, and clone the repository.
  ```
  # Optionally, append `-b ign-rendering#` (replace # with a number) to check out a specific version
  git clone https://github.com/ignitionrobotics/ign-rendering.git
  ```

4. Configure and build
  ```
  cd ign-rendering
  mkdir build
  cd build
  cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
  cmake --build . --config Release
  ```

5. Optionally, install
  ```
  cmake --install . --config Release
  ```

# macOS

## Binary Installation

On macOS, add OSRF packages:
  ```
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  brew tap osrf/simulation
  ```

Install Ignition Rendering:
  ```
  brew install ignition-rendering<#>
  ```

Be sure to replace `<#>` with a number value, such as 5 or 6, depending on
which version you need.

## Source Installation

1. Clone the repository
  ```
  git clone https://github.com/ignitionrobotics/ign-rendering -b ign-rendering<#>
  ```
  Be sure to replace `<#>` with a number value, such as 5 or 6, depending on
  which version you need.

2. Install dependencies
  ```
  brew install --only-dependencies ignition-rendering<#>
  ```
  Be sure to replace `<#>` with a number value, such as 5 or 6, depending on
  which version you need.

3. Configure and build
  ```
  cd ign-rendering
  mkdir build
  cd build
  cmake ..
  make
  ```

4. Optionally, install
  ```
  sudo make install
  ```

# Documentation

API documentation can be generated using Doxygen
  ```
  sudo apt install -y doxygen
  ```

Build documentation
  ```
  cd build
  make doc
  ```

View documentation
  ```
  firefox doxygen/html/index.html
  ```

# Testing

Tests can be run by building the `test` target:
  ```
  cd build
  make test
  ```

To run tests specific to a render engine, set the `RENDER_ENGINE_VALUES` environment variable, e.g.
  ```
  RENDER_ENGINE_VALUES=ogre2 make test
  ```
