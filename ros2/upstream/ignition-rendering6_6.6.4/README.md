# Gazebo Rendering: Rendering library for robot applications

**Maintainer:** ichen [AT] openrobotics [DOT] org

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/gz-rendering.svg)](https://github.com/gazebosim/gz-rendering/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/gz-rendering.svg)](https://github.com/gazebosim/gz-rendering/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/gz-rendering/branch/ign-rendering6/graph/badge.svg)](https://codecov.io/gh/gazebosim/gz-rendering/branch/ign-rendering6)
Ubuntu Focal | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_rendering-ci-ign-rendering6-focal-amd64)](https://build.osrfoundation.org/job/ignition_rendering-ci-ign-rendering6-focal-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_rendering-ci-ign-rendering6-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_rendering-ci-ign-rendering6-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ign_rendering-ign-6-win)](https://build.osrfoundation.org/job/ign_rendering-ign-6-win)

Gazebo Rendering is a C++ library designed to provide an abstraction
for different rendering engines. It offers unified APIs for creating
3D graphics applications.

Gazebo Rendering is a component in the [Gazebo](https://gazebosim.org/) framework, a set
of libraries designed to rapidly develop robot applications.

# Table of Contents

[Features](#features)

[Install](#install)

[Usage](#usage)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#code-of-contributing)

[Versioning](#versioning)

[License](#license)

# Features

* Support for rendering engines including OGRE and OptiX (experimental)
* Plugin-based architecture. Multiple rendering engine plugins can be loaded at run time.
* Object-oriented scene management.

# Install

See the [installation tutorial](https://gazebosim.org/api/rendering/5.0/installation.html).

# Usage

The Gazebo Rendering API can be found in the documentation. See the
[installation tutorial](https://gazebosim.org/api/rendering/5.0/installation.html)
on how to build the documentation files using Doxygen.

You can also take a look at the sample applications in the `examples` folder.

# Folder Structure

* `include/ignition/rendering`: Contains all the public header files which will be installed

* `src`: Contains all the C++ source code which are not installed.

* `test`: All integration, performance and regression tests go here, under their
  specific folders.

* `examples`: Sample programs to demonstrate different features of ign-rendering

* `doc`: Files used by Doxygen when generating documentation.

Rendering engine plugin implementation code is stored in their own folders

* `ogre` : OGRE 1.x rendering engine plugin

* `ogre2` : OGRE 2.x rendering engine plugin (available in versions >= ign-rendering1)

* `optix` : OptiX rendering engine plugin

# Contributing

Please see
[CONTRIBUTING.md](https://gazebosim.org/docs/all/contributing).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/gazebosim/gz-sim/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Gazebo project](https://gazebosim.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Gazebo website](https://gazebosim.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-rendering/blob/main/LICENSE) file.
