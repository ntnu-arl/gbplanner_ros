\page camera_tracking Camera tracking

This example shows a camera tracking a moving target. You can use the keyboard to move the target being tracked, toggle between different tracking modes, and adjust the tracking offsets.

## Compile and run the example

Clone the source code, create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-rendering
cd ign-rendering/examples/camera_tracking
mkdir build
cd build
cmake ..
make
```

To run the example:

```{.sh}
./camera_tracking
```

You'll see:

```{.sh}
===============================
  TAB - Switch render engines
  ESC - Exit

  W: Move box forward
  S: Move box backward
  A: Rotate box to the left
  D: Rotate box to the right

  1: Camera tracking only
  2: Camera tracking and
     following
  3: Camera tracking and
     following (world frame)

  T: Toggle smooth tracking
  F: Toggle smooth following

  Track offset
  u/U: +- 0.1 on X
  i/I: +- 0.1 on Y
  o/O: +- 0.1 on Z

  Follow offset
  j/J: +- 0.1 on X
  k/K: +- 0.1 on Y
  l/L: +- 0.1 on Z
===============================

```

@image html img/camera_tracking.png

## Code

There are two main functions in the `Camera` class that allow us to track and follow the object:

[SetTrackTarget](https://ignitionrobotics.org/api/rendering/4.0/classignition_1_1rendering_1_1Camera.html#a492d6ad21711d6050ccda1a96676f8fe): Set a node for camera to track. The camera will automatically change its orientation to face the target being tracked. If a null node is specified, tracking is disabled. In contrast to `SetFollowTarget` the camera does not change its position when tracking is enabled.

\snippet examples/camera_tracking/GlutWindow.cc camera track

[SetFollowTarget](https://ignitionrobotics.org/api/rendering/4.0/classignition_1_1rendering_1_1Camera.html#ae5dac859964f1c47eba4d46d6d3ef9f9): Set a node for camera to follow. The camera will automatically update its position to keep itself at the specified offset distance from the target being followed. If a null node is specified, camera follow is disabled. In contrast to `SetTrackTarget`, the camera does not change its orientation when following is enabled.

\snippet examples/camera_tracking/GlutWindow.cc camera follow
