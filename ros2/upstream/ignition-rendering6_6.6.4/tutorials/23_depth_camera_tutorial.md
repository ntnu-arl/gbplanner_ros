\page depth_camera Depth camera

This example shows how to use the depth camera.

## Compile and run the example

Clone the source code, create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-rendering
cd ign-rendering/examples/simple_demo
mkdir build
cd build
cmake ..
make
```

Execute the example:

```{.sh}
./depth_camera ogre
```

You'll see:

```{.sh}
[Msg] Loading plugin [ignition-rendering-ogre]
===============================
  ESC - Exit                   
===============================
```

@image html img/depth_camera_ogre.png

## Code

Most of the code is adapted from \ref simple_demo, here we outline the key differences. 

The function `buildScene()` is responsible for creating and configuring the depth camera.
The main points to note are:

- The image format is set to `PixelFormat::PF_FLOAT32_RGBA`,
- Anti-alising is enabled,
- The depth texture is created after configuration is modified from defaults.

\snippet examples/depth_camera/Main.cc create camera

The window updating code is in `GlutWindow.cc`. The function `initCamera()`
registers a callback with the camera that captures a copy of
the depth image each frame. This will later be drawn to the window in
the main update loop.

\snippet examples/depth_camera/GlutWindow.cc init camera

The depth camera is capable of generating both color and depth data.
This example shows how to connect and receive depth data in the
`OnNewDepthFrame()` callback function.

\snippet examples/depth_camera/GlutWindow.cc depth frame callback

It converts the depth image to a RGB grayscale format and copies it from the
depth camera to a memory reserved in a globally scoped `Image`.

\snippet examples/depth_camera/GlutWindow.cc convert depth to image

The other option is the [ConnectNewRgbPointCloud](https://github.com/ignitionrobotics/ign-rendering/blob/ign-rendering6/include/ignition/rendering/DepthCamera.hh#L58) call that receives colored point cloud data on callback.

The reason for setting the camera image format to `PixelFormat::PF_FLOAT32_RGBA`
is to ensure that when the `Image` is created it reserves a buffer of the
correct size so that it is able to pack both color and depth data.
