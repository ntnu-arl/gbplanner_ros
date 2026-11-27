\page custom_shaders Custom shaders

This example shows how use custom shaders in ign-rendering to change the appearance of objects in the scene. It demonstrates two uses of shaders: The first is setting shaders for a camera and the other is setting shaders for an object in the scene.

## Compile and run the example

Clone the source code, create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-rendering
cd ign-rendering/examples/custom_shaders
mkdir build
cd build
cmake ..
make
```

```{.sh}
./custom_shaders
```

You'll see:

```{.sh}
Image saved: depth.png
Image saved: regular.png
```

The `depth.png` shows result of setting shaders for a camera: This example shades objects in the scene based on their depth values

@image html img/custom_shaders_depth.png


The `regular.png` shows result of setting shaders for a visual:  This example changes the box visual in the scene to a flat green color

@image html img/custom_shaders_rgb.png

## Code

In the first lines of the `custom_shaders.cc` file there are some constants defined with the name of the shaders that the program will use:

\snippet examples/custom_shaders/custom_shaders.cc init shaders variables

Construct absolute path to the shaders. Here the `RESOURCE_PATH` variable points to the `media` directory where the two sets of shaders are located:

\snippet examples/custom_shaders/custom_shaders.cc Get shader path

The first set of shaders are applied to the camera:

\snippet examples/custom_shaders/custom_shaders.cc add shader to camera

The second set of shaders are applied to the visual's material:

\snippet examples/custom_shaders/custom_shaders.cc add shader to visual
