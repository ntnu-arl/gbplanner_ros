\page render_pass Render pass

This example demonstrates the use of the render pass system for adding Gaussian noise post-processing effect to a camera.

## Compile and run the example

Clone the source code, create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-rendering
cd ign-rendering/examples/render_pass
mkdir build
cd build
cmake ..
make
```
Execute the example:

```{.sh}
./render_pass
```

You'll see:

```{.sh}
[Msg] Loading plugin [ignition-rendering6-ogre]
Engine 'optix' is not supported
===============================
  TAB - Switch render engines
  ESC - Exit
===============================
```

@image html img/render_pass.gif

## Code

Get the render pass system and create a Gaussian noise render pass. Then we just need to set the noise mean and the standard deviation parameters and apply this render pass to the camera.

\snippet examples/render_pass/Main.cc get render pass system
