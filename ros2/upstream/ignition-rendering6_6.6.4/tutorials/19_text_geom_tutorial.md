\page text_geom Text geometry

This example shows how to include text in the scene.

## Compile and run the example

Clone the source code, create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-rendering
cd ign-rendering/examples/text_geom
mkdir build
cd build
cmake ..
make
```
Execute the example:

```{.sh}
./text_geom
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
@image html img/text_geom.png

## Code

We can create text in the same way we create other objects in the scene. We just need to use the scene to create the text using the method `CreateText`. It's possible to configure: font, text and alignment. Then we need to add the text as a visual in the scene:

\snippet examples/text_geom/Main.cc create text geometry
