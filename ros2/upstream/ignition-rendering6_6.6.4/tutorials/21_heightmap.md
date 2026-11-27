\page heightmap Heightmap

This example shows how to add a heigntmap to the scene.

It loads 2 different heightmaps with different parameters.

## Compile and run the example

Clone the source code, create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-rendering
cd ign-rendering/examples/heightmap
mkdir build
cd build
cmake ..
make
```
Execute the example:

```{.sh}
./heightmap
```

You'll see:

```{.sh}
[Msg] Loading plugin [ignition-rendering6-ogre]
[Msg] Loading heightmap: scene::Heightmap(65528)
[Msg] Heightmap loaded. Process took 217 ms.
===============================
  TAB - Switch render engines
  ESC - Exit
===============================
```
@image html img/heightmaps.png

## Code

A heightmap is a terrain defined by a 2D grid of height values. This demo
loads the heights from a grayscale image, where the color black represents
the lowest point, and white represents the highest.

The heightmap's information is stored in the `HeightmapDescriptor` class.
In addition to the height data, the heightmap descriptor also exposes
functionality such as:

* Its size in meters in XYZ space.
* The position of its origin in the world.
* The textures to use according to the height.
* How to blend these textures.

Here's the snippet of code from `examples/heightmap/Main.cc` that adds a heightmap
to the scene:

\snippet examples/heightmap/Main.cc create a heightmap
