\page mesh_viewer Mesh Viewer

## Compile and run the example

Clone the source code, create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-rendering
cd ign-rendering/examples/mesh_viewer
mkdir build
cd build
cmake ..
make
```

To run the example:

```{.sh}
./mesh_viewer
```

@image html img/mesh_viewer.png

## Code

In the the `Main.cc` file there is a function called `buildScene` which is in charge of creating the objects in the scene (for example: the grid or the duck).
The steps to visualize a mesh are the following:

  - Create a visual
  - Attach mesh to the visual
  - Include the visual in the scene

\snippet examples/mesh_viewer/Main.cc create a mesh

Similarly, we can add a grid to the floor:

\snippet examples/mesh_viewer/Main.cc create grid visual

Or we can include a camera and set up the position, orientation, dimensions and other relevant features of cameras:

\snippet examples/mesh_viewer/Main.cc create camera
