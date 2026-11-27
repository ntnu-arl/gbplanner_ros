\page boundingbox_camera Bounding Box camera

This example shows how to use the bounding box camera.

## Compile and run the example

Clone the source code, create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/gazebosim/gz-rendering
cd gz-rendering/examples/boundingbox_camera
mkdir build
cd build
cmake ..
make
```

Execute the example:

```{.sh}
./boundingbox_camera
```

You'll see:

```{.sh}
[Msg] Loading plugin [ignition-rendering-ogre2]
===============================
   S  - Save image & its boxes
  ESC - Exit
===============================
```

By default, you'll see 3D boxes:

@image html img/boundingbox_camera.png

There are 2 other types of boxes:

* 2D visible: box around the visible parts of an object.
* 2D full: box around the object, including parts that aren't visible to the camera.

Compare the two 2D box types as followS:

```{.sh}
./boundingbox_camera 2D_visible
```

@image html img/boundingbox_camera_2d_visible.png

```{.sh}
./boundingbox_camera 2D_full
```

@image html img/boundingbox_camera_2d_full.png

Note how the bounding box for the right sphere is larger for the 2D full box.
