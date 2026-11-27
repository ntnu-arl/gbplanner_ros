\page actor_animation Actor animation tutorial

This tutorial will show you how to use the Ignition Rendering library to create an actor animation.

## Compile and run the example

Clone the source code, Create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-rendering
cd ign-rendering/examples/actor_animation
mkdir build
cd build
cmake ..
make
```

When the code is compiled you can execute the example with the following command. Using the left and right button of the mouse you can move around the scene and zoom in and out.

```{.sh}
./actor_animation
```

@image html img/actor_animation.png

## Code

In this section we will describe the main classes and methods used to create the actor. The basic steps are:

 - Load the mesh with animations
 - Create the actor visual
 - Load the mesh into the render engine
 - Attach the mesh to the actor visual
 - Animate the skeleton

Create a `MeshDescriptor` class with the mesh name of the actor, then using the `MeshManager` Singleton class, load the mesh and its animations using the descriptor:

\snippet examples/actor_animation/Main.cc load mesh

The mesh is now loaded with all animations. You can also add additional animations to the mesh:

\snippet examples/actor_animation/Main.cc add animation

Create a `Visual` pointer with the scene manager and set the position and rotation of the visual:

\snippet examples/actor_animation/Main.cc create a visual for the actor

Create the mesh in ign-rendering - this loads the animations into the render engine

\snippet examples/actor_animation/Main.cc create mesh

Finally, attach the mesh to the visual and add the visual to the scene:

\snippet examples/actor_animation/Main.cc attach mesh

There are two ways to play the animations:

 - Update animation time: The first method is to advance the time every iteration and let the render engine handle the animations.

\snippet examples/actor_animation/GlutWindow.cc update actor

 - Update bone pose: The second and more involved method is to manually compute and set the bone pose over time.

\snippet examples/actor_animation/GlutWindow.cc update pose

