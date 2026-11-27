\page custom_scene_viewer Custom scene viewer

This application allows us to view a number of pre constructed scenes.
The initial render engine is Ogre.

When the application starts you will see a blank window.
When you click the `+` or `-` keys you can change the scene.
By pressing the `Tab` button you will advance to the next render engine.

You may see the render engine title in the window change to Optix if you have compiled your Ignition Rendering library with OptiX (otherwise you can only use Ogre).
The frame rate may also change based on your computer's capabilities.

The following scenes have more primitive objects such as cones or cilinders. Again, you can see the scene with different render engines pressing `Tab`.
Some of the scenes available include:

 - Shapes
 - Textures
 - Normal mapping
 - Reflective property
 - Soft shadows

There are some scenes demonstrating reflective materials and mutiple point light shadows but they appear the same in ogre because they are not supported.

## Compile and run the example

Clone the source code, create a build directory and use `cmake` and `make` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-rendering
cd ign-rendering/examples/custom_scene_viewer
mkdir build
cd build
cmake ..
make
```

To run the example:

```{.sh}
./custom_scene_viewer
```

@image html img/custom_scene_viewer.png

## Code

In the main function we can have a look at all the defined scenes.

\snippet examples/custom_scene_viewer/ManualSceneDemo.cc add scenes

Inside `SceneBuilder.cc` some classes are defined which create the world. The method `BuildScene` will create the world for each scene.

For example, we can have a look at the `SimpleSceneBuilder` class inside `SceneBuilder.cc`. This class will only define the floor, the sphere and the light.

\snippet examples/custom_scene_viewer/SceneBuilder.cc build scene SimpleSceneBuilder

The `AllShapesSceneBuilder` class inside `SceneBuilder.cc` will define more objects such us: `CYLINDER`, `CONE` or `BOX`.

\snippet examples/custom_scene_viewer/SceneBuilder.cc build scene AllShapesSceneBuilder

The class `TextureSceneBuilder` includes all the objects and the textures:

\snippet examples/custom_scene_viewer/SceneBuilder.cc build scene TextureSceneBuilder

The class `NormalMapSceneBuilder` includes all the objects, the textures and the normal maps:

\snippet examples/custom_scene_viewer/SceneBuilder.cc build scene NormalMapSceneBuilder
