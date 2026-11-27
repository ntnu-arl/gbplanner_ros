# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Ignition Rendering 6.2.1 to 6.X

### Modifications

1. Ogre 2 heightmaps: the Y position sign was flipped

1. `Scene::SetTime` is often unset. Ignition's `Ogre2` now defaults to 60hz otherwise rendering won't advance forward.
	+ Mostly affects Particles.
	+ Also may affect gaussian postprocessing and other filters dependant on time.
	+ Previous behavior was using real time instead of simulation time, which is wrong.
	+ See https://github.com/ignitionrobotics/ign-rendering/issues/556 for details.

## Ignition Rendering 5.x to 6.x

### Modifications

1. **Scene.hh**
    + Added `Scene::PostRender`. The function `Camera::Render` must be executed
      between calls to `Scene::PreRender` and `Scene::PostRender`. Failure to do
      so will result in asserts triggering informing users to correct their code.
      Alternatively calling `Scene::SetCameraPassCountPerGpuFlush( 0 )` avoids
      this strict requirement.
      Users handling only one Camera can call `Camera::Update` or `Camera::Capture`
      and thus do not need to worry.
      However for more than one camera (of any type) the optimum way to handle them is to update them in the following pattern:
      ```
      scene->PreRender();
      for( auto& camera in cameras )
          camera->Render();
      for( auto& camera in cameras )
          camera->PostRender();
      scene->PostRender();
      ```
      This pattern maximizes the chances of improving performance.
      *Note*: Calling instead `Camera::Update` for each camera is a waste of CPU resources.
    + It is invalid to modify the scene between `Scene::PreRender` and `Scene::PostRender` (e.g. add/remove objects, lights, etc)
    + Added `Scene::SetCameraPassCountPerGpuFlush`. Setting this value to 0 forces legacy behavior which eases porting.
    + Systems that rely on Graphics components like particle FXs and postprocessing are explicitly affected by Scene's Pre/PostRender. Once `Scene::PostRender` is called, the particle FXs' simulation is moved forward, as well as time values sent to postprocessing shaders. In previous ign-rendering versions each `Camera::Render` call would move the particle simulation forward, which could cause subtle bugs or inconsistencies when Cameras were rendering the same frame from different angles. Setting SetCameraPassCountPerGpuFlush to 0 will also cause these subtle bugs to reappear.

1. **Visual.hh** and **Node.hh**
    + `*UserData` methods and the `Variant` type alias have been moved from the `Visual` class to the `Node` class.
    `Node::UserData` now returns no data for keys that don't exist (prior to Rendering 6.x, if
    `Visual::UserData` was called with a key that doesn't exist, an `int` was returned by default).

1. **depth_camera_fs.glsl** and **depth_camera_final_fs.glsl**
    + Far clipping changed from clipping by depth to clipping by range, i.e. distance to point, so that the data generated will never exceed the specified max range of the camera.

## Ignition Rendering 4.0 to 4.1

## ABI break

1. **ogre2/include/ignition/rendering/ogre2/Ogre2DepthCamera.hh**
    + Medium severity ABI break with the addition of the `AddRenderPass` override.

## Ignition Rendering 3.X to 4.X

### Deprecations

1. **ignition::common::Time** deprecated in favor of **std::chrono::steady_clock::duration**
    + Deprecated: `void BaseScene::SetSimTime(const common::Time &_time)`
    + Replacement: `void BaseScene::SetTime(const std::chrono::steady_clock::duration &_time)`
    + Deprecated: `common::Time SimTime() const`
    + Replacement: `std::chrono::steady_clock::duration Time() const`
    + Deprecated: `common::Time simTime`
    + Replacement: `std::chrono::steady_clock::duration time`

## Ignition Rendering 2.X to 3.X

### Deletions

1. **Ogre2DepthCamera.hh**
    + Removed unused member variables `captureData` and `newData`

## Ignition Rendering 1.X to 2.X

### Modifications

1. **Visual.hh** and **Node.hh**
    + All functions for setting and getting scale properties in the Visual class
      are moved to the base Node class. These functions are now inherited by
      the Visual class.
