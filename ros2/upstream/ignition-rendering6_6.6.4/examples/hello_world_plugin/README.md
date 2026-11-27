# Engine plugin

This example shows how to create a plugin that integrates a rendering engine with
Ignition Rendering and how to load it with Ignition Gazebo.

## Build

From the root of the `ign-rendering` repository, do the following to build the example:

~~~
cd examples/hello_world_plugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `HelloWorldPlugin` library under `build`.
The exact name of the library file depends on the operating system
such as `libHelloWorldPlugin.so` on Linux, `libHelloWorldPlugin.dylib` on macOS,
and `HelloWorldPlugin.dll` on Windows.

## Run

Be sure to have the `IGN_GAZEBO_RENDER_ENGINE_PATH` environment variable set to the path
where your plugin is located.  From within the `build` directory of this example, you can run

~~~
export IGN_GAZEBO_RENDER_ENGINE_PATH=$PWD
~~~

to set the environment variable accordingly.


Now you can run `ign gazebo` with the name of the resultant library file (without the `lib` prefix
or the file extension, i.e., libHelloWorldPlugin.so -> HelloWorldPlugin):

~~~
ign gazebo --render-engine HelloWorldPlugin
~~~

You should see a blank screen within the Ignition GUI, as this mocked plugin provides no implementation
for the scene.
