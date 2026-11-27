# Lidar Visual Example

Demo of a lidar visualisation using data from a GPU Rays sensor.
A laser sensor model is used to detect objects in the rendering window
environment and the collected data is visualised.

## Build

~~~
cd examples/lidar_visual
mkdir build
cd build
cmake ..
make
~~~

## Use

By default, the demo uses the Ogre 1 engine:

    ./lidar_visual

It's also possible to use Ogre 2:

    ./lidar_visual ogre2

