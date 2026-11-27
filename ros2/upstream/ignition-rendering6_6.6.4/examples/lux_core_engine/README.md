# LuxCore Render Engine 

This creates an example plugin using the LuxCore render engine. The implementation is incomplete and only supports basic scenes. Some parameters of the engine are hardcoded and can be adjusted for different effects:

https://wiki.luxcorerender.org/LuxCore_SDL_Reference_Manual_v2.6

## How to build and run the example
Download the LuxCoreSDK using the following link (make sure to download the SDK and not the standalone): https://luxcorerender.org/download/
1. Build and install ign-rendering:
    ```
    cd ign-rendering
    mkdir build
    cd build
    cmake ..
    make install
    ```

2. Build the example:
    ```
    export LUXCORE_SDK_PATH=<PATH_TO_LUXCORE_SDK_DIRECTORY>
    cd ../examples/lux_core_engine
    mkdir build
    cd build
    cmake ..
    make
    ```
3. Run the example:

    ~~~
    export IGN_RENDERING_PLUGIN_PATH=$PWD
    # Run the demo with the "RTPATHOCL" render engine
    ./simple_demo RTPATHOCL
    ~~~

    Hit `TAB` any time to pause and bake the scene

## Troubleshooting

If you get crash with an error message like the one below:

```
terminate called after throwing an instance of 'std::runtime_error'
what():  No hardware device selected or available
Aborted (core dumped)
```

OpenCL may not be supported or working on your machine. Try updating your
graphics drivers. Alternatively, you can try running the demo with a
different LuxCore render engine type. See a list of supported render engine
types here:

https://github.com/LuxCoreRender/LuxCore/blob/luxcorerender_v2.6/include/slg/engines/renderengine.h#L38

As an example:

~~~
# Run the demo with the "RTPATHCPU" render engine
./luxcore_demo RTPATHCPU
~~~

