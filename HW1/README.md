## Building and Running the Renderer

1. Navigate to the build directory or make a new one:
`mkdir build && cd build`

2. Run the following commands from the build directory:
```
cmake ..
cmake --build .
```

3. Run the executable with:
`./renderer`

If you would like to change camera settings or mesh parameters, look at src/render.cpp. Materials are hardcoded and can be changed in include/ray.h
