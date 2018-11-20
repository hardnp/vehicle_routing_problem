Vehicle Routing Problem (VRP) solver
===

## Basic build

See [documentation](https://cmake.org/cmake/help/v3.12/manual/cmake.1.html) on
how to build with CMake.

> By default, `cmake` generates Unix Makefiles, if your system supports it (e.g.
> `Linux`), you can use it to build the code. Otherwise, see [specific cmake
> command-line](#cross-platform-specifics-cmake-generators) when building for
> `Windows`, `OSX`.

* Release
    ~~~bash
    mkdir build
    cd build/
    cmake ..
    cmake --build .
    ~~~

* Debug
    ~~~bash
    mkdir build
    cd build/
    cmake -DCMAKE_BUILD_TYPE=Debug ..
    cmake --build .
    ~~~

* Built executable `vrp_solver` is located under `build/` directory
    ~~~bash
    [Linux] ./build/vrp_solver test_data/sample_input.csv
    [Windows] build\\vrp_solver.exe test_data\\sample_input.csv
    ~~~

### Cross Platform Specifics: cmake generators

* `Windows`: change `cmake ..` to:
    ~~~bash
    cmake -G "Visual Studio <version> Win64" ..
    ~~~
    This will generate MSVC solution

* `OSX`: change `cmake ..` to:
    ~~~bash
    cmake -G Xcode ..
    ~~~
    This will generate XCode project
