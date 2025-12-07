# AStar-Lite

Simple, lightweight, header-only C++20 implementation of the A* search algorithm (with zero external dependencies).

## Overview

**A\* Pathfinding Library (Header-Only, C++20)**

A lightweight, modern, header-only C++ library that provides a clean and efficient implementation of the A* search algorithm.
The library is designed to be simple to integrate, easy to extend, and fast enough for real-time applications such as games, robotics, and simulation.

This repository includes:

- A generic, reusable A* search engine
- Example programs demonstrating the API
- A clean CMake configuration for seamless integration

## Features

- **Header-only** — just include and use.
- **C++20** — uses constrained templates and modern language features.
- **Customizable heuristics** — plug in your own metric or cost functions.
- **Extensible node types** — use grids, graphs, or your own custom domain.
- **Deterministic and fast** — suitable for real-time applications.
- **Well-structured examples** — get started quickly.

## Getting Started

### Add the library to your project

Because the library is header-only, integration is easy.

#### Option 1: Copy the headers directly

Copy the `include/` directory into your project and add:

``` cpp
#include "AStar-Lite.hpp"
```

#### Option 2: Use CMake

Inside your project:

``` cmake
find_package(astar_lite REQUIRED)
target_link_libraries(your_project PRIVATE astar_lite)
```

Or add it as a subdirectory:

``` cmake
add_subdirectory(path/to/astar_lite)
target_link_libraries(your_project PRIVATE astar_lite)
```

## Building the Examples

You can build the examples using the included CMake configuration.

``` bash
git clone https://github.com/ethan-och/AStar-Lite.git
cd AStar-Lite
mkdir build
cd build
cmake ..
cmake --build .
```

This will create the example executables:

- grid_basic

Run them from the build directory:

``` bash
./examples/grid_basic
```

On Windows:

``` powershell
.\examples\Debug\grid_basic.exe
```

## Installation (Optional)

If you want to make the library available system-wide:

``` bash
cmake --install build/
```

Then other CMake projects can use:

``` cmake
find_package(astar REQUIRED)
target_link_libraries(myapp PRIVATE astar)
```

## Support

If you enjoy this library, consider giving the repository a star! It helps others discover it and motivates future improvements. 😊
