# Basic grid example
add_executable(grid_basic grid_basic.cpp)
target_link_libraries(grid_basic PRIVATE astar_lite)

# Put all examples in the examples folder
set_target_properties(grid_basic
    PROPERTIES
    FOLDER "examples"
)
