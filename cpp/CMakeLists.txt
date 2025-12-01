cmake_minimum_required(VERSION 3.10)
project(placo_example)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# This assumes placo is cloned in placo subdirectory
add_subdirectory(placo)

find_package(Python3 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)
find_package(eigenpy REQUIRED)
find_package(hpp-fcl)

# We add a simple example
add_library(libexample SHARED example.cpp)
target_link_libraries(libexample libplaco)

# And make it a Python module instead of a main, for easy visualization
set(CMAKE_SHARED_MODULE_PREFIX "")
add_library(example MODULE module.cpp)
target_link_libraries(example libexample ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} libplaco eigenpy::eigenpy)
set_target_properties(example PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PYTHON_SITELIB})