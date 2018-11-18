# cmake min version
cmake_minimum_required(VERSION 2.8)
# project/library name
project(vrp)

# search file pattern
file(GLOB SOURCES src/*.cpp)

# create library
add_library(${PROJECT_NAME} STATIC ${SOURCES})

# add include directories visible outside of the library
# this allows to automatically find header files when build a binary
# that is linked to this library
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
