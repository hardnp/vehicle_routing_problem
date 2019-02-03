# cmake min version
cmake_minimum_required(VERSION 2.8)
# project/library name
project(vrp)

# search file pattern
file(GLOB_RECURSE SOURCES src/*.cpp)

# create library
add_library(${PROJECT_NAME} STATIC ${SOURCES} src/internal/seq_insertion.cpp include/seq_insertion.h)

# add include directories visible outside of the library
# this allows to automatically find header files when build a binary
# that is linked to this library
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
