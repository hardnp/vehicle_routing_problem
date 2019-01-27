# cmake min version
cmake_minimum_required(VERSION 2.8)
# project/binary name
project(vrp_solver)

# set cmake standard, turn off non-standard extensions
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

# set compiler flags for debug/release
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function")
    # ignored due to cplex:
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes")
    set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS} -Wall -Werror -g -O0")
    set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS} -Wall -Werror -O3")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function")
    # ignored due to cplex:
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes")
    set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS} -Wall -Werror -g -O0")
    set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS} -Wall -Werror -O3")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS} /Wall /WX /O0")
    set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS} /Wall /WX /O3")
endif()

# add CMAKE_MODULE_PATH entries for local modules discovery
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find CPLEX if CPLEX_DIR is set
option(WITH_CPLEX "Enable IBM CPLEX heuristics" OFF)
if (${WITH_CPLEX})
    include(cplex)
    message("CPLEX include directory: ${CPLEX_INCLUDE_DIR}")
    message("CONCERT include directory: ${CPLEX_CONCERT_INCLUDE_DIR}")
    message("CPLEX libraries: ${CPLEX_LIBRARIES}")
endif()

# add directory with library
add_subdirectory(lib)

# create binary executable
add_executable(${PROJECT_NAME} main.cpp)

# link libraries to binary
target_link_libraries(${PROJECT_NAME} vrp)
