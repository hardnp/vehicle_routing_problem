# cmake min version
cmake_minimum_required(VERSION 2.8)
# project/library name
project(vrp)

# search file pattern
file(GLOB_RECURSE SOURCES src/*.cpp)

# create library
add_library(${PROJECT_NAME} STATIC ${SOURCES})

# add include directories visible outside of the library
# this allows to automatically find header files when build a binary
# that is linked to this library
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

if (${WITH_CPLEX})  # add IBM CPLEX dependency
    target_compile_definitions(${PROJECT_NAME} PRIVATE NO_CPLEX_IMPL=0)
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CPLEX_INCLUDE_DIR} ${CPLEX_CONCERT_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${CPLEX_LIBRARIES})
    if (UNIX)  # TODO: check for OS X
        target_link_libraries(${PROJECT_NAME} PRIVATE pthread ${CMAKE_DL_LIBS})
    endif()
else()  # fallback to implementation stub
    target_compile_definitions(${PROJECT_NAME} PRIVATE NO_CPLEX_IMPL=1)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
