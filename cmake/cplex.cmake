# cmake min version
cmake_minimum_required(VERSION 2.8)

set(CPLEX_ROOT_DIR "")
if (EXISTS ${CPLEX_DIR})
    set(CPLEX_ROOT_DIR ${CPLEX_DIR})
elseif(EXISTS $ENV{CPLEX_DIR})
    set(CPLEX_ROOT_DIR $ENV{CPLEX_DIR})
else()
    message(FATAL_ERROR
        "CPLEX_DIR variable is required.\
        Either use -DCPLEX_DIR in cmake or add it as an environment variable")
endif()
    if (EXISTS )
endif()

if (UNIX)
    # include directory
    set(CPLEX_INCLUDE_DIR "${CPLEX_ROOT_DIR}/cplex/include")
    set(CPLEX_CONCERT_INCLUDE_DIR "${CPLEX_ROOT_DIR}/concert/include")
    # libraries
    # file(GLOB_RECURSE CPLEX_LIBS "${CPLEX_ROOT_DIR}/cplex/bin/**/libcplex*.so")
    # TODO: is static libs enough or dynamic needed?
    file(GLOB_RECURSE CPLEX_LIBS
        "${CPLEX_ROOT_DIR}/cplex/lib/**/*cplex*.a")
    file(GLOB_RECURSE CPLEX_CONCERT_LIBS
        "${CPLEX_ROOT_DIR}/concert/lib/**/*concert*.a")
    set(CPLEX_LIBRARIES ${CPLEX_LIBS} ${CPLEX_CONCERT_LIBS})
else()
    message(FATAL_ERROR "Finding CPLEX is not implemented for non-UNIX systems")
endif()
