# cmake min version
cmake_minimum_required(VERSION 2.8)

set(TBB_ROOT_DIR "")
if (EXISTS ${TBB_DIR})
    set(TBB_ROOT_DIR ${TBB_DIR})
elseif(EXISTS $ENV{TBB_DIR})
    set(TBB_ROOT_DIR $ENV{TBB_DIR})
else()
    message(FATAL_ERROR
        "TBB_DIR variable is required.\
        Either use -DTBB_DIR in cmake or add it as an environment variable")
endif()
    if (EXISTS )
endif()

if (UNIX)
    # include directory
    set(TBB_INCLUDE_DIR "${TBB_ROOT_DIR}/include")
    # libraries
    file(GLOB_RECURSE TBB_DYN_LIBS "${TBB_ROOT_DIR}/build/**/*.so")
    set(TBB_LIBRARIES ${TBB_DYN_LIBS})
else()
    message(FATAL_ERROR "Finding TBB is not implemented for non-UNIX systems")
endif()
