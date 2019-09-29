#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME validation)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenMP
if (USE_OPENMP)
  find_package(OpenMP REQUIRED)
endif()

# nanoflann
find_package(nanoflann QUIET)

set(${EXE_TARGET_NAME}_HDR
    include/Metric.h
    include/MetricAddS.h
    include/MetricEmpty.h
    include/MetricRMS.h
    include/MetricRMSV.h
    include/MetricRMSSymmetry.h
    include/PointCloudAdaptor.h
    include/ObjectResults.h
    include/SingleVideoResult.h
    include/kdTree.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/Metric.cpp
    src/MetricAddS.cpp
    src/MetricEmpty.cpp
    src/MetricRMS.cpp
    src/MetricRMSV.cpp
    src/MetricRMSSymmetry.cpp
    src/ObjectResults.cpp
    src/SingleVideoResult.cpp
    src/main.cpp
    )

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      Eigen3::Eigen
                      )

if (nanoflann_FOUND)
    message(STATUS "nanoflann found on the system. Using system library.")
    target_link_libraries(${EXE_TARGET_NAME} PRIVATE nanoflann::nanoflann)
else()
    message(STATUS "nanoflann not found on the system. Using header shipped with this package.")
    set(nanoflann_INCLUDE_DIR include/nanoflann/)
    target_include_directories(${EXE_TARGET_NAME}
                               PRIVATE
                               ${nanoflann_INCLUDE_DIR}
                               )
endif()

if (USE_OPENMP)
    if(NOT TARGET OpenMP::OpenMP_CXX)
        find_package(Threads REQUIRED)
        add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
        set_property(TARGET OpenMP::OpenMP_CXX
                     PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
        # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
        set_property(TARGET OpenMP::OpenMP_CXX
                     PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
    endif()
    target_link_libraries(${EXE_TARGET_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
