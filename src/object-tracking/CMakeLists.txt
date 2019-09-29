#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME object-tracking)

# Bayes Filters
find_package(BayesFilters 0.9.0 QUIET)
if(NOT BayesFilters_FOUND)
    message(STATUS "Did not found required master release of BayesFilters. Looking for devel version.")
    find_package(BayesFilters 0.9.100 REQUIRED)
endif()

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# OpenMP
if (USE_OPENMP)
  find_package(OpenMP REQUIRED)
endif()

# PCL
find_package(PCL REQUIRED)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             eigen
             cv
             )

# armadillo
find_package(Armadillo REQUIRED)

# mlpack
find_package(MLPACK REQUIRED)

# nanoflann
find_package(nanoflann QUIET)

set(${EXE_TARGET_NAME}_HDR
    include/Correction.h
    include/CorrectionICP.h
    include/DiscretizedKinematicModel.h
    include/Filter.h
    include/GaussianCorrection.h
    include/GaussianFilter.h
    include/InitGroundTruth.h
    include/MaskSegmentation.h
    include/ObjectMeasurements.h
    include/ObjectPointCloudPrediction.h
    include/PointCloudAdaptor.h
    include/PointCloudModel.h
    include/PointCloudPrediction.h
    include/PointCloudSegmentation.h
    include/StaticPrediction.h
    include/SUKFCorrection.h
    include/Validator2D.h
    include/kdTree.h
    )

set(${EXE_TARGET_NAME}_SRC
    src/Correction.cpp
    src/CorrectionICP.cpp
    src/DiscretizedKinematicModel.cpp
    src/Filter.cpp
    src/GaussianCorrection.cpp
    src/GaussianFilter.cpp
    src/InitGroundTruth.cpp
    src/MaskSegmentation.cpp
    src/ObjectMeasurements.cpp
    src/ObjectPointCloudPrediction.cpp
    src/PointCloudModel.cpp
    src/PointCloudSegmentation.cpp
    src/StaticPrediction.cpp
    src/SUKFCorrection.cpp
    src/Validator2D.cpp
    src/main.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR
    thrift/objecttracking.thrift
    )

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               ${${EXE_TARGET_NAME}_THRIFT_SRC}
               )

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${ARMADILLO_INCLUDE_DIRS}
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${CMAKE_CURRENT_SOURCE_DIR}/include/vcglib
                           ${MLPACK_INCLUDE_DIRS}
                           )

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      BayesFilters::BayesFilters
                      CameraLibrary
                      Eigen3::Eigen
                      YARP::YARP_OS
                      YARP::YARP_eigen
                      YARP::YARP_cv
                      assimp
                      ${ARMADILLO_LIBRARIES}
                      ${MLPACK_LIBRARIES}
                      ${OpenCV_LIBS}
                      )

target_link_libraries(${EXE_TARGET_NAME} PRIVATE ${PCL_LIBRARIES})
target_include_directories(${EXE_TARGET_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(${EXE_TARGET_NAME} PRIVATE ${PCL_DEFINITIONS})
target_compile_options(${EXE_TARGET_NAME} PRIVATE ${PCL_COMPILE_OPTIONS})

target_compile_definitions(CameraLibrary PRIVATE ${PCL_DEFINITIONS})
target_compile_options(CameraLibrary PRIVATE ${PCL_COMPILE_OPTIONS})

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

file(GLOB ${EXE_TARGET_NAME}_YCB_MODELS ${CMAKE_CURRENT_SOURCE_DIR}/models/*)

file(GLOB ${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF}                           DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking)
yarp_install(FILES ${${EXE_TARGET_NAME}_YCB_MODELS}                     DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/object-tracking/models)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
