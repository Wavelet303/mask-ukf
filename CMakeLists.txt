#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

cmake_minimum_required(VERSION 3.5)

project(mask-ukf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

option(BUILD_OBJECT_TRACKING "Build object-tracking" OFF)
if (BUILD_OBJECT_TRACKING)
   # iCub contrib
   find_package(ICUBcontrib REQUIRED)
endif()

# Default build type to Release
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                              ${ICUBCONTRIB_MODULE_PATH}
                              )

if (BUILD_OBJECT_TRACKING)
   include(ICUBcontribHelpers)
   include(ICUBcontribOptions)
   icubcontrib_set_default_prefix()
   icubcontrib_add_uninstall_target()
endif()

add_subdirectory(src)
