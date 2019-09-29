#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(USE_OPENMP "Use OpenMP" OFF)

add_subdirectory(validation)

if (BUILD_OBJECT_TRACKING)
   add_subdirectory(camera-library)
   add_subdirectory(object-tracking)
endif()
