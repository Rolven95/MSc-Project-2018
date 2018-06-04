#.rst:
# FindOpenCVBinFiles
# --------------
# Use this to find the opencv binary files (dlls) on Windows 64-Bit systems for Windows 64-Bit systems.
# This will *not* work for OSX, and will not fetch the required files for building java/android applications etc.
#
#=============================================================================
# Copyright 2018 Daniel J. Finnegan
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)


file (
  GLOB_RECURSE
    OpenCV_BIN_FILES
  ${CMAKE_SOURCE_DIR}/dependencies/opencv/build/x64/*.dll
)

# Get the first file returned
list (
  GET
  OpenCV_BIN_FILES
  0
  TEMP_BIN_FILE
)

# Get the directory name for the given path
get_filename_component (
  OpenCV_BIN_DIR
  "${TEMP_BIN_FILE}"
  DIRECTORY
)

if (${OpenCV_BIN_DIR})

  set (
    OpenCV_BIN_DIR_found
    true
    CACHE
    BOOL
    "OpenCV x64 binary directory"
  )

endif ()

message(STATUS "Found OpenCV x64 binary directory : ${OpenCV_BIN_DIR}")