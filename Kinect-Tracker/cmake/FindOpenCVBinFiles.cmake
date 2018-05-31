#.rst:
# FindOpenCVBinFiles
# --------------
# Use this to find the opencv binary files (dlls) on Windows 64-Bit systems for Windows 64-Bit systems.
# This will *not* work for OSX, and will not fetch the required files for building java/android applications etc.
# 
# @author Daniel J. Finnegan
# @date May 2018


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
    TRUE
  )

endif ()

message(STATUS "Found OpenCV x64 binary directory : ${OpenCV_BIN_DIR}")