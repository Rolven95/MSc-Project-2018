# First, go find the OpenCV binary directory
list (
	APPEND
	CMAKE_MODULE_PATH
	"${CMAKE_SOURCE_DIR}/cmake"
)

find_package (
	OpenCVBinFiles
	REQUIRED
)

set (
	HANDTRACKINGTESTAPP_SOURCES
	TestApp.cpp
)

add_executable (
	HandTrackingTestApp
	${HANDTRACKINGTESTAPP_SOURCES}
)

add_custom_command (
	TARGET
		HandTrackingTestApp
	POST_BUILD
	COMMAND
		${CMAKE_COMMAND} -DCOMPONENT=opencv -P ${CMAKE_BINARY_DIR}/cmake_install.cmake 
)

link_directories (
	${KinectSDK2_LIBRARY_DIRS}
)

target_include_directories (
	HandTrackingTestApp
	PUBLIC
		"include"
)

target_link_libraries (
	HandTrackingTestApp
	PRIVATE
		HandTrackerLib
		${KinectSDK2_LIBRARIES}
#		${Boost_LIBRARIES}
		${OpenCV_LIBS}
)

# Install the dll file into the exe directory in the project root directory
install (
	TARGETS
		HandTrackingTestApp
	RUNTIME
	DESTINATION
		${CMAKE_SOURCE_DIR}/exe
)

# Copy over the OpenCV dlls
install (
	DIRECTORY
		${OpenCV_BIN_DIR}/
	DESTINATION
		${CMAKE_SOURCE_DIR}/exe
	COMPONENT
		opencv
	FILES_MATCHING
	PATTERN
		"*.dll"
)