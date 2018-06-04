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
	QRTESTAPP_SOURCES
	video.cpp
)

add_executable (
	QRTestApp
	${QRTESTAPP_SOURCES}
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
	QRTestApp
	PRIVATE
		HandTrackerLib
		${KinectSDK2_LIBRARIES}
#		${Boost_LIBRARIES}
		${OpenCV_LIBS}
)

# Install the dll file into the exe directory in the project root directory
install (
	TARGETS
		QRTestApp
	RUNTIME
	DESTINATION
		${CMAKE_SOURCE_DIR}/exe
)