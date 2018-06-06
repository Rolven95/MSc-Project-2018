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

find_package (
	OpenCV
	REQUIRED
	PATHS
		"${CMAKE_SOURCE_DIR}/dependencies/opencv/build" # You might need to changes this based on future releases of OpenCV. If you do, come talk to me as we need to adapt this file to make things future-proof!
)

list (
	APPEND
	CMAKE_MODULE_PATH
	"${CMAKE_SOURCE_DIR}/cmake"
)

find_package (
	KinectSDK2
	REQUIRED
)

set (
	QRTESTAPP_SOURCES
	video.cpp
	../HandTrackerLib/KinectManager.cpp
	../HandTrackerLib/HandTracker.cpp
	../HandTrackerLib/Hand.cpp
)

set (
	QRTESTAPP_HEADERS
	../HandTrackerLib/include/KinectManager.h
)

add_executable (
	QRTestApp
	${QRTESTAPP_SOURCES}
	${QRTESTAPP_HEADERS}
)

link_directories (
	${KinectSDK2_LIBRARY_DIRS}
)

target_include_directories (
	QRTestApp
	PUBLIC
		${CMAKE_SOURCE_DIR}/src/HandTrackerLib/include
		${KinectSDK2_INCLUDE_DIRS}
		${Boost_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
)

link_directories (
	${KinectSDK2_LIBRARY_DIRS}
#	${Boost_LIBRARY_DIRS}
#	${OpenCV_LIB_DIR} # According to the OpenCV cmake config file, this is not needed
)

target_link_libraries (
	QRTestApp
	PRIVATE
		${KinectSDK2_LIBRARIES}
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