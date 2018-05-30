set (
	HANDTRACKINGTESTAPP_SOURCES
	TestApp.cpp
	Unity.cpp
)

set (
	HANDTRACKINGTESTAPP_HEADERS
	"include/Unity.h"
)

add_executable (
	HandTrackingTestApp
	${HANDTRACKINGTESTAPP_SOURCES}
	${HANDTRACKINGTESTAPP_HEADERS}
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