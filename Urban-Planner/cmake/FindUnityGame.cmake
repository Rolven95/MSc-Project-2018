# Find functionality for the Unity Game Engine
#
# Note this may not work as suspected; the best
# I can do for now is get the *runtime* version of Unity.
# The assumption is that the runtime version is the same
# as the editor version.
# Unless you have a bespoke installation, this assumption
# should be valid.
#
# Variables:
# 	- UnityGame_EXECUTABLE -> The main Unity executable
#	- UnityGame_VERSION_STRING -> The version number of Unity
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

if (NOT EXISTS "${CMAKE_SOURCE_DIR}/unity/Assets/Editor/GetUnityVersion.cs")

	set (
		UnityGame_FOUND
		false
		CACHE
		BOOL
		""
	)

endif ()

# Read the variables from the cache

if (APPLE) # We only support OSX or Windows, both 64 bit

	set (
		UnityGame_EXECUTABLE
		/Applications/Unity/Unity.app/Contents/MacOS/Unity
		CACHE
		STRING
		"Path to the unity editor executable"
	)

elseif (WIN32) # We only support OSX or Windows, both 64 bit

	set (
		UnityGame_EXECUTABLE
		C:/Program Files/Unity/Editor/Unity.exe
		CACHE
		STRING
		"Path to the unity editor executable"
	)

else () # Just echo a statement on any other system

	message (
		SEND_ERROR
		"FindUnityGame.cmake : Only MacOS and Windows supported."
	)

	set (
		UnityGame_FOUND
		false
		CACHE
		BOOL
		""
	)

	set (
		UnityGame_VERSION_STRING
	)

endif ()

# The arguments specifying the path are passed first as the GetUnityVersion.GetVersionInfo
# inspects the first argument passed

if (NOT UnityGame_FOUND)

	execute_process (
		COMMAND
			${UnityGame_EXECUTABLE}
			"${CMAKE_BINARY_DIR}/unity_version.txt"
			-quit
			-batchmode
			-projectPath ${CMAKE_SOURCE_DIR}/unity/
			-executeMethod GetUnityVersion.GetVersionInfo 
		RESULT_VARIABLE
			_UNITY_GAME_COMMAND_SUCCESS
	)

	if (${_UNITY_GAME_COMMAND_SUCCESS} EQUAL 1)

		message (
			SEND_ERROR
			"FindUnityGame.cmake : Couldn't run the Unity command. Please check installation for ${CMAKE_SOURCE_DIRECTORY}/unity/Assets/Editor/GetUnityVersion.cs"
		)

		set (
			UnityGame_FOUND
			false
			CACHE
			BOOL
			""
		)

		set (
			_UNITY_GAME_COMMAND_SUCCESS
		)

	else ()

		file (
			STRINGS
			"${CMAKE_BINARY_DIR}/unity_version.txt"
			_UNITY_GAME_VERSION_NUM
		)

		# All our tests have passed, so set the main cache variables

		list (
		  GET
		  _UNITY_GAME_VERSION_NUM
		  0
		  UnityGame_VERSION_MAJOR
		)

		list (
		  GET
		  _UNITY_GAME_VERSION_NUM
		  1
		  UnityGame_VERSION_MINOR
		)

		set (
			_UNITY_GAME_VERSION_NUM
		)

		set (
			UnityGame_VERSION_STRING
			${UnityGame_VERSION_MAJOR}.${UnityGame_VERSION_MINOR}
			CACHE
			STRING
			"The version of unity installed on the system"
		)

		if(UnityGame_FIND_VERSION_EXACT)

			if (${UnityGame_VERSION_STRING} VERSION_LESS ${UnityGame_FIND_VERSION})

				set (
					UnityGame_FOUND
					false
					CACHE
					BOOL
					""
				)

			else ()
			
				set (
					UnityGame_FOUND
					true
					CACHE
					BOOL
					""
				)

			endif ()

		endif ()

	endif ()

endif ()

message (
	STATUS
	"Found unity : ${UnityGame_FOUND}"
)

message (
	STATUS
	"Found unity : ${UnityGame_EXECUTABLE}"
)

message (
	STATUS
	"Unity version : ${UnityGame_VERSION_STRING}"
)