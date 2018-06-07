# Find functionality for the Unity Game Engine
#
# This currently only works for Unity.
# Operates by simply querying the Windows Registry (regedit.exe)
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

# Search the windows registry for the installation of Unity

if (WIN32)

if (NOT UnityGame_FOUND)

	get_filename_component (
		UnityGame_EXECUTABLE
		"[HKEY_LOCAL_MACHINE\\SOFTWARE\\WOW6432Node\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Unity;DisplayIcon]"
		ABSOLUTE
		CACHE
	)

	get_filename_component (
		_UNITY_GAME_VERSION_NUM
		"[HKEY_LOCAL_MACHINE\\SOFTWARE\\WOW6432Node\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Unity;DisplayVersion]"
		NAME
	)

	# Do a match on Unity to make sure we found the right key
	if (UnityGame_EXECUTABLE MATCHES "[Uu]nity")

		set (
			UnityGame_FOUND
			true
			CACHE
			BOOL
			""
		)

	else ()

		set (
			UnityGame_FOUND
			false
			CACHE
			BOOL
			""
		)

	endif ()

	# If we found Unity, go generate the version string
	if (UnityGame_FOUND)

		string(
			REPLACE
			"."
			";"
			VERSION_LIST
			${_UNITY_GAME_VERSION_NUM}
		)

		list (
		  GET
		  VERSION_LIST
		  0
		  UnityGame_VERSION_MAJOR
		)

		list (
		  GET
		  VERSION_LIST
		  1
		  UnityGame_VERSION_MINOR
		)

		unset (
			_UNITY_GAME_VERSION_NUM
		)

		unset (
			VERSION_LIST
		)

		set (
			UnityGame_VERSION_STRING
			${UnityGame_VERSION_MAJOR}.${UnityGame_VERSION_MINOR}
			CACHE
			STRING
			"The version of unity installed on the system"
		)

		if(UnityGame_FIND_VERSION_EXACT)

			if (NOT ${UnityGame_VERSION_STRING} VERSION_EQUAL ${UnityGame_FIND_VERSION})

				set (
					UnityGame_FOUND
					false
					CACHE
					BOOL
					""
					FORCE
				)

				message (
					STATUS
					"Requested Unity version ${UnityGame_FIND_VERSION} but found ${UnityGame_VERSION_STRING}"
				)

			else ()
			
				set (
					UnityGame_FOUND
					true
					CACHE
					BOOL
					""
					FORCE
				)

			endif ()

		endif ()

		if (${UnityGame_VERSION_STRING} VERSION_LESS ${UnityGame_FIND_VERSION})

			set (
				UnityGame_FOUND
				false
				CACHE
				BOOL
				""
				FORCE
			)

			message (
				STATUS
				"Requested Unity version ${UnityGame_FIND_VERSION} but found ${UnityGame_VERSION_STRING}"
			)

		endif ()

	endif ()

	if (UnityGame_FOUND)

	message (
		STATUS
		"Found unity : ${UnityGame_EXECUTABLE}"
	)

	message (
		STATUS
		"Unity version : ${UnityGame_VERSION_STRING}"
	)	

	endif ()

endif ()

else ()

message (
	FATAL_ERROR
	"Only Windows 64-Bit supported!"
)

endif ()