# Find functionality for the Unity Game Engine
#
# This currently only works for Unity.
# Operates by simply querying the Windows Registry (regedit.exe)
#
# Variables:
# 	- UnityGame_EXECUTABLE -> The main Unity executable
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

include (
	FindPackageHandleStandardArgs
)

# Search the windows registry for the installation of Unity

if (WIN32)

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
			UnityGame_VERSION
			${UnityGame_VERSION_MAJOR}.${UnityGame_VERSION_MINOR}
			CACHE
			STRING
			"The version of unity installed on the system"
		)

	endif ()

	# This function is very powerful.
	# One doesn't need to even check for validity
	# on the UnityGame_EXECUTABLE as this function
	# will handle the validity check
	find_package_handle_standard_args (
		UnityGame
		REQUIRED_VARS
			UnityGame_EXECUTABLE
		VERSION_VAR
			UnityGame_VERSION
		FAIL_MESSAGE
			"Could not find a suitable installation of Unity3D"
	)

else ()

	message (
		FATAL_ERROR
		"Only Windows 64-Bit supported!"
	)

endif ()