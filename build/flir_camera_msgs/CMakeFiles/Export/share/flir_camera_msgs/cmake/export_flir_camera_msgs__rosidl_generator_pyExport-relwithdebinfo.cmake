#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "flir_camera_msgs::flir_camera_msgs__rosidl_generator_py" for configuration "RelWithDebInfo"
set_property(TARGET flir_camera_msgs::flir_camera_msgs__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(flir_camera_msgs::flir_camera_msgs__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libflir_camera_msgs__rosidl_generator_py.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libflir_camera_msgs__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS flir_camera_msgs::flir_camera_msgs__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_flir_camera_msgs::flir_camera_msgs__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libflir_camera_msgs__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)