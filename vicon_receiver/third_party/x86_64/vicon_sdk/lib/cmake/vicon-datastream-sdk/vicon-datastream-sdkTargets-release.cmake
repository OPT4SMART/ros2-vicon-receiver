#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vicon-datastream-sdk::ViconDataStreamSDK_CPP" for configuration "Release"
set_property(TARGET vicon-datastream-sdk::ViconDataStreamSDK_CPP APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vicon-datastream-sdk::ViconDataStreamSDK_CPP PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "Boost::system;Boost::thread;Boost::chrono"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libViconDataStreamSDK_CPP.so"
  IMPORTED_SONAME_RELEASE "libViconDataStreamSDK_CPP.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS vicon-datastream-sdk::ViconDataStreamSDK_CPP )
list(APPEND _IMPORT_CHECK_FILES_FOR_vicon-datastream-sdk::ViconDataStreamSDK_CPP "${_IMPORT_PREFIX}/lib/libViconDataStreamSDK_CPP.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
