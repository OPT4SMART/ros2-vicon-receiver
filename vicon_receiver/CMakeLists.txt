cmake_minimum_required(VERSION 3.8)
project(vicon_receiver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# detect arch
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(SYSTEM_ARCH aarch64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(SYSTEM_ARCH x86_64)
else()
  message(FATAL_ERROR "Unsupported arch: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

###########################################################
# --- Vicon DataStream SDK vendored inside this package ---
#   src/vicon_receiver/third_party/vicon_sdk/{include,lib}
set(VICON_SDK_ROOT         "${CMAKE_CURRENT_SOURCE_DIR}/third_party/${SYSTEM_ARCH}/vicon_sdk")
set(VICON_SDK_INCLUDE_DIR  "${VICON_SDK_ROOT}/include")
set(VICON_SDK_LIB          "${VICON_SDK_ROOT}/lib/libViconDataStreamSDK_CPP.so")

if(NOT EXISTS "${VICON_SDK_LIB}")
  message(FATAL_ERROR "Vicon SDK .so not found at: ${VICON_SDK_LIB}")
endif()

# Imported target with include dirs and location
add_library(vicon_datastream_sdk SHARED IMPORTED GLOBAL)
set_target_properties(vicon_datastream_sdk PROPERTIES
  IMPORTED_LOCATION "${VICON_SDK_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${VICON_SDK_INCLUDE_DIR};${VICON_SDK_INCLUDE_DIR}/vicon-datastream-sdk"
)
add_library(ViconDataStreamSDK::ViconDataStreamSDK_CPP ALIAS vicon_datastream_sdk)

###########################################################
# --- Vendored Boost 1.75 (no find_package) ---
set(BOOST_VENDOR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/${SYSTEM_ARCH}/boost")
set(BOOST_VENDOR_INCLUDE "${BOOST_VENDOR_ROOT}/include")
set(BOOST_VENDOR_LIBDIR "${BOOST_VENDOR_ROOT}/lib")

# Your filenames have '-mt-x64' and version suffixes; match them explicitly
# (Add more NAMES variants if your filenames differ)
find_library(BOOST_SYSTEM_LIB
  NAMES libboost_system-mt-x64.so.1.75.0 libboost_system.so.1.75.0 libboost_system
  PATHS "${BOOST_VENDOR_LIBDIR}" NO_DEFAULT_PATH REQUIRED)
find_library(BOOST_THREAD_LIB
  NAMES libboost_thread-mt-x64.so.1.75.0 libboost_thread.so.1.75.0 libboost_thread
  PATHS "${BOOST_VENDOR_LIBDIR}" NO_DEFAULT_PATH REQUIRED)
find_library(BOOST_CHRONO_LIB
  NAMES libboost_chrono-mt-x64.so.1.75.0 libboost_chrono.so.1.75.0 libboost_chrono
  PATHS "${BOOST_VENDOR_LIBDIR}" NO_DEFAULT_PATH REQUIRED)
find_library(BOOST_TIMER_LIB
  NAMES libboost_timer-mt-x64.so.1.75.0 libboost_timer.so.1.75.0 libboost_timer
  PATHS "${BOOST_VENDOR_LIBDIR}" NO_DEFAULT_PATH REQUIRED)

# Create IMPORTED targets that mimic the usual Boost::<comp> names
# (CMake allows creating imported targets with ::)
add_library(Boost::system SHARED IMPORTED)
set_target_properties(Boost::system PROPERTIES
  IMPORTED_LOCATION             "${BOOST_SYSTEM_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${BOOST_VENDOR_INCLUDE}")

add_library(Boost::thread SHARED IMPORTED)
set_target_properties(Boost::thread PROPERTIES
  IMPORTED_LOCATION             "${BOOST_THREAD_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${BOOST_VENDOR_INCLUDE}")

add_library(Boost::chrono SHARED IMPORTED)
set_target_properties(Boost::chrono PROPERTIES
  IMPORTED_LOCATION             "${BOOST_CHRONO_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${BOOST_VENDOR_INCLUDE}")

add_library(Boost::timer SHARED IMPORTED)
set_target_properties(Boost::timer PROPERTIES
  IMPORTED_LOCATION             "${BOOST_TIMER_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${BOOST_VENDOR_INCLUDE}")

# Threads are a transitive dependency of Boost::thread on Linux
find_package(Threads REQUIRED)
set_property(TARGET Boost::thread APPEND PROPERTY
  INTERFACE_LINK_LIBRARIES Threads::Threads)

###########################################################
# --- Node executable ---
add_executable(vicon_client
  src/communicator.cpp
  src/publisher.cpp
)

# Include our public headers when building this target
target_include_directories(vicon_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies (use imported targets consistently)
target_link_libraries(vicon_client
  ViconDataStreamSDK::ViconDataStreamSDK_CPP
  Boost::system Boost::thread Boost::chrono Boost::timer
)

ament_target_dependencies(vicon_client 
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Runtime search path so the binary finds libs in the install tree
set_target_properties(vicon_client PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib"
  BUILD_WITH_INSTALL_RPATH OFF
)

# --- Install: ship SDK + our headers and node ---
include(GNUInstallDirs)

# Install vendored Boost so your installed node finds the same libs at runtime
install(DIRECTORY "${BOOST_VENDOR_INCLUDE}/" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES
  "${BOOST_SYSTEM_LIB}" "${BOOST_THREAD_LIB}" "${BOOST_CHRONO_LIB}" "${BOOST_TIMER_LIB}"
  DESTINATION ${CMAKE_INSTALL_LIBDIR})

# install the SDK shared lib into install/lib
install(FILES "${VICON_SDK_LIB}" 
        DESTINATION ${CMAKE_INSTALL_LIBDIR})

# install SDK headers into install/include (keeps vendor tree)
install(DIRECTORY "${VICON_SDK_INCLUDE_DIR}/"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
# install our public headers (so other pkgs could include vicon_receiver/… if needed)
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# install the node and launch files
install(TARGETS vicon_client RUNTIME 
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
