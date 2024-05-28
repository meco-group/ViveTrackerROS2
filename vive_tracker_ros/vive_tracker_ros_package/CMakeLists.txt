cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_DOUBLE")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

# Find CasADi
set(CASADI_DIR "/usr/local")
set(CMAKE_PREFIX_PATH ${CASADI_DIR} ${CMAKE_PREFIX_PATH})
set(PKG_CONFIG_PATH ${CASADI_DIR}/lib/pkgconfig ${PKG_CONFIG_PATH})
find_package(PkgConfig REQUIRED)
pkg_search_module(CASADI casadi REQUIRED)
pkg_search_module(IPOPT ipopt REQUIRED)

link_directories(${CASADI_LIBRARY_DIRS})


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/../../libsurvive/include/libsurvive
${PROJECT_SOURCE_DIR}/../../libsurvive/redist
${CASADI_INCLUDEDIR}
)

set(LIBS
${PROJECT_SOURCE_DIR}/../../libsurvive/bin/libsurvive.so
)

# Node for vive tracker
rosbuild_add_executable(vive_tracker_ros_node
src/vive_tracker_node.cpp
)
target_link_libraries(vive_tracker_ros_node
${LIBS}
${CASADI_LIBRARIES}
ipopt
)
