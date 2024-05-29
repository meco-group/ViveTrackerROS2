cmake_minimum_required(VERSION 3.5)
project(vivetrackertest)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)



# #######################
# ## Find CasADi
# #######################
# message("If you get a complaint about missing libcasadi.lib, please copy the casadi.lib file to libcasadi.lib")
# message("Please check that the CASADI_LIBRARY_DIRS variable in the CMakeLists.txt corresponds to the path of your CasADi installation")
# set(CASADI_LIBRARY_DIRS "/home/arena/miniconda3/lib/python3.11/site-packages/casadi")
# set(CMAKE_PREFIX_PATH ${CASADI_LIBRARY_DIRS} ${CMAKE_PREFIX_PATH})
# set(PKG_CONFIG_PATH ${CASADI_LIBRARY_DIRS}/pkgconfig ${PKG_CONFIG_PATH})
# find_package(casadi 
#   REQUIRED 
#   HINTS ${CASADI_LIBRARY_DIRS} ${CASADI_LIBRARY_DIRS}/lib
# )
# set(CASADI_INCLUDE_DIRS ${CASADI_LIBRARY_DIRS}/include)
# set(CASADI_LIBRARIES casadi)

# # # Find CasADi
# # # run `pkg-config --cflags --libs casadi` to get the flags
# # # set the CASADI_DIR to the directory where casadi is installed\
# # # e.g. /usr/local or /home/<username>/miniconda3/lib/python3.xx/site-packages/casadi
# # # set the CMAKE_PREFIX_PATH to the CASADI_DIR
# # # set the PKG_CONFIG_PATH to the CASADI_DIR/lib/pkgconfig
# # # find_package(PkgConfig REQUIRED)
# # # pkg_search_module(CASADI casadi REQUIRED)
# # # target_include_directories(<target> PUBLIC ${CASADI_INCLUDE_DIRS})
# # # target_link_directories(<target> PUBLIC ${CASADI_LIBRARY_DIRS})
# # # target_link_libraries(<target> ${CASADI_LIBRARIES})
# # set(CASADI_DIR "/usr/local")
# # set(CMAKE_PREFIX_PATH ${CASADI_DIR} ${CMAKE_PREFIX_PATH})
# # set(PKG_CONFIG_PATH ${CASADI_DIR}/lib/pkgconfig ${PKG_CONFIG_PATH})
# find_package(PkgConfig REQUIRED)
# # pkg_search_module(CASADI casadi REQUIRED)
# pkg_search_module(IPOPT ipopt REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)

include_directories(
  # ${CASADI_INCLUDE_DIRS}         # For CasADi
  # ${CASADI_INCLUDE_DIRS}/include # For IPOPT
  ${EIGEN3_INCLUDE_DIR}          # For Eigen
  ${CMAKE_SOURCE_DIR}/../libsurvive/include/libsurvive
  ${CMAKE_SOURCE_DIR}/../libsurvive/redist
)

add_executable(vive_tracker_node src/vive_tracker_node.cpp)
ament_target_dependencies(vive_tracker_node 
  rclcpp
  tf2
  # ${CASADI_LIBRARIES}
)
target_link_libraries(vive_tracker_node 
  # ${CASADI_LIBRARIES}
  # ${IPOPT_LIBRARIES}
  ${CMAKE_SOURCE_DIR}/../libsurvive/bin/libsurvive.so
)

target_include_directories(vive_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# target_link_libraries(vive_tracker_node PUBLIC ${CASADI_LIBRARIES} ${IPOPT_LIBRARIES})
# target_link_directories(vive_tracker_node PUBLIC ${CASADI_LIBRARY_DIRS} ${IPOPT_LIBRARY_DIRS})


install(TARGETS vive_tracker_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  ${CMAKE_SOURCE_DIR}/../libsurvive/bin/libsurvive.so.0
  ${CMAKE_SOURCE_DIR}/../libsurvive/bin/libsurvive.so.0.3
  ${CMAKE_SOURCE_DIR}/../libsurvive/bin/libsurvive.so
  DESTINATION lib)




ament_package()
