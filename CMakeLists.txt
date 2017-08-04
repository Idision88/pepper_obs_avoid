cmake_minimum_required(VERSION 2.8.3)
project(pepper_obs_avoid)

add_definitions( -DCATKIN_BUILD )

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  naoqi_libqi
  naoqi_libqicore
  visp_bridge
)

find_package(VISP REQUIRED)
find_package(quadprog REQUIRED)
find_package(VISP_NAOQI REQUIRED )

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})


catkin_package(
  LIBRARIES
    pepper_obs_avoid
  
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf

  DEPENDS 
    VISP
    VISP_NAOQI
)

include_directories(include)

include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${visp_naoqi_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${QUADPROG_INCLUDE_DIRS}

)


# Declare a cpp library
add_library(pepper_obs_avoid
  include/pepper_obs_avoid.h
  src/pepper_obs_avoid.cpp
)

target_link_libraries(pepper_obs_avoid
  ${catkin_LIBRARIES}
  ${naoqi_libqi_LIBRARIES}
  ${VISP_LIBRARIES}
  ${visp_naoqi_LIBRARIES}
  ${QUADPROG_LIBRARIES}
)

## Declare a cpp executable
add_executable(pepper_obs_avoid_node src/main.cpp)

target_link_libraries(pepper_obs_avoid_node
  ${resource_retriever_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QUADPROG_INCLUDE_DIRS}
  pepper_obs_avoid
)

install(
  TARGETS
    pepper_obs_avoid
    pepper_obs_avoid_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

