cmake_minimum_required(VERSION 3.5.0)
project(hairo_world_plugin_joy)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

set(CHOREONOID_SKIP_QT_CONFIG true)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  choreonoid
)

catkin_package()

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++11)
endif()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(jackal_joy_node src/jackal_joy_node.cpp)
set_target_properties(jackal_joy_node PROPERTIES OUTPUT_NAME jackal_joy_node PREFIX "")
target_link_libraries(jackal_joy_node Choreonoid::CnoidUtil ${catkin_LIBRARIES}
  )

install(TARGETS jackal_joy_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

add_executable(husky_joy_node src/husky_joy_node.cpp)
set_target_properties(husky_joy_node PROPERTIES OUTPUT_NAME husky_joy_node PREFIX "")
target_link_libraries(husky_joy_node Choreonoid::CnoidUtil ${catkin_LIBRARIES}
  )

 install(TARGETS husky_joy_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
