cmake_minimum_required(VERSION 2.8.3)
project(lynxmotion_tm4c)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs trajectory_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tm4c_driver tm4c_driver
   CATKIN_DEPENDS roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(tm4c src/tm4c.cpp)
target_link_libraries(tm4c ${catkin_LIBRARIES})

add_library(tm4c_driver src/tm4c_driver.cpp)
target_link_libraries(tm4c_driver tm4c ${catkin_LIBRARIES})

add_executable(tm4c_node src/tm4c_node.cpp)
target_link_libraries(tm4c_node tm4c_driver ${catkin_LIBRARIES})

add_executable(test_tm4c src/test_tm4c.cpp)
target_link_libraries(test_tm4c tm4c_driver ${catkin_LIBRARIES})

install(TARGETS tm4c tm4c_driver tm4c_node test_tm4c
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
