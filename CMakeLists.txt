cmake_minimum_required(VERSION 2.8.3)
project(test_self_service)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(single_thread src/single_thread.cpp)
target_link_libraries(single_thread ${catkin_LIBRARIES})
add_executable(call_from_outside src/call_from_outside)
target_link_libraries(call_from_outside ${catkin_LIBRARIES})

install(TARGETS single thread call_from_outside
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
