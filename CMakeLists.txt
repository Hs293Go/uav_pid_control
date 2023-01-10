cmake_minimum_required(VERSION 3.14.2)
project(uav_pid_control)
set(CMAKE_CXX_CLANG_TIDY "clang-tidy;--header-filter=^${CMAKE_SOURCE_DIR}")
set(CMAKE_CXX_CPPCHECK: "cppcheck;--inline-suppr")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
             geometry_msgs
             mavros_msgs
             roscpp
             tf2_eigen
             message_generation
             message_runtime)

find_package(Eigen3 REQUIRED)

add_message_files(FILES PositionTarget.msg PIDControlStatus.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  uav_pid_control
  CATKIN_DEPENDS
  roscpp
  std_msgs
  tf2_eigen
  geometry_msgs
  mavros_msgs
  # DEPENDS Eigen3
)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/uav_pid_control.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC include
                                                       ${catkin_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${catkin_LIBRARIES}
                                                  ${PROJECT_NAME})
