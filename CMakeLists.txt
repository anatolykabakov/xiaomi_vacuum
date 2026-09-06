cmake_minimum_required(VERSION 3.19)
project(xiaomi_robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTING "Build tests" OFF)
option(BUILD_PYBIND "Build pyxiaomi_robot Python module" OFF)
option(
  XIAOMI_ROBOT_ENABLE_DRIVERS
  "Build Player drivers library and enable PlayerNode" OFF
)

option(BUILD_TOOLS "Build host tools (bag_record)" ON)

add_subdirectory(proto)
add_subdirectory(cpp)
if(BUILD_TOOLS)
  add_subdirectory(tools)
endif()

install(
  EXPORT XiaomiRobotTargets
  NAMESPACE xiaomi_robot::
  DESTINATION lib/cmake/${PROJECT_NAME}
)
