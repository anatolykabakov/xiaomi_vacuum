cmake_minimum_required(VERSION 3.14)
project(pyxiaomi_robot LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(${PROJECT_NAME} bindings.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE xiaomi_robot_algo
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_SOURCE_DIR}/app/include
)

set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION .
)
