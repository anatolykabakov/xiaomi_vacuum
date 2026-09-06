cmake_minimum_required(VERSION 3.16)
project(updater_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_executable(
  updater_example
  src/main.cpp src/logger.cpp src/udp_listener.cpp src/ota_controller.cpp
  src/updater_app.cpp
)

target_include_directories(
  updater_example
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(Threads REQUIRED)
target_link_libraries(
  updater_example
  PRIVATE Threads::Threads
)
