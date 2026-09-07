cmake_minimum_required(VERSION 3.16)
project(watchdog LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_executable(
  watchdog
  src/main.cpp src/config.cpp src/supervisor.cpp
)

target_include_directories(
  watchdog
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
