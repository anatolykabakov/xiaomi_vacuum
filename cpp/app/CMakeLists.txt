find_package(jsoncpp CONFIG REQUIRED)
find_package(tclap CONFIG REQUIRED)
find_package(cppzmq CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

file(
  GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
list(FILTER APP_SOURCES EXCLUDE REGEX ".*/algo/occupacy_mapping\\.cpp$")
list(FILTER APP_SOURCES EXCLUDE REGEX ".*/drivers/.*\\.cpp$")

if(NOT XIAOMI_ROBOT_ENABLE_DRIVERS)
  list(FILTER APP_SOURCES EXCLUDE REGEX ".*/nodes/player_node\\.cpp$")
endif()

if(XIAOMI_ROBOT_ENABLE_DRIVERS)
  add_subdirectory(src/drivers)
endif()

add_library(xiaomi_robot_algo STATIC src/algo/occupacy_mapping.cpp)
set_target_properties(
  xiaomi_robot_algo
  PROPERTIES POSITION_INDEPENDENT_CODE ON
)
target_include_directories(
  xiaomi_robot_algo
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  xiaomi_robot_algo
  PUBLIC Eigen3::Eigen
)

add_executable(
  xiaomi_robot
  main.cpp ${APP_SOURCES}
)

target_link_libraries(
  xiaomi_robot
  PRIVATE
    xiaomi_robot_algo
    JsonCpp::JsonCpp
    tclap::tclap
    cppzmq
    middleware
    xiaomi_proto
    Eigen3::Eigen
    spdlog::spdlog
)

if(XIAOMI_ROBOT_ENABLE_DRIVERS)
  add_dependencies(xiaomi_robot player-lib)
  target_link_libraries(
    xiaomi_robot
    PRIVATE xiaomi_robot_drivers
  )
  target_compile_definitions(
    xiaomi_robot
    PRIVATE XIAOMI_ROBOT_ENABLE_DRIVERS
  )
endif()

target_include_directories(
  xiaomi_robot
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
)

target_compile_definitions(
  xiaomi_robot
  PRIVATE ${XIAOMI_ROBOT_DEFINES}
)

target_compile_options(
  xiaomi_robot
  PRIVATE -g
          -fno-omit-frame-pointer
)
target_link_options(xiaomi_robot PRIVATE -rdynamic)

set_target_properties(
  xiaomi_robot
  PROPERTIES OUTPUT_NAME xiaomi_robot
)

install(
  TARGETS xiaomi_robot
  EXPORT XiaomiRobotTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.hpp|*.h"
)
