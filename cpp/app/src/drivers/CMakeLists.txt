include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/import_player.cmake)

add_library(
  xiaomi_robot_drivers STATIC player_client_c.cpp player_client_cpp.cpp
)

set_target_properties(
  xiaomi_robot_drivers
  PROPERTIES POSITION_INDEPENDENT_CODE ON
)

target_include_directories(
  xiaomi_robot_drivers
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)

target_link_libraries(
  xiaomi_robot_drivers
  PUBLIC xiaomi_player::clients
         Eigen3::Eigen
)

add_dependencies(xiaomi_robot_drivers player-lib)
