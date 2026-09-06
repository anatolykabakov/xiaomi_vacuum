find_package(Eigen3 CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp")

add_executable(
  xiaomi_robot_test
  main.cpp ${TEST_SOURCES}
)

target_include_directories(
  xiaomi_robot_test
  PRIVATE ${CMAKE_SOURCE_DIR}/cpp/app/include
)

target_link_libraries(
  xiaomi_robot_test
  PRIVATE
    xiaomi_robot_algo
    GTest::gtest
    Eigen3::Eigen
)

# GTest >=1.14 headers need C++14 (transparent functors in gtest).
set_target_properties(
  xiaomi_robot_test
  PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

add_test(
  NAME xiaomi_robot_test
  COMMAND xiaomi_robot_test
)
