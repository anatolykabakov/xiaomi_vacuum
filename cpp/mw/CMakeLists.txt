# Middleware = исходного проекта manager (header-only), скопирован как есть проекта:
#   include/исходного проекта/middleware/manager.hpp
#   include/исходного проекта/utils/logger.h   (зависимость manager.hpp)
# Требует C++17.
add_library(middleware INTERFACE)
target_include_directories(
  middleware
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
)
target_compile_features(middleware INTERFACE cxx_std_17)
find_package(Threads REQUIRED)
target_link_libraries(middleware INTERFACE Threads::Threads)

install(
  TARGETS middleware
  EXPORT XiaomiRobotTargets
)
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
)
