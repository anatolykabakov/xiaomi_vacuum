find_package(cppzmq CONFIG REQUIRED)
find_package(tclap CONFIG REQUIRED)

# Запись ZMQ-топиков робота в бег (хостовый инструмент).
add_executable(bag_record bag_record.cpp)
target_link_libraries(bag_record PRIVATE cppzmq tclap::tclap xiaomi_proto)
set_target_properties(bag_record PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
