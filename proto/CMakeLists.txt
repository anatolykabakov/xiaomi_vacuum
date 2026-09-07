cmake_minimum_required(VERSION 3.16)

find_package(protobuf REQUIRED)
find_package(protobuf_BUILD REQUIRED)

set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${PROTO_GEN_DIR}")

file(GLOB_RECURSE PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
set(PROTOBUF_GENERATE_CPP_APPEND_PATH ON)
protobuf_generate_cpp(XIAOMI_PROTO_SRCS XIAOMI_PROTO_HDRS ${PROTO_FILES})

add_library(xiaomi_proto STATIC ${XIAOMI_PROTO_SRCS} ${XIAOMI_PROTO_HDRS})

target_include_directories(
  xiaomi_proto
  PUBLIC "${CMAKE_CURRENT_BINARY_DIR}"
         "${PROTO_GEN_DIR}"
)

target_link_libraries(
  xiaomi_proto
  PUBLIC protobuf::protobuf
)
