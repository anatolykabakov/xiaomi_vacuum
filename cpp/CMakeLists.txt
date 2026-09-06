add_subdirectory(mw)
add_subdirectory(app)

if(BUILD_PYBIND)
  add_subdirectory(pybind)
endif()

if(BUILD_TESTING)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()
