from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout


class XiaomiRobotConan(ConanFile):
    name = "xiaomi_robot"
    version = "0.1"
    package_type = "application"

    settings = "os", "arch", "compiler", "build_type"

    options = {
        "build_testing": [True, False],
        "build_pybind": [True, False],
    }
    default_options = {
        "build_testing": False,
        "build_pybind": False,
    }

    def requirements(self):
        self.requires("jsoncpp/1.9.6")
        self.requires("tclap/1.2.5")
        self.requires("protobuf/3.21.12")
        self.requires("cppzmq/4.10.0")
        self.requires("eigen/3.4.0")
        # 1.15 pulls fmt/11.x (needs C++14 + GCC≥5); Trusty cross-gcc is 4.8.
        self.requires("spdlog/1.9.2")
        if self.options.build_testing:
            self.test_requires("gtest/1.10.0")
        if self.options.build_pybind:
            self.requires("pybind11/2.13.6")

    def build_requirements(self):
        self.tool_requires("protobuf/3.21.12")
        if self.options.build_testing:
            self.tool_requires("gtest/1.10.0")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.cache_variables["BUILD_TESTING"] = self.options.build_testing
        tc.cache_variables["BUILD_PYBIND"] = self.options.build_pybind
        tc.generate()

        deps = CMakeDeps(self)
        deps.build_context_activated = ["protobuf"]
        deps.build_context_suffix = {"protobuf": "_BUILD"}
        deps.generate()
