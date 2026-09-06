[settings]
os=Linux
arch=armv7
compiler=gcc
compiler.version=4.8
compiler.cppstd=gnu11
compiler.libcxx=libstdc++11
build_type=Release

[conf]
# Absolute path via profile_dir so dependency builds (e.g. gtest in .conan2) still find the file.
tools.cmake.cmaketoolchain:user_toolchain={{ [os.path.normpath(os.path.join(profile_dir, "..", "..", "cmake", "toolchain-armv7.cmake"))] }}
tools.build:compiler_executables={"c":"arm-linux-gnueabihf-gcc","cpp":"arm-linux-gnueabihf-g++"}
