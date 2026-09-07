# Контекст build (x86_64): штатный GCC из образа (Trusty → /usr/bin/gcc ≈ 4.8).
# Conan берёт компилятор из этого профиля, а не «сам выбирает системный» без профиля.
#
# Если Conan/protobuf упираются в C++14+, варианты: отдельный компилятор (например gcc-5 в Dockerfile),
# другой базовый образ, или другая версия protobuf в conanfile.py.
[settings]
os=Linux
arch=x86_64
compiler=gcc
compiler.version=4.8
compiler.libcxx=libstdc++
compiler.cppstd=gnu11
build_type=Release

[conf]
tools.build:compiler_executables={"c":"/usr/bin/gcc","cpp":"/usr/bin/g++"}
