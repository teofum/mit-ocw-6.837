cmake_minimum_required(VERSION 3.27)
project(mit_ocw_6_837)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-framework GLUT -framework OpenGL -framework Cocoa -O2 -Wall -Wno-deprecated-declarations")

include_directories(lib/vecmath/include)

add_library(vecmath lib/vecmath/src/Matrix2f.cpp lib/vecmath/src/Matrix3f.cpp lib/vecmath/src/Matrix4f.cpp lib/vecmath/src/Quat4f.cpp lib/vecmath/src/util.cpp lib/vecmath/src/Vector2f.cpp lib/vecmath/src/Vector3f.cpp lib/vecmath/src/Vector4f.cpp)

add_executable(a0 assignments/a0/main.cpp assignments/a0/obj.cpp assignments/a0/simplify.cpp)
target_link_libraries(a0 vecmath)

add_executable(a1 assignments/a1/main.cpp assignments/a1/camera.cpp assignments/a1/curve.cpp assignments/a1/parse.cpp assignments/a1/surf.cpp)
target_link_libraries(a1 vecmath)
