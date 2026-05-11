cmake_minimum_required(VERSION 3.10)
project(CameraProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Вказуємо шлях до OpenCV ВЕРЕДИНІ вашого проєкту
set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/opencv/build")

find_package(OpenCV REQUIRED)

file(GLOB SOURCES "src/*.cpp")
add_executable(my_camera_app ${SOURCES})
# Вказуємо MSVC вшивати рантайм С++ прямо в .exe файл (/MT замість /MD)
set_property(TARGET my_camera_app PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

target_include_directories(my_camera_app PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(my_camera_app ${OpenCV_LIBS} ws2_32)