cmake_minimum_required(VERSION 3.16)
project(qt_basics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)          # 自动扫描 Q_OBJECT 并生成 moc 代码

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(hello    01_hello.cpp)
add_executable(button   02_button.cpp)
add_executable(clock    03_clock.cpp)
add_executable(worker   04_worker.cpp)
add_executable(qprocess 05_qprocess.cpp)

target_link_libraries(hello    PRIVATE Qt5::Widgets)
target_link_libraries(button   PRIVATE Qt5::Widgets)
target_link_libraries(clock    PRIVATE Qt5::Widgets)
target_link_libraries(worker   PRIVATE Qt5::Widgets)
target_link_libraries(qprocess PRIVATE Qt5::Widgets)
