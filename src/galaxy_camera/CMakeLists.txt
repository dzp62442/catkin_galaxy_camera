cmake_minimum_required(VERSION 3.5)
project(galaxy_camera_pkg)

set(CMAKE_CXX_STANDARD 11)

# 解决ros找不到opencv的问题：
# 先在终端locate OpenCVConfig.cmake的位置，再set OpenCV_DIR变量
SET(OpenCV_DIR /usr/local/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    image_transport
    cv_bridge  # 注意添加依赖
    camera_info_manager
    OpenCV REQUIRED
    )

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include/
    ./
)

add_executable(galaxy_camera_exe src/galaxy_camera.cpp src/galaxy_camera_node.cpp)
target_link_libraries(galaxy_camera_exe gxiapi ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${OpenCV_LIBS})


