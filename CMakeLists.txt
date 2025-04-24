cmake_minimum_required(VERSION 3.14)
project(drone_middleware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

########################  Dépendances ROS / externes  ########################
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(asio CONFIG REQUIRED)          # Asio header-only (Debian pkg: libasio-dev)

##############################################################################
#                       INCLUDES & DEFINITIONS                               #
##############################################################################
include_directories(
  include
)

add_compile_definitions(ASIO_STANDALONE)    # pour Asio non-Boost

##############################################################################
#                              LIBS CORE                                     #
##############################################################################
add_library(dmw_core
    src/core/morton.cpp
    src/core/packet_builder.cpp
    src/core/voxel_buffer.cpp
)
target_include_directories(dmw_core PUBLIC include)
target_link_libraries(dmw_core PUBLIC)

##############################################################################
#                         TRANSPORT – UDP backend                            #
##############################################################################
add_library(dmw_udp
    src/transport/udp/udp_transport.cpp
)
target_link_libraries(dmw_udp PUBLIC dmw_core asio::asio)

##############################################################################
#                       TRANSPORT – factory + url                            #
##############################################################################
add_library(dmw_transport
    src/transport/url.cpp
    src/transport/factory.cpp
)
target_link_libraries(dmw_transport PUBLIC dmw_core)

##############################################################################
#                                ROS layer                                   #
##############################################################################
add_library(dmw_ros
    src/ros/octomap_sub.cpp
)
target_link_libraries(dmw_ros
    PUBLIC dmw_core rclcpp visualization_msgs::visualization_msgs
)

##############################################################################
#                               EXECUTABLE                                   #
##############################################################################
add_executable(dmw_node src/main.cpp)
target_link_libraries(dmw_node
    PUBLIC dmw_core dmw_udp dmw_transport dmw_ros rclcpp
)

install(TARGETS dmw_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

##############################################################################
#                        COLCON / ament export                               #
##############################################################################
ament_export_include_directories(include)
ament_export_libraries(dmw_core dmw_udp dmw_transport dmw_ros)
ament_package()
