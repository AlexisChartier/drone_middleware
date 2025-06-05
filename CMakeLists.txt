cmake_minimum_required(VERSION 3.16)
project(drone_middleware LANGUAGES CXX)

# ───────────────  options globales  ──────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_definitions(ASIO_STANDALONE)

# ───────────────  dépendances  ───────────────────────────────────
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(octomap       REQUIRED)       
find_package(octomap_msgs  REQUIRED)
find_package(ZLIB          REQUIRED)

find_path(ASIO_INCLUDE_DIR asio.hpp REQUIRED)
include_directories(include ${ASIO_INCLUDE_DIR} ${OCTOMAP_INCLUDE_DIRS})

# ───────────────  sources  ───────────────────────────────────────
set(SRC
  src/main.cpp
  src/core/binary_packet_builder.cpp
  src/core/udp_header.cpp
  src/transport/udp/udp_transport.cpp
  src/transport/udp/plugin.cpp
  src/transport/url.cpp
  src/transport/factory.cpp
  src/ros/octomap_sub.cpp
)

# ───────────────  exécutable  ────────────────────────────────────
add_executable(dmw_node ${SRC})

# *** on lie les bibliothèques "classiques" d'OctoMap ***
target_link_libraries(dmw_node
  ${OCTOMAP_LIBRARIES}      # fournit   octomap  octomath
  ZLIB::ZLIB
)

ament_target_dependencies(dmw_node
  rclcpp
  octomap_msgs              # messages ROS 2
)

# ───────────────  installation  ─────────────────────────────────
install(TARGETS dmw_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/launch
        OPTIONAL)

# indispensable : copie du package.xml pour qu’ament référence le paquet
install(FILES package.xml
        DESTINATION share/${PROJECT_NAME})

ament_package()
