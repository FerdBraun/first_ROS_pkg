cmake_minimum_required(VERSION 3.5)
project(costmaps_only)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Поиск пакетов
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)

# Создаём исполняемый файл
add_executable(costmaps_server_node src/main.cpp src/costmaps_only.cpp)

# Подключаем заголовочные файлы
target_include_directories(costmaps_server_node PRIVATE
  include
  ${nav2_util_INCLUDE_DIRS}
  ${nav2_costmap_2d_INCLUDE_DIRS}
)

# Линковка библиотек
target_link_libraries(costmaps_server_node
  ${rclcpp_LDFLAGS}
  ${rclcpp_lifecycle_LDFLAGS}
  ${tf2_ros_LDFLAGS}
  ${nav2_util_LIBRARIES}
  ${nav2_costmap_2d_LIBRARIES}
)

# Установка исполняемого файла
install(TARGETS costmaps_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка launch-файлов
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Установка конфигурационных файлов
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()