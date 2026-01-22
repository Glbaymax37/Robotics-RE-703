cmake_minimum_required(VERSION 3.8)
project(bt_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Load Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED) # Penting untuk mendapatkan path package
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Executable ---
add_executable(bt_main
  src/bt_main.cpp
)

ament_target_dependencies(bt_main
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  behaviortree_cpp_v3
  ament_index_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# --- Install Executable ---
install(TARGETS
  bt_main
  DESTINATION lib/${PROJECT_NAME}
)

# --- PERBAIKAN UTAMA DI SINI ---
# 1. Hapus tanda '/' di awal (agar relatif terhadap folder project)
# 2. Ganti nama folder jadi 'bt' (sesuai kode C++ Anda)
# 3. Install satu folder penuh agar isinya terbawa
install(DIRECTORY
  bt
  DESTINATION share/${PROJECT_NAME}
)
# -------------------------------

ament_package()
