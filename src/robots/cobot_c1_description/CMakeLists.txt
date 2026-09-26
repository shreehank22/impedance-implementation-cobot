cmake_minimum_required(VERSION 3.8)
project(cobot_c1_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(
  urdf
  xacro
)

ament_package()
