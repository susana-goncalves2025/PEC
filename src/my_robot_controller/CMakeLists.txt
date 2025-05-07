cmake_minimum_required(VERSION 3.5)
project(my_robot_controller)

# Dependências
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Geração de interfaces de serviços
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFloat64.srv"
)

ament_package()

