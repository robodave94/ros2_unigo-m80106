cmake_minimum_required(VERSION 3.11)
project(m80106_execs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── ROS 2 / workspace dependencies ─────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(m80106_lib  REQUIRED)
find_package(serial      REQUIRED)

# ─── actuator_ping ───────────────────────────────────────────────────────────
add_executable(actuator_ping
    src/actuator_ping_node.cpp
)

target_link_libraries(actuator_ping
    m80106_lib::m80106_lib
)

ament_target_dependencies(actuator_ping
    rclcpp
    serial
)

target_compile_features(actuator_ping PRIVATE cxx_std_17)

# ─── scan_motor_info ─────────────────────────────────────────────────────────
add_executable(scan_motor_info
    src/scan_motor_information_node.cpp
)

target_link_libraries(scan_motor_info
    m80106_lib::m80106_lib
)

ament_target_dependencies(scan_motor_info
    rclcpp
    serial
)

target_compile_features(scan_motor_info PRIVATE cxx_std_17)

# ─── motor_operation ─────────────────────────────────────────────────────────
add_executable(motor_operation
    src/motor_operation_node.cpp
)

target_link_libraries(motor_operation
    m80106_lib::m80106_lib
)

ament_target_dependencies(motor_operation
    rclcpp
    serial
)

target_compile_features(motor_operation PRIVATE cxx_std_17)

# ─── dual_motor_operation ────────────────────────────────────────────────────
add_executable(dual_motor_operation
    src/dual_motor_operation_node.cpp
)

target_link_libraries(dual_motor_operation
    m80106_lib::m80106_lib
)

ament_target_dependencies(dual_motor_operation
    rclcpp
    serial
)

target_compile_features(dual_motor_operation PRIVATE cxx_std_17)

# ─── interface_testing ───────────────────────────────────────────────────────
add_executable(interface_testing
    src/interface_testing_node.cpp
)

target_link_libraries(interface_testing
    m80106_lib::m80106_lib
)

ament_target_dependencies(interface_testing
    rclcpp
    serial
)

target_compile_features(interface_testing PRIVATE cxx_std_17)

# ─── motion_data_collection ──────────────────────────────────────────────────
add_executable(motion_data_collection
    src/motion_data_collection_node.cpp
)

target_link_libraries(motion_data_collection
    m80106_lib::m80106_lib
)

ament_target_dependencies(motion_data_collection
    rclcpp
    serial
)

target_compile_features(motion_data_collection PRIVATE cxx_std_17)

# ─── Install ─────────────────────────────────────────────────────────────────
install(TARGETS
    actuator_ping
    scan_motor_info
    motor_operation
    dual_motor_operation
    interface_testing
    motion_data_collection
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
