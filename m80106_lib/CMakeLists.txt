cmake_minimum_required(VERSION 3.8)
project(m80106_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)

# ─── Architecture-conditional SDK shared object ───────────────────────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(M80106_SDK_SO
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/libUnitreeMotorSDK_M80106_Arm64.so")
else()
    set(M80106_SDK_SO
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/libUnitreeMotorSDK_M80106_Linux64.so")
endif()

# Build-tree IMPORTED target — resolved by the CONFIG_EXTRAS at install time.
add_library(unitree_motor_sdk SHARED IMPORTED GLOBAL)
set_target_properties(unitree_motor_sdk PROPERTIES
    IMPORTED_LOCATION "${M80106_SDK_SO}"
)

# ─── m80106_lib INTERFACE target (header-only) ───────────────────────────────
#
# Downstream packages do:
#   find_package(m80106_lib REQUIRED)
#   target_link_libraries(my_target m80106_lib::m80106_lib)
#
# This provides:
#   • include/ m80106_lib/  — user-facing headers (motor_types, motor_bus,
#                              motor_driver, control_modes)
#   • sdk_include/          — bundled Unitree SDK headers (SerialPort, IOPort,
#                              unitreeMotor, crc_ccitt)
#   • link to libUnitreeMotorSDK_M80106_*.so (bundled, no external download)
#
# Note: motor_bus.hpp uses <serial/serial.h>.  Downstream executables that
# call MotorBus must also find_package(serial) and link against serial.
# Declare that dependency in your package.xml as <depend>serial</depend>.

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk_include>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/m80106_lib_sdk>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
    # Build tree: use the IMPORTED target created above.
    # Install tree: CONFIG_EXTRAS re-creates unitree_motor_sdk before this
    #               target is imported, so the name resolves correctly.
    $<BUILD_INTERFACE:unitree_motor_sdk>
    $<INSTALL_INTERFACE:unitree_motor_sdk>
)

# ─── Installations ────────────────────────────────────────────────────────────
install(FILES "${M80106_SDK_SO}"
    DESTINATION lib
    PERMISSIONS
        OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ             GROUP_EXECUTE
        WORLD_READ             WORLD_EXECUTE
)

install(DIRECTORY include/m80106_lib
    DESTINATION include
)

install(DIRECTORY sdk_include/
    DESTINATION include/m80106_lib_sdk
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
)

install(EXPORT ${PROJECT_NAME}_targets
    FILE        ${PROJECT_NAME}Targets.cmake
    NAMESPACE   ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake
)

# ─── ament exports ────────────────────────────────────────────────────────────
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(serial)

# CONFIG_EXTRAS creates the 'unitree_motor_sdk' IMPORTED target when
# find_package(m80106_lib) is called by a downstream package.
ament_package(
    CONFIG_EXTRAS "cmake/m80106_lib_sdk_extras.cmake"
)
