cmake_minimum_required(VERSION 3.11)
project(m80106_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── ROS 2 / workspace dependencies ─────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(m80106_lib  REQUIRED)
find_package(serial      REQUIRED)

# ─── FTXUI — modern C++ TUI library (fetched at build time) ─────────────────
# No system package needed; FetchContent downloads and builds FTXUI from source.
include(FetchContent)
FetchContent_Declare(ftxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI
    GIT_TAG        v5.0.0
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
set(FTXUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(ftxui)

# ─── m80106_ui_node ──────────────────────────────────────────────────────────
add_executable(m80106_ui_node
    src/m80106_ui_node.cpp
    src/tui.cpp
)

target_include_directories(m80106_ui_node
    PRIVATE
        include
)

target_link_libraries(m80106_ui_node
    m80106_lib::m80106_lib
    ftxui::component
    ftxui::dom
    ftxui::screen
)

ament_target_dependencies(m80106_ui_node
    rclcpp
    serial
)

target_compile_features(m80106_ui_node PRIVATE cxx_std_17)

# ─── m80106_change_id ────────────────────────────────────────────────────────
add_executable(m80106_change_id
    src/change_id_node.cpp
)

target_include_directories(m80106_change_id
    PRIVATE
        include
)

target_link_libraries(m80106_change_id
    m80106_lib::m80106_lib
)

ament_target_dependencies(m80106_change_id
    rclcpp
    serial
)

target_compile_features(m80106_change_id PRIVATE cxx_std_17)

# ─── Install ─────────────────────────────────────────────────────────────────
install(TARGETS
    m80106_ui_node
    m80106_change_id
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
