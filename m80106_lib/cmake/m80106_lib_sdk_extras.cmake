# m80106_lib_sdk_extras.cmake
#
# Creates the 'unitree_motor_sdk' IMPORTED target at find_package(m80106_lib)
# time, pointing to the bundled SDK shared object installed alongside this
# package.
#
# This file is included automatically by the generated m80106_libConfig.cmake.
# It is NOT meant to be included directly by user code.
#
# Install layout (per-package colcon space without --merge-install):
#   /install/m80106_lib/
#     lib/                              ← .so lives here
#     share/m80106_lib/cmake/           ← this file lives here
#
# Three directory levels up from CMAKE_CURRENT_LIST_DIR reaches the package root.

get_filename_component(_m80106_lib_pkg_root
    "${CMAKE_CURRENT_LIST_DIR}/../../.."
    ABSOLUTE)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(_m80106_sdk_so
        "${_m80106_lib_pkg_root}/lib/libUnitreeMotorSDK_M80106_Arm64.so")
else()
    set(_m80106_sdk_so
        "${_m80106_lib_pkg_root}/lib/libUnitreeMotorSDK_M80106_Linux64.so")
endif()

if(NOT TARGET unitree_motor_sdk)
    add_library(unitree_motor_sdk SHARED IMPORTED GLOBAL)
    set_target_properties(unitree_motor_sdk PROPERTIES
        IMPORTED_LOCATION "${_m80106_sdk_so}"
    )
endif()

unset(_m80106_lib_pkg_root)
unset(_m80106_sdk_so)
