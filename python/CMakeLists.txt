# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand Pro 2025 SDK is licensed under Mulan PSL v2


set(CUR_TARGET_NAME "agibot_hand_core")

set(PYTHON_PKG_DIR ${CMAKE_BINARY_DIR}/agibot_hand_pkg)

pybind11_add_module(${CUR_TARGET_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/binding.cc)

target_include_directories(
  ${CUR_TARGET_NAME}
  PRIVATE ${CMAKE_SOURCE_DIR}/src
          ${CMAKE_SOURCE_DIR}/src/can_bus_device
          ${USER_PATH})

target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE omniHandPro25Can)

set_target_properties(${CUR_TARGET_NAME} PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
    LIBRARY_OUTPUT_DIRECTORY ${PYTHON_PKG_DIR}/agibot_hand
)

add_custom_command(
  TARGET ${CUR_TARGET_NAME}
  PRE_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PYTHON_PKG_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PYTHON_PKG_DIR}/agibot_hand)


add_custom_command(
  TARGET ${CUR_TARGET_NAME}
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:omniHandPro25Can> ${PYTHON_PKG_DIR}/agibot_hand/
  COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/thirdParty/usbcanfd_libusb_x64_1.0.10_250328/libusbcanfd.so.1.0.10 ${PYTHON_PKG_DIR}/agibot_hand/
  COMMAND ${CMAKE_COMMAND} -E create_symlink libusbcanfd.so.1.0.10 ${PYTHON_PKG_DIR}/agibot_hand/libusbcanfd.so.1
  COMMAND ${CMAKE_COMMAND} -E create_symlink libusbcanfd.so.1.0.10 ${PYTHON_PKG_DIR}/agibot_hand/libusbcanfd.so
)

add_custom_target(
  create_python_pkg ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PYTHON_PKG_DIR}/agibot_hand
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/agibot_hand ${PYTHON_PKG_DIR}/agibot_hand
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/setup.py ${CMAKE_CURRENT_SOURCE_DIR}/pyproject.toml ${CMAKE_CURRENT_SOURCE_DIR}/MANIFEST.in ${PYTHON_PKG_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/VERSION ${PYTHON_PKG_DIR}/VERSION
  COMMAND ${Python3_EXECUTABLE} -m build --wheel
  WORKING_DIRECTORY ${PYTHON_PKG_DIR}
  DEPENDS ${CUR_TARGET_NAME})
