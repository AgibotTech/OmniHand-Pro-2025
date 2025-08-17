# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand Pro 2025 SDK is licensed under Mulan PSL v2


set(CUR_TARGET_NAME "agibot_cpp_example")

add_executable(${CUR_TARGET_NAME} main.cc)

set(PYTHON_PKG_DIR ${CMAKE_BINARY_DIR}/agibot_hand_pkg)

target_include_directories(
  ${CUR_TARGET_NAME}
  PRIVATE ${CMAKE_SOURCE_DIR}/src
          ${CMAKE_SOURCE_DIR}/src/can_bus_device
          ${USER_PATH})

target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE omniHandPro25Can)

