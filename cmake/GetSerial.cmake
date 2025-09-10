# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get serial ...")

set(serial_DOWNLOAD_URL
    "https://github.com/wjwwood/serial/archive/refs/tags/1.2.1.tar.gz"
    CACHE STRING "")

if(serial_LOCAL_SOURCE)
  FetchContent_Declare(
    serial
    SOURCE_DIR ${serial_LOCAL_SOURCE}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    serial
    URL ${serial_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_serial)
  FetchContent_GetProperties(serial)
  if(NOT serial_POPULATED)
    FetchContent_Populate(serial)
    file(READ ${serial_SOURCE_DIR}/CMakeLists.txt SERIAL_TMP_VAR)

    message("==>${serial_SOURCE_DIR}/CMakeLists.txt")

    # 1. 注释掉原来的 find_package
    string(REPLACE "find_package(catkin REQUIRED)" "# find_package(catkin REQUIRED)" SERIAL_TMP_VAR "${SERIAL_TMP_VAR}")

    # 2. 在注释的 find_package 后面添加新的设置
    string(
      REPLACE
        "# find_package(catkin REQUIRED)\n"
        "# find_package(catkin REQUIRED)\nset(CMAKE_CXX_STANDARD 11)\nset(CMAKE_CXX_STANDARD_REQUIRED True)\nset(TARGET_NAME \${PROJECT_NAME})\nset(rt_LIBRARIES rt)\nset(pthread_LIBRARIES pthread)\n"
        SERIAL_TMP_VAR
        "${SERIAL_TMP_VAR}")

    # 3. 注释掉 catkin_package 相关内容
    string(REPLACE "catkin_package(" "# catkin_package(" SERIAL_TMP_VAR "${SERIAL_TMP_VAR}")

    string(REPLACE "LIBRARIES \${PROJECT_NAME}" "    # LIBRARIES \${PROJECT_NAME}" SERIAL_TMP_VAR "${SERIAL_TMP_VAR}")

    string(REPLACE "DEPENDS rt pthread\n    )" "    # DEPENDS rt pthread\n    #)" SERIAL_TMP_VAR "${SERIAL_TMP_VAR}")

    string(REPLACE "INCLUDE_DIRS include\n    )" "    # INCLUDE_DIRS include\n    #)" SERIAL_TMP_VAR "${SERIAL_TMP_VAR}")
    string(REPLACE "INCLUDE_DIRS include" "    # INCLUDE_DIRS include" SERIAL_TMP_VAR "${SERIAL_TMP_VAR}")

    # 写入修改后的内容
    file(WRITE ${serial_SOURCE_DIR}/CMakeLists.txt "${SERIAL_TMP_VAR}")

    add_subdirectory(${serial_SOURCE_DIR} ${serial_BINARY_DIR})
  endif()
endfunction()

get_serial()
