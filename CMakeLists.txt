cmake_minimum_required(VERSION 3.16)
project(omni_hand_pro_2025 CXX)

# Set cmake path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Some option
option(BUILD_PYTHON_BINDING "Build Python binding" ON)
option(BUILD_CPP_EXAMPLES "Build CPP examples" ON)

# Some necessary settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Master project configuration
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

  set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/_deps)

  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_C_VISIBILITY_PRESET hidden)
  set(CMAKE_C_VISIBILITY_INLINES_HIDDEN ON)
  set(CMAKE_CXX_VISIBILITY_PRESET hidden)
  set(CMAKE_CXX_VISIBILITY_INLINES_HIDDEN ON)

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    if(AIMRT_MUJOCO_SIM_BUILD_TESTS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
    endif()
  endif()

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GetPybind11)

# find python dependencies
if(BUILD_PYTHON_BINDING)
  find_package(
    Python3
    COMPONENTS Interpreter Development
    REQUIRED)
  find_package(pybind11 QUIET)
  if(NOT pybind11_FOUND)
    message(STATUS "System pybind11 not found, will download from git...")
    include(GetPybind11)
  endif()

  include(CheckPythonPackage)
  check_python_package(build BUILD_FOUND)
  check_python_package(setuptools SETUPTOOLS_FOUND)
  check_python_package(wheel WHEEL_FOUND)

  if(NOT BUILD_FOUND
     OR NOT SETUPTOOLS_FOUND
     OR NOT WHEEL_FOUND)
    message(WARNING "Can not find build, setuptools or wheel in your python environment, will not generate aimrt_py package! ")
    message(WARNING "Try to install build, setuptools and wheel by `pip3 install build setuptools wheel --upgrade`")
  endif()
endif()

add_subdirectory(src)

if(BUILD_PYTHON_BINDING)
  add_subdirectory(python)
endif()

if(BUILD_CPP_EXAMPLES)
  add_subdirectory(examples)
endif()
