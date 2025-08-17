English | [中文](README.zh_CN.md)

# OmniHand Pro 2025 SDK

## Overview

OmniHand Pro 2025 is a 12-degree-of-freedom professional dexterous hand featuring precise operation and flexible control capabilities. It is equipped with tactile sensors and multiple control modes (position control, force control, hybrid control), making it suitable for a wide range of applications including research and education, entertainment and commercial performances, exhibition guidance, and industrial scenarios. To facilitate rapid development and application by users, we provide the accompanying OmniHand Pro 2025 SDK development package, which supports both Python and C++ API interfaces for implementing dexterous hand control and data acquisition functions.

![](document/pic/hand.jpg)

## Getting Started

### System Requirements

#### Hardware Requirements

- ZLG USBCANFD series (USBCANFD-100U-mini recommended)

#### Software Requirements

- Operating System: Ubuntu 22.04 (x86_64)
- Compiler: GCC 11.4 or higher
- Build Tools: CMake 3.16 or higher
- Python: 3.10 or higher

### Installation

You can choose between source code compilation installation or pre-compiled package installation.

#### Source Code Compilation Installation

Execute the following command in the project root directory:

```bash
./build.sh -DCMAKE_BUILD_TYPE=Release \
           -DCMAKE_INSTALL_PREFIX=./build/install \
           -DBUILD_PYTHON_BINDING=ON \
           -DBUILD_CPP_EXAMPLES=OFF \
```

#### Pre-compiled Package Installation

##### Python whl Package Installation

```bash
# Download the corresponding version of python whl package from GitHub
# Example: agibot_hand_py-1.0.0-cp310-cp310-linux_x86_64.whl
pip install agibot_hand_py-1.0.0-cp310-cp310-linux_x86_64.whl
```

## Dexterous Hand Motor Index

OmniHand Pro 2025 has 12 degrees of freedom, indexed from 1 to 12. The control motors corresponding to each index are shown in the figure below:

![](document/pic/hand_joints.jpg)

## Running Examples

```bash
cd python/example

python3 ./demo_gestures_ok.py
```

## Directory Structure

```bash
├── thirdParty              # Third-party dependency libraries
├── src                     # C++ core source code
│   ├── proto.h
│   ├── export_symbols.h
│   ├── CMakeLists.txt
│   ├── can_bus_device
│   ├── c_agibot_hand.h
│   └── c_agibot_hand.cc
├── scripts                 # Script tools directory
│   └── setup.sh
├── python                  # Python binding module (Python interface generated from C++ source code)
├── examples                # C++ example code
├── document                # Documentation directory
├── CMakeLists.txt          # Main CMake configuration file
├── cmake                   # CMake modules directory
└── build.sh                # Build script
```

## API Introduction

For detailed API usage instructions, please refer to the following links:

- [OmniHand Pro 2025 SDK C++ API Documentation](document/API_CPP.md)
- [OmniHand Pro 2025 SDK Python API Documentation](document/API_PYTHON.md)

## FAQ

### Q1: Unable to communicate with the hand when starting the program?

**A:** First, ensure that the driver is properly installed. For details, refer to the [ZLG Driver Installation Guide](https://manual.zlg.cn/web/#/42/1710:~:text=%23sudo%20chmod%20666%20/dev/bus/usb/xxx/yyy). Make sure the hand power is connected and the USB end is plugged into the computer, then execute the following commands:

```shell
lsusb

sudo chmod 666 /dev/bus/usb/xxx/yyy
# Replace `xxx/yyy` with the actual USB device path shown by the `lsusb` command.
```

## Copyright

Copyright (c) 2025 Agibot Co., Ltd. OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

---

_Document Version: v1.0.0_  
_Last Updated: 2025-8_
