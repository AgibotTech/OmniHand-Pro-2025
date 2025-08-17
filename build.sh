#!/bin/bash

# exit on error and print each command
set -e

if [ -d ./build/install ]; then
    rm -rf ./build/install
fi

cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    -DBUILD_PYTHON_BINDING=ON \
    -DBUILD_CPP_EXAMPLES=ON \
    $@

cmake --build build --config Release --target install --parallel $(nproc)
