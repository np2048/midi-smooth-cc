#!/bin/bash
set -e

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/.lv2 -DBUILD_TESTS=OFF
cmake --build build -j$(nproc)
cmake --install build
