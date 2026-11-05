#!/bin/bash
set -e # Exit immediately if any command fails

echo "=== [1/3] Configuring CMake ==="
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

echo "=== [2/3] Building Project and Tests ==="
# This actually compiles the C++ files, including the new integration tests
cmake --build build -j"$(nproc)"

echo "=== [3/3] Running Tests ==="
# Now that the executable exists and is up-to-date, ctest will run properly
ctest --test-dir build --output-on-failure

echo "=== All steps completed successfully ==="
