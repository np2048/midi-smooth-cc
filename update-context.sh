#!/usr/bin/env bash

# Update temporary files context for LLMs

mkdir -p tmp

echo "cat src/*" >tmp/code.cpp
cat src/* >>tmp/code.cpp

echo "---" >>tmp/code.cpp
echo "cat lv2/*" >>tmp/code.cpp
cat lv2/* >>tmp/code.cpp

echo "---" >>tmp/code.cpp
echo "cat CMakeLists.txt" >>tmp/code.cpp
cat CMakeLists.txt >>tmp/code.cpp

echo "cat rpn_calc/tests/*" >tmp/tests.cpp
cat tests/* >>tmp/tests.cpp
