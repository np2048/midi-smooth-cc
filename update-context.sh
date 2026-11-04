#!/usr/bin/env bash

# Update temporary files context for LLMs

mkdir -p tmp

echo "cat src/*" >tmp/code.py
cat src/* >>tmp/code.py
echo "---" >>tmp/code.py
echo "cat lv2/*" >>tmp/code.py
cat lv2/* >>tmp/code.py

echo "cat rpn_calc/tests/*" >tmp/tests.py
cat tests/* >>tmp/tests.py
