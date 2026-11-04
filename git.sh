#!/bin/bash
git add .
if [ $# -eq 0 ]; then
    git commit -m 'auto save'
else
    git commit -m "$*"
fi
git pull
git push
