#!/bin/bash

CLEAN=false
while getopts "c" opt; do
    case $opt in
        c)
            CLEAN=true
            ;;
    esac
done

if [ "$CLEAN" = true ]; then
    rm -rf build
    exit 0
fi

mkdir -p build

cd build

cmake ..

# Build the project
cmake --build .
