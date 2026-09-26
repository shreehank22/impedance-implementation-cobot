#!/bin/bash

# Navigate to the third_party directory
cd ./src/third_party

# Remove build directories in each subfolder
for dir in */; do
    if [ -d "${dir}build" ]; then
        echo "Removing build directory in ${dir}"
        rm -rf "${dir}build"
    fi
done

echo "Completed removing build directories"
