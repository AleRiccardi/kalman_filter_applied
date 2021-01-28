#!/usr/bin/env bash
set -e

build_path=$(catkin locate -b -e kfa)
echo "Reading compile_commands.json from $build_path"

echo "Running clang-tidy on all project sources"
run-clang-tidy -p $build_path -j$(nproc --all)
