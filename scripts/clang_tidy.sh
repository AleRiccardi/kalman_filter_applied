#!/usr/bin/env bash
set -e

build_path="$1"
echo "Reading compile_commands.json from $build_path"

echo "Running clang-tidy on all project sources"
run-clang-tidy -quiet -p $build_path -j$(nproc --all)
