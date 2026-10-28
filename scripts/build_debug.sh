#!/bin/bash

cd `dirname $0`
cd ..

export PYTHONWARNINGS="ignore::DeprecationWarning,ignore::UserWarning,ignore::FutureWarning"

colcon build  --symlink-install  --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-g -fno-omit-frame-pointer" "$@"

espeak "build complete" >/dev/null 2>&1 || echo "Build complete"