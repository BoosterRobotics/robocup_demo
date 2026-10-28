#!/bin/bash

cd `dirname $0`
cd ..

colcon build  --symlink-install  --parallel-workers $(nproc) --packages-select brain "$@"
espeak "build complete" >/dev/null 2>&1 || echo "Build complete"