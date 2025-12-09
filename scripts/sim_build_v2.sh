#!/bin/bash

cd `dirname $0`
cd ..

colcon build  --symlink-install --parallel-workers $(nproc) --packages-ignore vision $@