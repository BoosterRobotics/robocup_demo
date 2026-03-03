#!/bin/bash

cd `dirname $0`

./build.sh --cmake-args -DNO_CUDA=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-g -rdynamic"