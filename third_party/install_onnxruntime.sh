#!/bin/bash

# Install ONNX Runtime C++ library quickly

cd /usr/local/booster_robot/third_party

echo "Extracting..."
tar -xzf "onnxruntime-linux-x64-1.23.2.tgz"

echo "Installing to /usr/local/ ..."
cd "onnxruntime-linux-x64-1.23.2"
sudo cp -r include/* /usr/local/include/
sudo cp -r lib/* /usr/local/lib/
sudo ldconfig

echo ""
echo "✓ Installation complete!"
echo ""
echo "Verification:"
ls -lh /usr/local/include/onnxruntime_cxx_api.h
ls -lh /usr/local/lib/libonnxruntime.so*
