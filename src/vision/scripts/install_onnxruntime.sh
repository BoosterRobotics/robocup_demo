#!/bin/bash

# Quick install ONNX Runtime C++ library

set -e

ONNX_VERSION="1.21.0"
ARCH=$(uname -m)

echo "Detected system architecture: $ARCH"

if [ "$ARCH" = "x86_64" ]; then
    PACKAGE="onnxruntime-linux-x64-${ONNX_VERSION}"
elif [ "$ARCH" = "aarch64" ]; then
    PACKAGE="onnxruntime-linux-aarch64-${ONNX_VERSION}"
else
    echo "Error: Unsupported architecture $ARCH"
    exit 1
fi

URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/${PACKAGE}.tgz"

echo "Downloading ONNX Runtime ${ONNX_VERSION}..."
echo "Download URL: $URL"
echo ""

cd /tmp
wget -q --show-progress "$URL" || {
    echo "Download failed! Please check your network connection or download manually:" 
    echo "$URL"
    exit 1
}

echo "Extracting..."
tar -xzf "${PACKAGE}.tgz"

echo "Installing to /usr/local/ ..."
cd "$PACKAGE"
sudo cp -r include/* /usr/local/include/
sudo cp -r lib/* /usr/local/lib/
sudo ldconfig

echo ""
echo "✓ Installation complete!"
echo ""
echo "Verify:"
ls -lh /usr/local/include/onnxruntime_cxx_api.h
ls -lh /usr/local/lib/libonnxruntime.so*

echo ""
echo "You can now build the vision package:"
echo "  cd /home/hicc/Workspace/booster_soccer"
echo "  colcon build --packages-select vision --cmake-args -DNO_CUDA=ON"
