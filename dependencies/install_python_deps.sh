#!/bin/bash
set -e

echo "Installing python dependencies for segment anything..."

pip install opencv-python pycocotools matplotlib onnxruntime onnx

echo "done installing python dependencies"