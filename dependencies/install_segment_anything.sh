#!/bin/bash
set -e

echo "Installing python dependencies for segment anything..."
pip install opencv-python pycocotools matplotlib onnxruntime onnx
echo "done installing python dependencies"

echo "installing segment-anything..."
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR/segment-anything"
pip install -e .
cd $SCRIPT_DIR
echo "done installing segment-anything"

DATA_DIR="$SCRIPT_DIR/data"
echo "downloading segment-anything model checkpoint to: $DATA_DIR"
mkdir -p $DATA_DIR
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth -O $DATA_DIR/sam-model-checkpoint.pth
echo "done downloading segment-anything model checkpoint"

echo "MAKE SURE YOU HAVE TORCH INSTALLED! See https://pytorch.org/get-started/locally/"