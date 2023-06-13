#!/bin/bash
set -e

if [ "$#" -ne 2 ]; 
    then
    echo "Invalid number of arguments. Built path must be provided."
    echo "Usage: bash /path_to/cad_image_markup/examples/run_example_train_station.sh [BUILD_PATH] [OUTPUT_DIR]"
    exit 0
fi

EXECUTABLE_PATH="$1/cad_image_markup_main_obsolete"
if [ -f "$EXECUTABLE_PATH" ]; then
    echo "found executable"
else 
    echo "Invalid Build path, this path must contain cad_image_markup_main_obsolete"
    echo "Usage: bash /path_to/cad_image_markup/examples/run_example_train_station.sh [BUILD_PATH] [OUTPUT_DIR]"
    exit 0
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

DATA_ROOT="$SCRIPT_DIR/example_data"
CONFIG_ROOT="$SCRIPT_DIR/example_config"

CAD="$DATA_ROOT/labelled_images/real_cad_feature_label.json"
CAD_IMG="$DATA_ROOT/cad/real_cad.png"
CANNY="$DATA_ROOT/canny_edge/canny_edges_cad.png"
IMAGE="$DATA_ROOT/images/real_view_1_mask.png"
CANNY_IMAGE="$DATA_ROOT/canny_edge/canny_edges_image.png"
INTRINSICS="$DATA_ROOT/Radtan_intrinsics_phone.json"
DEFECT_IMG="$DATA_ROOT/marked_up_images/real_view_1_image.png"
POSE="$DATA_ROOT/poses/initial_pose.json"

CONFIG="$CONFIG_ROOT/SolutionParamsExampleTrainStation.json"
CERES_CONFIG="$CONFIG_ROOT/CeresParamsExample.json"

# combine into one command
cmd="$EXECUTABLE_PATH --cad $CAD --cad_image $CAD_IMG --canny_edge_cad $CANNY"
cmd="$cmd --image $IMAGE --canny_edge_image $CANNY_IMAGE --intrinsics $INTRINSICS"
cmd="$cmd --defect_image $DEFECT_IMG --output_directory $2 --initial_pose $POSE"
cmd="$cmd --config $CONFIG --ceres_config $CERES_CONFIG"

# display command to user and run
echo "Running command: "
echo $cmd
$cmd

