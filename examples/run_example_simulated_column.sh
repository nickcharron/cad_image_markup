#!/bin/bash
set -e

if [ "$#" -ne 1 ]; 
    then
    echo "Invalid number of arguments. Built path must be provided."
    echo "Usage: bash /path_to/cad_image_markup/examples/run_example_train_station.sh [BUILD_PATH]"
    exit 0
fi

EXECUTABLE_PATH="$1/cad_image_markup_main"
if [ -f "$EXECUTABLE_PATH" ]; then
    echo "found executable"
else 
    echo "Invalid Build path, this path must contain cad_image_markup_main"
    echo "Usage: bash /path_to/cad_image_markup/examples/run_example_train_station.sh [BUILD_PATH]"
    exit 0
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

DATA_ROOT="$SCRIPT_DIR/example_data"
CONFIG_ROOT="$SCRIPT_DIR/example_config"

CAD="$DATA_ROOT/labelled_images/sim_cad_feature_label.json"
CAD_IMG="$DATA_ROOT/cad/sim_cad.png"
IMAGE="$DATA_ROOT/labelled_images/sim_view_2_feature_label.json"
DEFECT_IMG="$DATA_ROOT/marked_up_images/sim_view_2_image_markup.png"
INTRINSICS="$DATA_ROOT/Radtan_intrinsics.json"

OUTPUT="$DATA_ROOT/marked_up_cad/sim_cad_view_2_markup.png"
POSE="$DATA_ROOT/poses/initial_pose.json"

CONFIG="$CONFIG_ROOT/SolutionParamsExampleSim.json"
CERES_CONFIG="$CONFIG_ROOT/CeresParamsExample.json"

# combine into one command
cmd="$EXECUTABLE_PATH --cad $CAD --cad_image $CAD_IMG"
cmd="$cmd --image $IMAGE --intrinsics $INTRINSICS"
cmd="$cmd --defect_image $DEFECT_IMG --output_image $OUTPUT --initial_pose $POSE"
cmd="$cmd --config $CONFIG --ceres_config $CERES_CONFIG"

# display command to user and run
echo "Running command: "
echo $cmd
$cmd

