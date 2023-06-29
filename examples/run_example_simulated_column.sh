#!/bin/bash
set -e

if [ "$#" -ne 2 ]; 
    then
    echo "Invalid number of arguments. Built path must be provided."
    echo "Usage: bash /path_to/cad_image_markup/examples/run_example_train_station.sh [BUILD_PATH] [OUTPUT_DIR]"
    exit 0
fi

EXECUTABLE_PATH_ALIGN="$1/cad_image_markup_main"
if [ -f "$EXECUTABLE_PATH_ALIGN" ]; then
    echo "found align and markup executable"
else 
    echo "Invalid Build path, this path must contain cad_image_markup_main"
    echo "Usage: bash /path_to/cad_image_markup/examples/run_example_train_station.sh [BUILD_PATH] [OUTPUT_DIR]"
    exit 0
fi

EXECUTABLE_PATH_EXDEFECTS="$1/cad_image_markup_extract_annotated_defects"
if [ -f "$EXECUTABLE_PATH_EXDEFECTS" ]; then
    echo "found extract defects executable"
else 
    echo "Invalid Build path, this path must contain cad_image_markup_extract_annotated_defects"
    echo "Usage: bash /path_to/cad_image_markup/examples/run_example_train_station.sh [BUILD_PATH] [OUTPUT_DIR]"
    exit 0
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

DATA_ROOT="$SCRIPT_DIR/example_data"
CONFIG_ROOT="$SCRIPT_DIR/example_config"

# setup the defect extractor
DEFECT_IMG="$DATA_ROOT/marked_up_cad/sim_cad_view_2_markup.png"
DEFECT_PATH="$2/defect_labels_output.json"
DEFECT_COLOR="red"

cmd1="$EXECUTABLE_PATH_EXDEFECTS --image_path $DEFECT_IMG"
cmd1="$cmd1 --output_json $DEFECT_PATH --defect_color $DEFECT_COLOR"

# setup the aligment and markup tool
CAD_LABEL="$DATA_ROOT/labelled_images/sim_cad_feature_label.json"
CAD_IMG="$DATA_ROOT/cad/sim_cad.png"
IMAGE_LABEL="$DATA_ROOT/labelled_images/sim_view_2_feature_label.json"
IMAGE_IMG="$DATA_ROOT/marked_up_images/sim_view_2_image_markup.png"
INTRINSICS="$DATA_ROOT/Radtan_intrinsics.json"
POSE="$DATA_ROOT/poses/initial_pose_sim.json"

SOLUTION_CONFIG="$CONFIG_ROOT/SolutionParamsExampleSim.json"
CERES_CONFIG="$CONFIG_ROOT/CeresParamsExample.json"

# combine into one command
cmd2="$EXECUTABLE_PATH_ALIGN --cad_label_path $CAD_LABEL --cad_image_path $CAD_IMG"
cmd2="$cmd2 --image_label_path $IMAGE_LABEL --image_path $IMAGE_IMG --intrinsics_path $INTRINSICS"
cmd2="$cmd2 --defect_path $DEFECT_PATH --output_directory $2 --initial_pose_path $POSE"
cmd2="$cmd2 --solution_config_path $SOLUTION_CONFIG --ceres_config_path $CERES_CONFIG"

# display commands to user and run
echo "Running command to extract defects from provided image: "
echo $cmd1
$cmd1

echo "Running command to perform alignment and markup: "
echo $cmd2
$cmd2

