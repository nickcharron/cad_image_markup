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

EXECUTABLE_PATH_EXEDGES="$1/cad_image_markup_extract_edges_canny"
if [ -f "$EXECUTABLE_PATH_EXDEFECTS" ]; then
    echo "found extract edges with canny executable"
else 
    echo "Invalid Build path, this path must contain cad_image_markup_extract_edges_canny"
    echo "Usage: bash /path_to/cad_image_markup/examples/run_example_train_station.sh [BUILD_PATH] [OUTPUT_DIR]"
    exit 0
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

DATA_ROOT="$SCRIPT_DIR/example_data/train_station"
CONFIG_ROOT="$SCRIPT_DIR/example_config"

# setup the defect extractor
DEFECT_IMG="$DATA_ROOT/img_with_defects.png"
DEFECT_LABELS="$2/defect_labels.json"
DEFECT_COLOR="red"

cmd1="$EXECUTABLE_PATH_EXDEFECTS --image_path $DEFECT_IMG"
cmd1="$cmd1 --output_json $DEFECT_LABELS --defect_color $DEFECT_COLOR"

# setup the edge extractor
IMAGE_EDGES="$DATA_ROOT/img_edges.png"
IMAGE_LABEL="$2/image_labels.json"
CANNY_CONFIG="$CONFIG_ROOT/CannyParamsExampleTrainStation.json"

cmd2="$EXECUTABLE_PATH_EXEDGES --image_path $IMAGE_EDGES --output_json $IMAGE_LABEL"
cmd2="$cmd2 --config $CANNY_CONFIG"

# setup the aligment and markup tool
IMAGE_IMG="$DATA_ROOT/img.png"
CAD_LABEL="$DATA_ROOT/cad_labels.json"
CAD_IMG="$DATA_ROOT/cad_img.png"
INTRINSICS="$DATA_ROOT/intrinsics.json"
POSE="$DATA_ROOT/initial_pose.json"

SOLUTION_CONFIG="$CONFIG_ROOT/SolutionParamsExampleTrainStation.json"
CERES_CONFIG="$CONFIG_ROOT/CeresParamsExample.json"

# combine into one command
cmd3="$EXECUTABLE_PATH_ALIGN --cad_label_path $CAD_LABEL --cad_image_path $CAD_IMG"
cmd3="$cmd3 --image_label_path $IMAGE_LABEL --image_path $IMAGE_IMG --intrinsics_path $INTRINSICS"
cmd3="$cmd3 --defect_path $DEFECT_LABELS --output_directory $2 --initial_pose_path $POSE"
cmd3="$cmd3 --solution_config_path $SOLUTION_CONFIG --ceres_config_path $CERES_CONFIG"

# display commands to user and run
echo "Running command to extract defects from provided image: "
echo $cmd1
$cmd1

echo "Running command to extract edges from provided image: " 
echo $cmd2
$cmd2

echo "Running command to perform alignment and markup: "
echo $cmd3
$cmd3

