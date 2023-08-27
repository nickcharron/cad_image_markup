###########################################################################
# Descr. Study to examine the sensitivity of the pose estimation to initial 
#        poses
###########################################################################

import subprocess
import os

########################################
# Define ground truth poses            #
# For train station real-world dataset #
########################################
T_CAMERA_WORLD_base1 = [0.999655, -0.0131739, 0.0227275, -0.0386311,
                        0.0136331, 0.999704, -0.0201694, -0.0738061, 
                        -0.0224551, 0.0204723, 0.999538, 2.20684, 
                        0, 0, 0, 1]

T_CAMERA_WORLD_base2 = [0.958018, -0.000869915, 0.286708, -0.0508706,
                        0.0129315, 0.999109, -0.0401783, 0.0167515, 
                        -0.286418, 0.0421991, 0.957175, 1.93095, 
                        0, 0, 0, 1]

T_CAMERA_WORLD_base3 = [0.946142, -0.02028, 0.323117, 0.00419045, 
                        0.0450889, 0.996564, -0.06948, 0.00905989, 
                        -0.320598, 0.0803069, 0.943805, 2.30273,
                        0, 0, 0, 1]

T_CAMERA_WORLD_base4 = [0.998358, 0.00461789, -0.0570924, -0.103161,
                        -0.00327674, 0.999717, 0.0235622, 0.0120111, 
                        0.0571851, -0.0233364, 0.998091, 1.93493, 
                        0, 0, 0, 1]

########################################
# Define test parameters               #
########################################
NUM_TRIALS = 20

SCRIPT_DIR = os.getcwd()
DATA_ROOT = os.path.join(SCRIPT_DIR, "examples/example_data/train_station")
CONFIG_ROOT = os.path.join(SCRIPT_DIR, "examples/example_config")

print("DATA ROOT: ", DATA_ROOT)
print("CONFIG ROOT : ", CONFIG_ROOT)

# output_path
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "tests/ablation_study/outputs")

print("OUTPUT DIR: ", OUTPUT_DIR)

# defect extractor params
# need to run this??

# edge extractor 
EDGE_EXTRACTOR_EXE = os.path.join(SCRIPT_DIR, "build/cad_image_markup_extract_edges_canny")
IMAGE_EDGES = os.path.join(DATA_ROOT, "img_edges.png")
IMAGE_LABEL = os.path.join(OUTPUT_DIR, "image_labels.json")
CANNY_CONFIG = os.path.join(CONFIG_ROOT, "CannyParamsExampleTrainStation.json")

print("Edge extractor executable: ", EDGE_EXTRACTOR_EXE)

# align and markup tool
MARKUP_EXE = os.path.join(SCRIPT_DIR, "build/cad_image_markup_main")
IMAGE_IMG = os.path.join(DATA_ROOT, "img.png")
CAD_LABEL = os.path.join(DATA_ROOT, "cad_labels.json")
CAD_IMG = os.path.join(DATA_ROOT, "cad_img.png")
INTRINSICS = os.path.join(SCRIPT_DIR, "tests/ablation_study/intrinsics.json") # use a local copy since it will change
POSE = os.path.join(DATA_ROOT, "initial_pose.json")

SOLUTION_CONFIG = os.path.join(CONFIG_ROOT, "SolutionParamsExampleTrainStation.json")
CERES_CONFIG = os.path.join(CONFIG_ROOT, "CeresParamsExample.json")

########################################
# Run trial                            #
########################################


print("----------------------RUNNING EDGE EXTRACTOR--------------------")
subprocess.run([EDGE_EXTRACTOR_EXE, 
                "--image_path", IMAGE_EDGES, 
                "--output_json", IMAGE_LABEL,
                "--config", CANNY_CONFIG])

print("------------------------RUNNING MARKUP--------------------------")
subprocess.run([MARKUP_EXE, 
                "--cad_label_path", , 
                "--cad_image_path", , 
                "--image_label_path", , 
                "--image_path", , 
                "--intrinsics_path", , 
                "--defect_path", , 
                "--output_directory", , 
                "initial_pose_path", , 
                "--solution_config_path", , 
                "--ceres_config_path", ])
