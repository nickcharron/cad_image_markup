###########################################################################
# Descr. Study to examine the sensitivity of the pose estimation to initial 
#        poses
###########################################################################

import random
import subprocess
import os
import numpy as np
import json

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
EPSILON = 0.3

SCRIPT_DIR = os.getcwd()
DATA_ROOT = os.path.join(SCRIPT_DIR, "examples/example_data/train_station")
CONFIG_ROOT = os.path.join(SCRIPT_DIR, "examples/example_config")
DEFECT_FILE = os.path.join(SCRIPT_DIR, "tests/ablation_study/dummy_defect_labels.json")

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
TRUE_POSE = os.path.join(SCRIPT_DIR, "tests/ablation_study/true_pose_1_train_station.json")
TEST_POSE = os.path.join(SCRIPT_DIR, "tests/ablation_study/test_pose.json")

SOLUTION_CONFIG = os.path.join(SCRIPT_DIR, "tests/ablation_study/SolutionParamsTrainStationAblation.json")
CERES_CONFIG = os.path.join(CONFIG_ROOT, "CeresParamsExample.json")

########################################
# Helper Functions                     #
########################################

def get_pose_result(proc):

    if not "MARKUP: Solver successful" in str(proc.stdout): 
        return []

    pose_start_index = str(proc.stdout).find('T_WORLD_CAMERA')
    proc_slice_1 = str(proc.stdout)[pose_start_index:]
    pose_end_index = proc_slice_1.find('[INFO]')
    proc_slice_2 = proc_slice_1[:pose_end_index]
    proc_no_newlines = proc_slice_2.replace('\\n', '')
    proc_values_only = proc_no_newlines.replace('T_WORLD_CAMERA:', '')
    proc_values_list = proc_values_only.split(' ')

    transform_result = []
    for value in proc_values_list: 
        if value != '':
            try:
                transform_result.append(float(value))
            except: 
                return []

    return transform_result

def compare_transforms(ground_truth, solution, epsilon):

    if len(ground_truth) != len(solution): 
        return False
    
    all_components_correct = True

    for i in range(len(ground_truth)):
        if np.abs(ground_truth[i] - solution[i]) > epsilon: 
            all_components_correct = False

    return all_components_correct

def perturb_pose(initial_pose, max_pos_offset, max_angle_offset): 
    perturbed_pose = []
    for pos_value in initial_pose[:3]:
        perturbed_pose.append(
            pos_value + random.uniform(max_pos_offset, 
                                       -max_pos_offset))
        
    for angle_value in initial_pose[3:]: 
        perturbed_pose.append(
            angle_value + random.uniform(max_angle_offset, 
                                       -max_angle_offset)) 
        
    return perturbed_pose


########################################
# Run trial                            #
########################################

# exclude the solutions that just fail to parse
success_count = 0
failed_count = 0

for trial in range(NUM_TRIALS):

    # perturb the true camera pose
    max_pos_offset = 0.1
    max_angle_offset = 10  # (degrees)

    with open(TRUE_POSE) as f: 
        d = json.load(f)
        pose = d['pose']

    perturbed_init = perturb_pose(pose, max_pos_offset, max_angle_offset)

    d['pose'] = perturbed_init

    with open(TEST_POSE, 'w') as f: 
        json.dump(d, f)

    print("----------------------RUNNING EDGE EXTRACTOR--------------------")
    subprocess.run([EDGE_EXTRACTOR_EXE, 
                    "--image_path", IMAGE_EDGES, 
                    "--output_json", IMAGE_LABEL,
                    "--config", CANNY_CONFIG])

    print("------------------------RUNNING MARKUP--------------------------")
    proc = subprocess.run([MARKUP_EXE, 
                    "--cad_label_path", CAD_LABEL, 
                    "--cad_image_path", CAD_IMG, 
                    "--image_label_path", IMAGE_LABEL, 
                    "--image_path", IMAGE_IMG, 
                    "--intrinsics_path", INTRINSICS, 
                    "--defect_path", DEFECT_FILE, 
                    "--output_directory", OUTPUT_DIR, 
                    "--initial_pose_path", TEST_POSE, 
                    "--solution_config_path", SOLUTION_CONFIG, 
                    "--ceres_config_path", CERES_CONFIG], capture_output=True)

    solved_transform = get_pose_result(proc)

    print(solved_transform)

    result = compare_transforms(T_CAMERA_WORLD_base1, solved_transform, EPSILON)
    print(result)

    if result: 
        success_count += 1
    if not result and len(solved_transform) > 0: 
        failed_count += 1

print("------------------------TEST FINISHED--------------------------")
print("Number of successful solutions: ", success_count)
print("Number of failed solutions: ", failed_count)

