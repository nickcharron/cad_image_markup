# cad_image_markup

## Install Instructions:

### Clone reposityory

```
cd root_directory
git clone https://github.com/nickcharron/cad_image_markup.git
```

### Install dependencies

The following software needs to be install before building

* Eigen (min. 3.3.7)
* gflags
* Catch2 (min. 2.13.2)
* PCL (min. 1.11.0)
* Ceres (min. 1.14)
* OpenCV (min. 3.2.0)

To install dependencies, we have made a script called install_deps.bash. You can call the script using:


```
cd root_directory/cad_image_markup/scripts
sudo bash install_deps.sh
```

### Build

```
cd root_directory/cad_image_markup
mkdir build
cd build
cmake ..
make -j8
```

## Running program

There is one main executable to run:

```
cd root_directory/cad_image_markup/build
./cad_image_markup_main [args]
```

The argument descriptions can be displayed using:

```
./cad_image_markup_main --help
```

TODO: move these descriptions to the main executable gflags which can be read using --help. 
We want these to all be in ont spot so they only need to be updated once

### Parameter descriptions

The test data provides examples for the data and files required for the CAD image markup and a template for how they may be organized. The contents of each folder, and whether they are read (READ) or written to (WRITE) by the CAD image markup are summarized below: 

- cad/: (READ) The CAD drawing of the structure to be marked-up. In .png format.
- canney_edges/: (WRITE) The output of the edge detection process. The structure point cloud used for pose estimation of the camera is downsampled from the white pixels in this image. This image is only generated if automatic edge detection is selected. In .png format.
- images/: (READ) The original camera images of the structure used for the image to CAD markup process. This is only required to be provided if automatic edge detection is selected. In .png format.
- labelled_images/: (READ) Files containing the structure edge labels generated using the [Slava label tool](https://github.com/Slava/label-tool). 
    If using manual edges, the edges from the CAD drawing should be included here as well. The label files are in .json format. 
- marked_up_cad/: (WRITE) The output of the image to CAD markup process. This is the input CAD drawing with the defects superimposed on it. In .png format.
- marked_up_images/: (READ) The original camera image with the defects to be transfered drawn on with a configurable color. In .png format.
- poses/: (READ) The initial estimate of the pose of the CAD frame in the camera frame. The Camera frame is defined with the z-axis pointing out of the lense, with the x-axis pointing right and the y-axis down. The CAD frame is considered to originate at the centroid of the planar point cloud generated from the CAD drawing provided. The pose format is as follows, with translations in meters and rotations in degrees.
    
    ```
    {"pose":[<x translation>,<y translation>,<z translation>,<x rotation>,<y rotation>,<z rotation>]}
    ```


## Examples

We provide 3 example datasets:

1. Simulation: simulation dataset created using Gazebo where a T-shaped bridge pier is captured
2. Train Station: real dataset of a sloped colum taken from a train station
2. Bridge: real dataset captured of a rectangular bridge pier

To help understand the inputs, we provide scripts for running these examples:

1. run_example_simulated_column.sh: TODO (describe test)
2. run_example_train_station.sh: TODO (describe test)
3. ... TODO


## Configuration 

TODO: update this when code refactor is complete

The CAD image markup process can be configured at runtime by modifying configuration files in .json format. These files should inlcude the same fields and have the same structure as the provided CeresParamsDefault.json and SolutionParamsDefault.json files. The configurable parameters are explained below.

### Solution Parameters

**solution_options**
- cad_cloud_scale: (float) scale of the CAD drawing input [working units/pixel]
- max_solution_iterations: (int) maximum number of iterations of the pose estimation process
- visualize: (bool) run the solution visualizer 
- output_results: (bool) print solution results to the consol at each iteration
- align_centroids: (bool) align centroids of CAD projection and image points when calculating correspondences for Iterative Closest Point pose estimation process
- correspondence_type: (string: "P2POINT" or "P2LINE") type of correspondence to generate for Iterative Closest Point pose estimation process
- max_corr_distance: 100000 (int) initial maximum correspondence distance for Iterative Closest Point pose estimation process [pixels ^ 2]
- attenuate_corr_distance: (bool) decrease the maximum correspondece distance as the solution converges
- corr_bound_low: (int) lower bound of the maximum correspondence distance if it is decreasing
- attenuation_rate: (float: <1) rate at which to decrease the maximum correspondence distance between iterations

**convergence_options**
- convergence_type: (string: "GEOMETRIC" or "LOSS")  type of convergence condition to check for the camera pose estimation process. Geometric tells the solver to check the calculated pose. Loss tells the solver to check the objective function of the optimization process.
- convergence_condition: (string: "DIFFERENCE" or "ABSOLUTE") condition to check for the convergence of the camera estimation process. Difference tells the solver to check the change in the convergence quantity (set with convergence_type) between subsequent iterations. Absolute tells the solver to check the convergence quantity for the last iteration. Absolute is only available with the Loss convergence type since no ground truth pose is known a priori.
- converged_differential_cost: (float) differential cost threshold for pose estimation solution convergence (checked for "LOSS + DIFFERENCE")
- converged_absolute_cost: (float) absolute cost threshold for pose estimation solution convergence (checked for "LOSS + ABSOLUTE")
- converged_differential_translation_m: (float) differential translation threshold for pose estimation solution convergence (checked for "GEOMETRIC" + "DIFFERENCE") [working units]
- converged_differential_rotation_deg: (float) differential translation threshold for pose estimation solution convergence (checked for "GEOMETRIC" + "DIFFERENCE") [degrees]

**output_options**
- output_image: (bool) write the marked up CAD drawing to the specified filepath

**misc_options**
- defect_color: (string: "red", "green", "blue", "white" or "black") color of the defects drawn on the marked up image provided
- feature_label_type: (string: "MANUAL" or "AUTOMATIC") type of edge or feature labelling. If "MANUAL" image and CAD label files should be provided in .json format. If "AUTOMATIC" raw image and CAD files in .png format should be provided.
- cannny_low_threshold_cad: (int) low threshold for Canny edge detector applied to CAD drawing, used when label type is automatic 
- canny_ratio_cad: (int) ratio for Canny edge detector applied to CAD drawing, used when label type is automatic
- canny_kernel_size_cad: (int) Sobel kernel size for Canny edge detector applied to CAD drawing, used when label type is automatic
- cannny_low_threshold_image: (int) low threshold for Canny edge detector applied to iamge, used when label type is automatic 
- canny_ratio_image: (int) ratio for Canny edge detector applied to image, used when label type is automatic
- canny_kernel_size_image: (int) Sobel kernel size for Canny edge detector applied to image, used when label type is automatic
- downsample_image_cloud: (bool) downsample the detected edge pixels to generate clouds using a grid filter, used when label type is automatic
- downsample_grid_size: (int) grid filter size for downsampling edge pixels [pixels]

### Ceres Optimization Parameters
**solver_options**
- minimizer_progress_to_stdout: (bool) print Ceres optimizer progress to the consol at each iteration of the optimization
- max_num_iterations: (int) maximum number of iterations of the Ceres optimizer 
- max_solver_time_in_seconds: (int) maximum time before the Ceres optimizer exits if no solution found [s]
- function_tolerance: (float) Ceres optimization function numerical tolerance
- gradient_tolerance: (float) Ceres optimization gradient numerical tolerance
- parameter_tolerance: (float) Ceres optimization parameter numerical tolerance
- linear_solver_type: (string: "SPARSE_SCHUR", ...) Ceres linear solver algorithm to apply, not recommended to change
- preconditioner_type: (string: "SCHUR_JACOBI", ...) Ceres solver preconditioner type, not recommended to change
- output_results: (bool) output full results of Ceres solution to consol

**problem_options**
- local_parameterization_take_ownership: (bool: false) take ownership of Ceres parameterization data from calling code, not recommended to change
- cost_function_take_ownership: (bool: false) take ownership of Ceres cost function data from calling code, not recommended to change
- loss_function_take_ownership: (bool: false) take ownership of Ceres loss function data from calling code, not recommended to change

**loss_function**
- type: ("HUBER", ...) Loss function for Ceres optimization to use, not recommended to change
- scaling: (int) Scaling factor for Ceres loss function, changes solution sensitivity to outliers in correspondences

## Labelling images for the markup process
[TODO]
