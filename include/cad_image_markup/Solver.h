#pragma once

#include <fstream>
#include <stdio.h>

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <cad_image_markup/camera_models/CameraModel.h>
#include <cad_image_markup/optimization/CamPoseReprojectionCost.h>
#include <cad_image_markup/optimization/CeresParams.h>
#include <cad_image_markup/optimization/PointToLineCost.h>
#include <cad_image_markup/Utils.h>
#include <cad_image_markup/Visualizer.h>

namespace cad_image_markup {

using AlignVec2d = Eigen::aligned_allocator<Eigen::Vector2d>;

/**
 * @brief Class to solve camera pose estimation problem
 */
class Solver {
 public:
  struct Params {
    uint32_t max_solution_iterations{15};
    // TODO CAM: the ceres summary outputs the initial and final losses. These
    // losses are essentially the same this as your pixel error. It would be a
    // lot easier to just compare final loss to initial loss instead of your
    // method. Also we should implement a change in pose option. See this code
    // for an example:
    // https://github.com/BEAMRobotics/libbeam/blob/d07569a86f427455a573f6190d5ae7e2b983b1d6/beam_matching/src/loam/LoamScanRegistration.cpp#L290
    std::string convergence_type{
        "FINALLOSS"};  // Options: FINALLOSS, POSECHANGE
    bool transform_progress_to_stdout{false};
    bool visualize{true};
    double cloud_scale{1};
    double convergence_limit{5};
    bool output_results{true};
    std::string ceres_params_path;
  };

  struct ResultsSummary {
    ceres::Solver::Summary ceres_summary;
    uint8_t solution_iterations{0};
  }

  /**
   * @brief Constructor with custom params
   * @param camera_model
   * @param params params needed for this class
   */
  Solver(std::shared_ptr<beam_calibration::CameraModel> camera_model,
         const Params& params);

  /**
   * @brief Constructor with default params
   * @param camera_model
   */
  Solver(std::shared_ptr<beam_calibration::CameraModel> camera_model);

  /**
   * @brief Default destructor
   */
  ~Solver() = default;

  /**
   * @brief Method for estimating the camera pose for an image by solving the
   * projection optimization problem
   * @param cad_cloud 3D point cloud generated from the CAD drawing - used to
   * represent a planar surface of the structure
   * @param camera_cloud 3D point cloud generated from the camera image
   * @param initial_pose T_WORLD_CAMERA estimate
   * @return success boolean
   */
  bool Solve(PointCloud::ConstPtr cad_cloud, PointCloud::ConstPtr camera_cloud,
             const Eigen::Matix4d& T_WORLD_CAMERA, bool visualize);

  /**
   * @brief Accessor method to retrieve the final transform results
   * @return T_WORLD_CAMERA
   */
  Eigen::Matrix4d GetT_WORLD_CAMERA();

  /**
   * @brief method for accessing results summary
   */
  ResultsSummary GetResultsSummary();

 private:
  /**
   * @brief Method for building the Ceres problem by adding the residual blocks
   */
  void BuildCeresProblem();

  /**
   * @brief Method to call the ceres solver on the individual ceres problem
   */
  void SolveCeresProblem();

  /**
   * @brief Method to check the overall problem for convergence
   * @return true if correspondences have converged
   */
  bool HasConverged();

  /**
   * @brief Method to update the visualization display
   * @note Holds the solution until user enters 'n' to progress to the next iteration
   *       or 'r' to cancel the solution
   */
  bool UpdateVisualizer(PointCloud::Ptr CAD_cloud_scaled, Eigen::Matix4d& T_WORLD_CAMERA, pcl::CorrespondencesPtr proj_corrs);

  // options
  Params params_;
  optimization::CeresParams ceres_params_;

  // input member variables
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  std::shared_ptr<Visualizer> visualizer_;
  PointCloud::ConstPtr camera_cloud_;
  PointCloud::ConstPtr cad_cloud_;

  // new member variables
  ResultsSummary summary_;
  std::vector<double> results_;
  pcl::CorrespondencesPtr corrs_;
  std::shared_ptr<ceres::Problem> problem_;
  double last_iteration_cost_;
  uint16_t solution_iterations_;
  bool solution_stalled_;
};

}  // namespace cad_image_markup
