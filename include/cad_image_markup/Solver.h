#pragma once

#include <fstream>
#include <stdio.h>

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cad_image_markup/camera_models/CameraModel.h>
#include <cad_image_markup/optimization/CamPoseReprojectionCost.h>
#include <cad_image_markup/optimization/CeresParams.h>
#include <cad_image_markup/optimization/PointToLineCost.h>
#include <cad_image_markup/Utils.h>
#include <cad_image_markup/Params.h>
#include <cad_image_markup/Visualizer.h>

namespace cad_image_markup {

/**
 * @brief Class to solve camera pose estimation problem
 */
class Solver {
 public:

  struct ResultsSummary {
    ceres::Solver::Summary ceres_summary;
    uint8_t solution_iterations{0};
  };

  /**
   * @brief Constructor with custom params
   * @param camera_model
   * @param params params needed for this class. See Params.h
   * @param ceres config path. See optimization/CeresParams.h
   */
  Solver(const std::shared_ptr<cad_image_markup::CameraModel>& camera_model,
         const Params& params, const std::string& ceres_config_path = "");

  /**
   * @brief Constructor with default params
   * @param camera_model
   */
  Solver(const std::shared_ptr<cad_image_markup::CameraModel>& camera_model);

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
             Eigen::Matrix4d& T_WORLD_CAMERA, bool visualize);

  /**
   * @brief Accessor method to retrieve the final transform results
   * @return T_WORLD_CAMERA
   */
  Eigen::Matrix4d GetT_WORLD_CAMERA();

  /**
   * @brief method for accessing results summary
   */
  //ceres::Solver::Summary::FullReport GetResultsSummary();

 private:
  /**
   * @brief Method for building the Ceres problem by adding the residual blocks
   */
  void BuildCeresProblem(pcl::CorrespondencesPtr proj_corrs, std::shared_ptr<cad_image_markup::CameraModel> camera_model, 
                         PointCloud::ConstPtr camera_cloud, PointCloud::ConstPtr cad_cloud);

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
  bool UpdateVisualizer(PointCloud::Ptr CAD_cloud_scaled, Eigen::Matrix4d& T_WORLD_CAMERA, pcl::CorrespondencesPtr proj_corrs);

  // options
  const Params& params_;
  optimization::CeresParams ceres_params_;

  // input member variables
  std::shared_ptr<cad_image_markup::CameraModel> camera_model_;
  std::shared_ptr<Visualizer> visualizer_;
  PointCloud::ConstPtr camera_cloud_;
  PointCloud::ConstPtr cad_cloud_;

  // new member variables
  ceres::Solver::Summary summary_;
  std::vector<double> results_;
  std::vector<double> last_iteration_results_;
  pcl::CorrespondencesPtr corrs_;
  std::shared_ptr<ceres::Problem> problem_;
  double last_iteration_cost_;
  uint16_t solution_iterations_;
  bool solution_stalled_;
};

}  // namespace cad_image_markup
