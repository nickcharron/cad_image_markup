#include <cad_image_markup/Solver.h>

namespace cad_image_markup {

#define LOSS_CONVERGENCE 0 
#define GEO_CONVERGENCE 1

#define P2POINT 1
#define P2PLANE 2

#define DIFF_CONVERGENCE 0
#define ABS_CONVERGENCE 1

Solver::Solver(std::shared_ptr<cad_image_markup::CameraModel> camera_model,
               const struct Params& params)
    : camera_model_(camera_model), params_(params) {
  ceres_params_ = optimization::CeresParams(params_.ceres_params_path);
  visualizer_ = std::make_shared<Visualizer>("solution visualizer"); // Initialize visualizer directly in Solver
}

Solver::Solver(std::shared_ptr<cad_image_markup::CameraModel> camera_model)
    : camera_model_(camera_model) {
      visualizer_ = std::make_shared<Visualizer>("solution visualizer"); // Initialize visualizer directly in Solver 
}

bool Solver::Solve(PointCloud::ConstPtr cad_cloud, PointCloud::ConstPtr camera_cloud,
           const Eigen::Matrix4d& T_WORLD_CAMERA, bool visualize) {
  //cad_cloud_ = cad_cloud;
  //camera_cloud_ = camera_cloud;             
  // TODO CAM: update this: 
  // Remaining todo: 
  //    update convergence checking and error estimation 
  // ALSO: create a separate function for calculating correspondencs to make this more clear

  solution_iterations_ = 0;
  last_iteration_cost_ = 0;
  
  bool has_converged = false;

  PointCloud::Ptr CAD_cloud_scaled = utils::ScaleCloud(cad_cloud, params_.cad_cloud_scale);

  // correspondence object tells the cost function which points to compare
  pcl::CorrespondencesPtr proj_corrs (new pcl::Correspondences);

  // transform, project, and get correspondences
  utils::CorrespondenceEstimate(CAD_cloud_scaled, camera_cloud_, T_WORLD_CAMERA, proj_corrs,
                params_.align_centroids, params_.correspondence_type);

  if (params_.visualize) visualizer_->startVis();

  // loop problem until it has converged
  while (!has_converged && solution_iterations_ < params_.max_solution_iterations) {
    solution_iterations_++;

    printf("Solver iteration %u \n", solution_iterations_);

    if (params_.visualize) {
      UpdateVisualizer(PointCloud::Ptr CAD_cloud_scaled, Eigen::Matix4d& T_WORLD_CAMERA, pcl::CorrespondencesPtr proj_corrs);
    }

    BuildCeresProblem(problem_, proj_corrs, camera_model_, camera_cloud_,
                      CAD_cloud_scaled);

    SolveCeresProblem(problem_, minimizer_progress_to_stdout_);

    T_WORLD_CAMERA = utils::QuaternionAndTranslationToTransformMatrix(results_);

    if (transform_progress_to_stdout_) {
      std::string sep = "\n----------------------------------------\n";
      std::cout << T_WORLD_CAMERA << sep;
    }

    // transform, project, and get correspondences
    utils::CorrespondenceEstimate(CAD_cloud_scaled, camera_cloud_, T_WORLD_CAMERA, proj_corrs, 
                  params_.align_centroids, params_.correspondence_type);

    has_converged = HasConverged();

    solution_iterations_++;
    last_iteration_results_ = results_;

  }

  if (params_.visualize) visualizer_->endVis();
  if (has_converged)
    return true;
  
  return false;
}

Eigen::Matrix4d Solver::GetT_WORLD_CAMERA() { 
  
  Eigen::Matrix4d T_WORLD_CAMERA = utils::QuaternionAndTranslationToTransformMatrix(results_);

  return T_WORLD_CAMERA; 
  
}

//Solver::Summary::FullReport Solver::GetResultsSummary() { return summary_; }

void Solver::BuildCeresProblem() {
  if (params_.output_results) {
    LOG_INFO("Building ceres problem...");
  }
  // initialize problem
  problem_ = std::make_shared<ceres::Problem>(ceres_params_.ProblemOptions());

  std::unique_ptr<ceres::LocalParameterization> parameterization =
      ceres_params_.SE3QuatTransLocalParametrization();
  problem_->AddParameterBlock(&(results_[0]), 7, parameterization.get());

  // add residuals
  for (int i = 0; i < corrs_->size(); i++) {
    Eigen::Vector2d pixel(camera_cloud_->at(corrs_->at(i).index_query).x,
                          camera_cloud_->at(corrs_->at(i).index_query).y);

    Eigen::Vector3d P_STRUCT1(cad_cloud_->at(corrs_->at(i).index_match).x,
                             cad_cloud_->at(corrs_->at(i).index_match).y,
                             cad_cloud_->at(corrs_->at(i).index_match).z);
    
    Eigen::Vector3d P_STRUCT2;

    // If two correspondences have been specified, every second correspondence should 
    // be the second correspondence for the same source point
    if (params_.correspondence_type == P2PLANE) {
      i++;
      P_STRUCT2(0) = cad_cloud_->at(corrs_->at(i).index_match).x;
      P_STRUCT2(1) = cad_cloud_->at(corrs_->at(i).index_match).y;
      P_STRUCT2(2) = cad_cloud_->at(corrs_->at(i).index_match).z;
    }

    // Add the appropriate cost function and loss function
    std::unique_ptr<ceres::CostFunction> cost_function1(
          CeresReprojectionCostFunction::Create(pixel, P_STRUCT1, camera_model_));

    std::unique_ptr<ceres::CostFunction> cost_function2(
          CeresReprojectionCostFunctionPlane::Create(pixel, P_STRUCT1, P_STRUCT2, camera_model_));

    std::unique_ptr<ceres::LossFunction> loss_function =
        ceres_params_.LossFunction();

    if (params_.correspondence_type == P2POINT)
      problem_->AddResidualBlock(cost_function1.release(), loss_function.get(),
                               &(results_[0]));

    else if (params_.correspondence_type == P2PLANE) 
      problem_->AddResidualBlock(cost_function2.release(), loss_function.get(),
                               &(results_[0]));
  }
}

void Solver::SolveCeresProblem() {
  //ceres::Solver::Summary ceres_summary;

  if (params_.output_results) {
    LOG_INFO("Solving ceres problem...");
    ceres::Solve(ceres_params_.SolverOptions(), problem_.get(),
                 &summary_);
    LOG_INFO("Done.");
    LOG_INFO("Outputting ceres summary:");
    std::string report = summary_.FullReport();
    std::cout << report << "\n";
  } else {
    ceres::Solve(ceres_params_.SolverOptions(), problem_.get(),
                 &summary_);
  }
}

bool Solver::HasConverged(){

    // Cannot converge on a single solver iteration
    if(solution_iterations <= 1)
      return false;

    // Check ceres loss convergence conditions
    if(params_.convergence_type == LOSS_CONVERGENCE) {
      double differential_cost = std::abs(summary_.ceres_results.final_cost - last_iteration_cost_); 
      if ((differential_cost <= params_.converged_differential_cost 
          && params_.convergence_condition == DIFF_CONVERGENCE)
          || (summary_.ceres_results.final_cost <= params_.converged_absolute_cost 
          && params_.convergence_condition == ABS_CONVERGENCE))
        return true
    }

    // Check physical geometric convergence conditions
    // Assumes that conditions are provided in the same unit as the cloud scale
    else if (params_.convergence_type = GEO_CONVERGENCE) {

      Eigen::Quaterniond q_current{results_[0], results_[1], results_[2], results_[3]};
      Eigen::Vector3f euler_angles_current = q.toRotationMatrix().eulerAngles(0, 1, 2);

      Eigen::Quaterniond q_last{last_iteration_results_[0], last_iteration_results_[1], last_iteration_results_[2], last_iteration_results_[3]};
      Eigen::Vector3f euler_angles_last = q.toRotationMatrix().eulerAngles(0, 1, 2);

      // Check absolute conditions - may not exist 
      /*
      if (euler_angles_current(0) <= params_.converged_absolute_rotation
          && euler_angles_current(1) <= params_.converged_absolute_rotation
          && euler_angles_current(2) <= params_.converged_absolute_rotation
          && results_[4] < params_.converged_absolute_translation
          && results_[5] < params_.converged_absolute_translation
          && results_[6] < params_.converged_absolute_translation
          && params_.convergence_condition == ABS_CONVERGENCE)
        return true;
      */

      // Check differential condition 
      if (std::abs(euler_angles_current(0)-euler_angles_last(0)) <= params_.converged_differential_rotation
          && std::abs(euler_angles_current(1)-euler_angles_last(1)) <= params_.converged_differential_rotation
          && std::abs(euler_angles_current(2)-euler_angles_last(2)) <= params_.converged_differential_rotation
          && std::abs(results_[4] - last_iteration_results_[4]) < params_.converged_differential_translation
          && std::abs(results_[5] - last_iteration_results_[5]) < params_.converged_differential_translation
          && std::abs(results_[6] - last_iteration_results_[6]) < params_.converged_differential_translation
          && params_.convergence_condition == DIFF_CONVERGENCE)
        return true;

      return false;

    }
      
}

bool Solver::UpdateVisualizer(PointCloud::Ptr CAD_cloud_scaled, Eigen::Matrix4d& T_WORLD_CAMERA, pcl::CorrespondencesPtr proj_corrs) {

  // update the position of the transformed cloud based on
  // the upated transformation matrix for visualization
  PointCloud::Ptr trans_cloud = utils::TransformCloud(CAD_cloud_scaled, T_CS);

  // project cloud for visualizer
  PointCloud::Ptr proj_cloud = utils::ProjectCloud(trans_cloud);

  // blow up the transformed cloud for visualization
  utils::ScaleCloud(trans_cloud, (1 / params_.cad_cloud_scale));

  visualizer_->displayClouds(camera_cloud_, trans_cloud, proj_cloud, proj_corrs,
                         "camera_cloud", "transformed_cloud",
                         "projected_cloud");

  // wait on user input to continue or cancel the solution
  char end = ' ';

  while (end != 'n' && end != 'r') {
    cin >> end;
  }

  if (end == 'r') return false;
  return true 
}

}  // namespace cad_image_markup