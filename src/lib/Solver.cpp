#include <cad_image_markup/Solver.h>

namespace cad_image_markup {

Solver::Solver(std::shared_ptr<beam_calibration::CameraModel> camera_model,
               const Params& params)
    : camera_model_(camera_model), params_(params) {
  ceres_params_ = optimization::CeresParams(params_.ceres_params_path);
  visualizer_ = std::make_shared<Visualizer>("solution visualizer"); // Initialize visualizer directly in Solver
}

Solver::Solver(std::shared_ptr<beam_calibration::CameraModel> camera_model)
    : camera_model_(camera_model) {
      visualizer_ = std::make_shared<Visualizer>("solution visualizer"); // Initialize visualizer directly in Solver 
}

bool Solve(PointCloud::ConstPtr cad_cloud, PointCloud::ConstPtr camera_cloud,
           Eigen::Matix4d& T_WORLD_CAMERA, bool visualize) {
  cad_cloud_ = cad_cloud;
  camera_cloud_ = camera_cloud;             
  // TODO CAM: update this: 
  // Remaining todo: 
  //    update convergence checking and error estimation 
  // ALSO: create a separate function for calculating correspondencs to make this more clear

  iterations_ = 0;
  last_iteration_cost_ = 0;
  
  bool has_converged = false;

  PointCloud::Ptr CAD_cloud_scaled = util::ScaleCloud(CAD_cloud_, params.cad_cloud_scale);

  // correspondence object tells the cost function which points to compare
  pcl::CorrespondencesPtr proj_corrs = std::make_shared<pcl::Correspondences>();

  // transform, project, and get correspondences
  util::CorrespondenceEstimate(CAD_cloud_scaled, camera_cloud_, T_WORLD_CAMERA, proj_corrs,
                offset_type_);

  if (visualize_) visualizer_->startVis();

  // loop problem until it has converged
  while (!has_converged && solution_iterations_ < max_solution_iterations_) {
    solution_iterations_++;

    printf("Solver iteration %u \n", solution_iterations_);

    if (visualize_) {
      UpdateVisualizer(PointCloud::Ptr CAD_cloud_scaled, Eigen::Matix4d& T_WORLD_CAMERA, pcl::CorrespondencesPtr proj_corrs);
    }

    BuildCeresProblem(problem, proj_corrs, camera_model, camera_cloud_,
                      CAD_cloud_scaled);

    SolveCeresProblem(problem, minimizer_progress_to_stdout_);

    T_WORLD_CAMERA = util::QuaternionAndTranslationToTransformMatrix(results_);

    if (transform_progress_to_stdout_) {
      std::string sep = "\n----------------------------------------\n";
      std::cout << T_WORLD_CAMERA << sep;
    }

    // transform, project, and get correspondences
    util::CorrespondenceEstimate(CAD_cloud_scaled, camera_cloud_, T_WORLD_CAMERA, proj_corrs, "none");

    has_converged = HasConverged();

    solution_iterations_++;
    last_iteration_results_ = results_;

  }

  if (visualize_) visualizer_->endVis();
  if (has_converged)
    return true;
  
  return false;
}

Eigen::Matrix4d Solver::GetT_WORLD_CAMERA() { 
  
  Eigen::Matrix4d T_WORLD_CAMERA = util::QuaternionAndTranslationToTransformMatrix(results_);

  return T_WORLD_CAMERA; 
  
}

ResultsSummary Solver::GetResultsSummary() { return summary_; }

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

    Eigen::Vector3d P_STRUCT(cad_cloud_->at(corrs_->at(i).index_match).x,
                             cad_cloud_->at(corrs_->at(i).index_match).y,
                             cad_cloud_->at(corrs_->at(i).index_match).z);

    // TODO CAM: add point to plane cost function option
    std::unique_ptr<ceres::CostFunction> cost_function(
        CeresReprojectionCostFunction::Create(pixel, P_STRUCT, camera_model));

    std::unique_ptr<ceres::LossFunction> loss_function =
        ceres_params_.LossFunction();
    problem_->AddResidualBlock(cost_function.release(), loss_function.get(),
                               &(results_[0]));
  }
}

void Solver::SolveCeresProblem() {
  ceres::Solver::Summary ceres_summary;

  if (params_.output_results) {
    LOG_INFO("Solving ceres problem...");
    ceres::Solve(ceres_params_.SolverOptions(), problem_.get(),
                 &summary_.ceres_summary);
    LOG_INFO("Done.");
    LOG_INFO("Outputting ceres summary:");
    std::string report = ceres_summary.FullReport();
    std::cout << report << "\n";
  } else {
    ceres::Solve(ceres_params_.SolverOptions(), problem_.get(),
                 &summary_.ceres_summary);
  }
}

bool Solver::HasConverged(){

    // Cannot converge on a single solver iteration
    if(solution_iterations <= 1)
      return false;

    // Check ceres loss convergence conditions
    if(params_.convergence_type == cad_image_markup::LOSS_CONVERGENCE) {
      double differential_cost = std::abs(summary_.ceres_results.final_cost - last_iteration_cost_); 
      if ((differential_cost <= params_.converged_differential_cost 
          && params_.convergence_condition == cad_image_markup::DIFF_CONVERGENCE)
          || (summary_.ceres_results.final_cost <= params_.converged_absolute_cost 
          && params_.convergence_condition == cad_image_markup::ABS_CONVERGENCE))
        return true
    }

    // Check physical geometric convergence conditions
    // Assumes that conditions are provided in the same unit as the cloud scale
    else if (params_.convergence_type = cad_image_markup::GEO_CONVERGENCE) {

      Eigen::Quaterniond q_current{results[0], results[1], results[2], results[3]};
      Eigen::Vector3f euler_angles_current = q.toRotationMatrix().eulerAngles(0, 1, 2);

      Eigen::Quaterniond q_last{last_iteration_results_[0], last_iteration_results_[1], last_iteration_results_[2], last_iteration_results_[3]};
      Eigen::Vector3f euler_angles_last = q.toRotationMatrix().eulerAngles(0, 1, 2);

      // Check absolute conditions first
      if (euler_angles_current(0) <= params_.converged_absolute_rotation
          && euler_angles_current(1) <= params_.converged_absolute_rotation
          && euler_angles_current(2) <= params_.converged_absolute_rotation
          && results_[4] < params_.converged_absolute_translation
          && results_[5] < params_.converged_absolute_translation
          && results_[6] < params_.converged_absolute_translation
          && params_.convergence_condition == cad_image_markup::ABS_CONVERGENCE)
          return true;

      // Check differential condition 
      if (std::abs(euler_angles_current(0)-euler_angles_last(0)) <= params_.converged_differential_rotation
          && std::abs(euler_angles_current(1)-euler_angles_last(1)) <= params_.converged_differential_rotation
          && std::abs(euler_angles_current(2)-euler_angles_last(2)) <= params_.converged_differential_rotation
          && std::abs(results_[4] - last_iteration_results_[4]) < params_.converged_absolute_translation
          && std::abs(results_[5] - last_iteration_results_[5]) < params_.converged_absolute_translation
          && std::abs(results_[6] - last_iteration_results_[6]) < params_.converged_absolute_translation
          && params_.convergence_condition == cad_image_markup::DIFF_CONVERGENCE)
      
      )

      return false;

    }
      
}

bool UpdateVisualizer(PointCloud::Ptr CAD_cloud_scaled, Eigen::Matix4d& T_WORLD_CAMERA, pcl::CorrespondencesPtr proj_corrs) {

  // update the position of the transformed cloud based on
  // the upated transformation matrix for visualization
  PointCloud::Ptr trans_cloud = util::TransformCloud(CAD_cloud_scaled, T_CS);

  // project cloud for visualizer
  PointCloud::Ptr proj_cloud = util::ProjectCloud(trans_cloud);

  // blow up the transformed cloud for visualization
  util::ScaleCloud(trans_cloud, (1 / params_.cad_cloud_scale));

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