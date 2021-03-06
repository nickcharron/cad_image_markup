#include <cad_image_markup/Solver.h>

namespace cad_image_markup {

Solver::Solver(
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model,
    const Params& params, const std::string& ceres_config_path)
    : camera_model_(camera_model), params_(params) {
  ceres_params_ = optimization::CeresParams(ceres_config_path);

  // Initialize visualizer directly in Solver
  visualizer_ = std::make_shared<Visualizer>("solution visualizer");
}

bool Solver::Solve(PointCloud::ConstPtr cad_cloud,
                   PointCloud::ConstPtr camera_cloud,
                   Eigen::Matrix4d& T_WORLD_CAMERA, bool visualize) {
  // cad_cloud_ = cad_cloud;
  // camera_cloud_ = camera_cloud;

  solution_iterations_ = 0;
  last_iteration_cost_ = 0;

  bool has_converged = false;

  PointCloud::Ptr CAD_cloud_scaled =
      utils::ScaleCloud(cad_cloud, params_.cad_cloud_scale);

  // correspondence object tells the cost function which points to compare
  pcl::CorrespondencesPtr proj_corrs(new pcl::Correspondences);

  // transform, project, and get correspondences
  int num_correspondences;
  if (params_.correspondence_type == CorrespondenceType::P2POINT) {
    num_correspondences = 1;
  } else if (params_.correspondence_type == CorrespondenceType::P2LINE) {
    num_correspondences = 2;
  }
  utils::CorrespondenceEstimate(CAD_cloud_scaled, camera_cloud_, T_WORLD_CAMERA,
                                proj_corrs, params_.align_centroids,
                                params_.max_corr_distance, num_correspondences);

  if (params_.visualize) visualizer_->StartVis();

  // loop problem until it has converged
  while (!has_converged &&
         solution_iterations_ < params_.max_solution_iterations) {
    solution_iterations_++;

    printf("Solver iteration %u \n", solution_iterations_);

    if (params_.visualize) {
      if (!UpdateVisualizer(CAD_cloud_scaled, T_WORLD_CAMERA, proj_corrs))
        return false;
    }

    BuildCeresProblem(proj_corrs, camera_model_, camera_cloud_,
                      CAD_cloud_scaled);

    SolveCeresProblem();

    T_WORLD_CAMERA = utils::QuaternionAndTranslationToTransformMatrix(results_);

    // transform, project, and get correspondences
    utils::CorrespondenceEstimate(
        CAD_cloud_scaled, camera_cloud_, T_WORLD_CAMERA, proj_corrs,
        params_.align_centroids, params_.max_corr_distance,
        num_correspondences);

    void CorrespondenceEstimate(
        PointCloud::ConstPtr cad_cloud, PointCloud::ConstPtr camera_cloud,
        const Eigen::Matrix4d& T, pcl::CorrespondencesPtr corrs,
        bool align_centroids, double max_corr_distance, int num_corrs);

    has_converged = HasConverged();

    solution_iterations_++;
    last_iteration_results_ = results_;
  }

  if (params_.visualize) visualizer_->EndVis();
  if (has_converged) return true;

  return false;
}

Eigen::Matrix4d Solver::GetT_WORLD_CAMERA() {
  Eigen::Matrix4d T_WORLD_CAMERA =
      utils::QuaternionAndTranslationToTransformMatrix(results_);

  return T_WORLD_CAMERA;
}

// Solver::Summary::FullReport Solver::GetResultsSummary() { return summary_; }

void Solver::BuildCeresProblem(
    pcl::CorrespondencesPtr proj_corrs,
    std::shared_ptr<cad_image_markup::CameraModel> camera_model,
    PointCloud::ConstPtr camera_cloud, PointCloud::ConstPtr cad_cloud) {
  if (params_.output_results) {
    LOG_INFO("Building ceres problem...");
  }
  // initialize problem
  problem_ = std::make_shared<ceres::Problem>(ceres_params_.ProblemOptions());

  std::unique_ptr<ceres::LocalParameterization> parameterization =
      ceres_params_.SE3QuatTransLocalParametrization();
  problem_->AddParameterBlock(&(results_[0]), 7, parameterization.get());

  // add residuals
  for (int i = 0; i < proj_corrs->size(); i++) {
    Eigen::Vector2d pixel(camera_cloud->at(proj_corrs->at(i).index_query).x,
                          camera_cloud->at(proj_corrs->at(i).index_query).y);

    Eigen::Vector3d P_STRUCT1(cad_cloud->at(proj_corrs->at(i).index_match).x,
                              cad_cloud->at(proj_corrs->at(i).index_match).y,
                              cad_cloud->at(proj_corrs->at(i).index_match).z);

    Eigen::Vector3d P_STRUCT2;

    // If two correspondences have been specified, every second correspondence
    // should be the second correspondence for the same source point
    if (params_.correspondence_type == CorrespondenceType::P2LINE) {
      i++;
      P_STRUCT2(0) = cad_cloud->at(proj_corrs->at(i).index_match).x;
      P_STRUCT2(1) = cad_cloud->at(proj_corrs->at(i).index_match).y;
      P_STRUCT2(2) = cad_cloud->at(proj_corrs->at(i).index_match).z;
    }

    // Add the appropriate cost function and loss function
    std::unique_ptr<ceres::CostFunction> cost_function1(
        CeresReprojectionCostFunction::Create(pixel, P_STRUCT1, camera_model_));

    std::unique_ptr<ceres::CostFunction> cost_function2(
        point_to_line_cost::CeresReprojectionCostFunction::Create(
            pixel, P_STRUCT1, P_STRUCT2, camera_model_));

    std::unique_ptr<ceres::LossFunction> loss_function =
        ceres_params_.LossFunction();

    if (params_.correspondence_type == CorrespondenceType::P2POINT)
      problem_->AddResidualBlock(cost_function1.release(), loss_function.get(),
                                 &(results_[0]));

    else if (params_.correspondence_type == CorrespondenceType::P2LINE)
      problem_->AddResidualBlock(cost_function2.release(), loss_function.get(),
                                 &(results_[0]));
  }
}

void Solver::SolveCeresProblem() {
  // ceres::Solver::Summary ceres_summary;

  if (params_.output_results) {
    LOG_INFO("Solving ceres problem...");
    ceres::Solve(ceres_params_.SolverOptions(), problem_.get(), &summary_);
    LOG_INFO("Done.");
    LOG_INFO("Outputting ceres summary:");
    std::string report = summary_.FullReport();
    std::cout << report << "\n";
  } else {
    ceres::Solve(ceres_params_.SolverOptions(), problem_.get(), &summary_);
  }
}

bool Solver::HasConverged() {
  // Cannot converge on a single solver iteration
  if (solution_iterations_ <= 1) return false;

  // Check ceres loss convergence conditions
  if (params_.convergence_type == ConvergenceType::LOSS) {
    double differential_cost =
        std::abs(summary_.final_cost - last_iteration_cost_);
    if ((differential_cost <= params_.converged_differential_cost &&
         params_.convergence_condition == ConvergenceCondition::DIFFERENCE) ||
        (summary_.final_cost <= params_.converged_absolute_cost &&
         params_.convergence_condition == ConvergenceCondition::ABSOLUTE))
      return true;
  }

  // Check physical geometric convergence conditions
  // Assumes that conditions are provided in the same unit as the cloud scale
  else if (params_.convergence_type == ConvergenceType::GEOMETRIC) {
    Eigen::Quaterniond q_current{results_[0], results_[1], results_[2],
                                 results_[3]};
    Eigen::Vector3d euler_angles_current =
        q_current.toRotationMatrix().eulerAngles(0, 1, 2);

    Eigen::Quaterniond q_last{
        last_iteration_results_[0], last_iteration_results_[1],
        last_iteration_results_[2], last_iteration_results_[3]};
    Eigen::Vector3d euler_angles_last =
        q_last.toRotationMatrix().eulerAngles(0, 1, 2);

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
    if (std::abs(euler_angles_current(0) - euler_angles_last(0)) <=
            params_.converged_differential_rotation &&
        std::abs(euler_angles_current(1) - euler_angles_last(1)) <=
            params_.converged_differential_rotation &&
        std::abs(euler_angles_current(2) - euler_angles_last(2)) <=
            params_.converged_differential_rotation &&
        std::abs(results_[4] - last_iteration_results_[4]) <
            params_.converged_differential_translation &&
        std::abs(results_[5] - last_iteration_results_[5]) <
            params_.converged_differential_translation &&
        std::abs(results_[6] - last_iteration_results_[6]) <
            params_.converged_differential_translation &&
        params_.convergence_condition == ConvergenceCondition::DIFFERENCE)
      return true;

    return false;
  }
}

bool Solver::UpdateVisualizer(PointCloud::Ptr CAD_cloud_scaled,
                              Eigen::Matrix4d& T_WORLD_CAMERA,
                              pcl::CorrespondencesPtr proj_corrs) {
  // update the position of the transformed cloud based on
  // the upated transformation matrix for visualization
  PointCloud::Ptr trans_cloud =
      utils::TransformCloud(CAD_cloud_scaled, T_WORLD_CAMERA);

  // project cloud for visualizer
  PointCloud::Ptr proj_cloud = utils::ProjectCloud(trans_cloud);

  // blow up the transformed cloud for visualization
  utils::ScaleCloud(trans_cloud, (1 / params_.cad_cloud_scale));

  visualizer_->DisplayClouds(camera_cloud_, trans_cloud, proj_cloud, proj_corrs,
                             "camera_cloud", "transformed_cloud",
                             "projected_cloud");

  // wait on user input to continue or cancel the solution
  char end = ' ';

  while (end != 'n' && end != 'r') {
    cin >> end;
  }

  if (end == 'r') return false;
  return true;
}

}  // namespace cad_image_markup