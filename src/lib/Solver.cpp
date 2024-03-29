#include <cad_image_markup/Solver.h>

#include <cad_image_markup/optimization/CamPoseReprojectionCost.h>
#include <cad_image_markup/optimization/OrientationPriorCost.h>
#include <cad_image_markup/optimization/PointToLineCost.h>

namespace cad_image_markup {

Solver::Solver(
    const std::shared_ptr<cad_image_markup::CameraModel>& camera_model,
    const Params& params, const std::string& ceres_config_path)
    : camera_model_(camera_model), params_(params) {
  ceres_params_ = optimization::CeresParams(ceres_config_path);

  // Initialize visualizer directly in Solver
  visualizer_ = std::make_shared<Visualizer>("solution visualizer");
  source_cloud_ = params_.source_cloud;
}

bool Solver::Solve(PointCloud::ConstPtr cad_cloud,
                   PointCloud::ConstPtr camera_cloud,
                   Eigen::Matrix4d& T_WORLD_CAMERA, bool visualize) {
  cad_cloud_ = cad_cloud;
  camera_cloud_ = camera_cloud;

  LOG_INFO("SOLVER: Initiated");

  solution_iterations_ = 0;
  last_iteration_cost_ = 0;

  bool has_converged = false;

  Eigen::Quaterniond results_init_rotation;
  Eigen::Vector3d results_init_translation;

  utils::TransformMatrixToQuaternionAndTranslation(
      T_WORLD_CAMERA, results_init_rotation, results_init_translation);

  results_ = {results_init_rotation.w(),    results_init_rotation.x(),
              results_init_rotation.y(),    results_init_rotation.z(),
              results_init_translation.x(), results_init_translation.y(),
              results_init_translation.z()};

  PointCloud::Ptr CAD_cloud_scaled = std::make_shared<PointCloud>(*cad_cloud);
  utils::ScaleCloud(CAD_cloud_scaled, 1.0);

  // correspondence object tells the cost function which points to compare
  pcl::CorrespondencesPtr proj_corrs(new pcl::Correspondences);

  // transform, project, and get correspondences
  int num_correspondences;
  if (params_.correspondence_type == CorrespondenceType::P2POINT) {
    num_correspondences = 1;
  } else if (params_.correspondence_type == CorrespondenceType::P2LINE) {
    num_correspondences = 2;
  }

  num_correspondences == 1
      ? LOG_INFO("SOLVER: Selected Point to Point Correspondences")
      : LOG_INFO("SOLVER: Selected Point to Line Correspondences");

  double max_corr_distance = params_.max_corr_distance;

  utils::CorrespondenceEstimate(CAD_cloud_scaled, camera_cloud_, T_WORLD_CAMERA,
                                proj_corrs, params_.align_centroids,
                                max_corr_distance, num_correspondences,
                                source_cloud_, camera_model_);

  LOG_INFO("SOLVER: Initial Correspondences Estimated: %ld",
           proj_corrs->size());

  if (params_.visualize) visualizer_->StartVis();

  // loop problem until it has converged
  while (!has_converged &&
         solution_iterations_ < params_.max_solution_iterations) {
    solution_iterations_++;

    if (params_.visualize) {
      if (!UpdateVisualizer(CAD_cloud_scaled, T_WORLD_CAMERA, proj_corrs))
        return false;
    }

    std::shared_ptr<ceres::Problem> problem = BuildCeresProblem(
        proj_corrs, camera_model_, camera_cloud_, CAD_cloud_scaled);

    T_WORLD_CAMERA = utils::QuaternionAndTranslationToTransformMatrix(results_);

    // attenuate the correspondence distance if this option selected
    if (params_.attenuate_corr_distance)
      max_corr_distance = (max_corr_distance - params_.corr_bound_low) *
                              params_.attenuation_rate +
                          params_.corr_bound_low;

    // transform, project, and get correspondences
    utils::CorrespondenceEstimate(
        CAD_cloud_scaled, camera_cloud_, T_WORLD_CAMERA, proj_corrs,
        params_.align_centroids, max_corr_distance, num_correspondences,
        source_cloud_, camera_model_);

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

std::shared_ptr<ceres::Problem> Solver::BuildCeresProblem(
    pcl::CorrespondencesPtr proj_corrs,
    std::shared_ptr<cad_image_markup::CameraModel> camera_model,
    PointCloud::ConstPtr camera_cloud, PointCloud::ConstPtr cad_cloud) {
  if (params_.output_results) { LOG_INFO("SOLVER: Building ceres problem..."); }

  // set ceres problem options
  ceres::Problem::Options ceres_problem_options;

  // if we want to manage our own data for these, we can set these flags:
  ceres_problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres_problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;

  // initialize problem
  problem_ = std::make_shared<ceres::Problem>(ceres_problem_options);

  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_problem_options);

  std::unique_ptr<ceres::LocalParameterization> parameterization =
      ceres_params_.SE3QuatTransLocalParametrization();

  problem->AddParameterBlock(&(results_[0]), 7, parameterization.get());

  if (params_.output_results) { LOG_INFO("SOLVER: Added Parameter Block..."); }

  std::unique_ptr<ceres::LossFunction> loss_function =
      ceres_params_.LossFunction();

  // add pose prior
  if (params_.prior_information_weight != 0) {
    Eigen::Vector4d q(results_[0], results_[1], results_[2], results_[3]);
    Eigen::Matrix3d sqrt_inv_cov =
        params_.prior_information_weight * Eigen::Matrix3d::Identity();
    std::unique_ptr<ceres::CostFunction> cost_function(
        OrientationPriorFunctor::Create(sqrt_inv_cov, q));
    problem->AddResidualBlock(cost_function.release(), loss_function.get(),
                              &(results_[0]));
  }

  // add residuals
  for (int i = 0; i < proj_corrs->size(); i++) {
    Eigen::Vector2d pixel(camera_cloud->at(proj_corrs->at(i).index_query).x,
                          camera_cloud->at(proj_corrs->at(i).index_query).y);

    Eigen::Vector3d p1(cad_cloud->at(proj_corrs->at(i).index_match).x,
                       cad_cloud->at(proj_corrs->at(i).index_match).y,
                       cad_cloud->at(proj_corrs->at(i).index_match).z);

    // Add the appropriate cost function and loss function
    if (params_.correspondence_type == CorrespondenceType::P2POINT) {
      std::unique_ptr<ceres::CostFunction> cost_function(
          CeresReprojectionCostFunction::Create(pixel, p1, camera_model_));
      problem->AddResidualBlock(cost_function.release(), loss_function.get(),
                                &(results_[0]));
    } else if (params_.correspondence_type == CorrespondenceType::P2LINE) {
      i++;
      Eigen::Vector3d p2(cad_cloud->at(proj_corrs->at(i).index_match).x,
                         cad_cloud->at(proj_corrs->at(i).index_match).y,
                         cad_cloud->at(proj_corrs->at(i).index_match).z);
      std::unique_ptr<ceres::CostFunction> cost_function(
          point_to_line_cost::CeresReprojectionCostFunction::Create(
              pixel, p1, p2, camera_model_));
      problem->AddResidualBlock(cost_function.release(), loss_function.get(),
                                &(results_[0]));
    }
  }

  LOG_INFO("SOLVER: Finished Building Ceres Problem");
  SolveCeresProblem(problem);
  return problem;
}

void Solver::SolveCeresProblem(const std::shared_ptr<ceres::Problem>& problem) {
  if (params_.output_results) {
    LOG_INFO("SOLVER: Solving ceres problem...");
    ceres::Solve(ceres_params_.SolverOptions(), problem.get(), &summary_);
    LOG_INFO("SOLVER: Finished solving ceres problem");
    LOG_INFO("SOLVER: Outputting ceres summary:");
    std::string report = summary_.FullReport();
    std::cout << report << "\n";
  } else {
    ceres::Solve(ceres_params_.SolverOptions(), problem.get(), &summary_);
  }
}

bool Solver::HasConverged() {
  // Cannot converge on a single solver iteration
  if (solution_iterations_ <= 1) return false;

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
    if (params_.convergence_condition == ConvergenceCondition::ABSOLUTE)
      LOG_ERROR("SOLVER: Cannot converge to absolute geometric condition, set "
                "convergence condition to DIFFERENCE in the solution "
                "parameters file");

    Eigen::Quaterniond q_current{results_[0], results_[1], results_[2],
                                 results_[3]};
    Eigen::Vector3d euler_angles_current =
        q_current.toRotationMatrix().eulerAngles(0, 1, 2);

    // convert to degrees
    euler_angles_current[0] = utils::RadToDeg(euler_angles_current[0]);
    euler_angles_current[1] = utils::RadToDeg(euler_angles_current[1]);
    euler_angles_current[2] = utils::RadToDeg(euler_angles_current[2]);

    Eigen::Quaterniond q_last{
        last_iteration_results_[0], last_iteration_results_[1],
        last_iteration_results_[2], last_iteration_results_[3]};
    Eigen::Vector3d euler_angles_last =
        q_last.toRotationMatrix().eulerAngles(0, 1, 2);

    // convert to degrees
    euler_angles_last[0] = utils::RadToDeg(euler_angles_last[0]);
    euler_angles_last[1] = utils::RadToDeg(euler_angles_last[1]);
    euler_angles_last[2] = utils::RadToDeg(euler_angles_last[2]);

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

    // return false;
  }

  // update for the next iteration
  last_iteration_cost_ = summary_.final_cost;

  return false;
}

bool Solver::UpdateVisualizer(PointCloud::Ptr CAD_cloud_scaled,
                              Eigen::Matrix4d& T_WORLD_CAMERA,
                              pcl::CorrespondencesPtr proj_corrs) {
  // update the position of the transformed cloud based on
  // the upated transformation matrix for visualization
  PointCloud::Ptr cloud_in_world = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*CAD_cloud_scaled, *cloud_in_world, T_WORLD_CAMERA);

  // project cloud for visualizer
  PointCloud::Ptr proj_cloud =
      utils::ProjectCloud(cloud_in_world, camera_model_);

  // blow up the transformed cloud for visualization
  utils::ScaleCloud(cloud_in_world, 1.0 / params_.cad_cloud_scale);

  visualizer_->DisplayClouds(camera_cloud_, cloud_in_world, proj_cloud,
                             proj_corrs, "camera_cloud", "transformed_cloud",
                             "projected_cloud", source_cloud_);

  // wait on user input to continue or cancel the solution
  char end = ' ';

  std::cout << "Enter 'n' to continue or 'q' to quit\n";
  while (end != 'n' && end != 'r') { cin >> end; }

  if (end == 'q') return false;

  std::cout << "continuing to next iteration...\n";
  return true;
}

} // namespace cad_image_markup