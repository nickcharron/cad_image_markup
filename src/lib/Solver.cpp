#include <cad_image_markup/Solver.h>

namespace cad_image_markup {

Solver::Solver(std::shared_ptr<beam_calibration::CameraModel> camera_model,
               const Params& params)
    : camera_model_(camera_model), params_(params) {
  ceres_params_ = optimization::CeresParams(params_.ceres_params_path);
  visualizer_ = std::make_shared<Visualizer>("solution visualizer"); // Initialize visualizer directly in Solver
}

Solver::Solver(std::shared_ptr<beam_calibration::CameraModel> camera_model, )
    : camera_model_(camera_model) {
      visualizer_ = std::make_shared<Visualizer>("solution visualizer"); // Initialize visualizer directly in Solver 
}

bool Solve(PointCloud::ConstPtr cad_cloud, PointCloud::ConstPtr camera_cloud,
           const Eigen::Matix4d& T_WORLD_CAMERA) {
  cad_cloud_ = cad_cloud;
  camera_cloud_ = camera_cloud;             
  // TODO CAM: update this:
  // ALSO: create a separate function for calculating correspondencs to make this more clear

  /*
  bool has_converged = false;

  PointCloud::Ptr CAD_cloud_scaled = std::make_shared<PointCloud>();
  PointCloud::Ptr trans_cloud = std::make_shared<PointCloud>();
  PointCloud::Ptr proj_cloud = std::make_shared<PointCloud>();

  // correspondence object tells the cost function which points to compare
  pcl::CorrespondencesPtr proj_corrs = std::make_shared<pcl::Correspondences>();

  CAD_cloud_scaled = util->ScaleCloud(CAD_cloud_, cloud_scale_);

  if (visualize_) vis->startVis();

  // transform, project, and get correspondences
  util->CorrEst(CAD_cloud_scaled, camera_cloud_, T_CS, proj_corrs,
                offset_type_);

  // transformed cloud is only for the visualizer,
  // the actual ceres solution takes just the original
  // CAD cloud and the iterative results
  trans_cloud = util->TransformCloud(CAD_cloud_scaled, T_CS);

  // project cloud for visualizer
  proj_cloud = util->ProjectCloud(trans_cloud);

  // blow up the transformed cloud for visualization
  util->ScaleCloud(trans_cloud, (1 / cloud_scale_));

  // set initial error before optimizing
  SetInitialPixelError(proj_cloud, camera_cloud_, proj_corrs);

  // loop problem until it has converged
  while (!has_converged && solution_iterations_ < max_solution_iterations_) {
    solution_iterations_++;

    printf("Solver iteration %u \n", solution_iterations_);

    if (visualize_) {
      vis->displayClouds(camera_cloud_, trans_cloud, proj_cloud, proj_corrs,
                         "camera_cloud", "transformed_cloud",
                         "projected_cloud");

      char end = ' ';

      while (end != 'n' && end != 'r') {
        cin >> end;
      }

      if (end == 'r') return false;
    }

    BuildCeresProblem(problem, proj_corrs, camera_model, camera_cloud_,
                      CAD_cloud_scaled);

    SolveCeresProblem(problem, minimizer_progress_to_stdout_);

    T_CS = util->QuaternionAndTranslationToTransformMatrix(results);

    if (transform_progress_to_stdout_) {
      std::string sep = "\n----------------------------------------\n";
      std::cout << T_CS << sep;
    }

    // transform, project, and get correspondences
    util->CorrEst(CAD_cloud_scaled, camera_cloud_, T_CS, proj_corrs, "none");

    // update the position of the transformed cloud based on
    // the upated transformation matrix for visualization
    trans_cloud = util->TransformCloud(CAD_cloud_scaled, T_CS);

    // project cloud for visualizer
    proj_cloud = util->ProjectCloud(trans_cloud);

    // blow up the transformed CAD cloud for visualization
    util->ScaleCloud(trans_cloud, (1 / cloud_scale_));

    if (convergence_type_ == "PIXEL")
      has_converged = CheckPixelConvergence(proj_cloud, camera_cloud_,
                                            proj_corrs, convergence_limit_);
  }

  if (visualize_) vis->endVis();
  if (has_converged)
    return true;
  else
    return false;
  */
  return false;
}

Eigen::Matrix4d Solver::GetT_WORLD_CAMERA() { return T_WORLD_CAMERA_; }

ResultsSummary Solver::GetResultsSummary() { return summary_; }

void Solver::BuildCeresProblem() {
  if (params_.output_results) {
    LOG_INFO("Building ceres problem...");
  }
  // initialize problem
  problem_ = std::make_shared<ceres::Problem>(ceres_params_.ProblemOptions());

  std::unique_ptr<ceres::LocalParameterization> parameterization =
      ceres_params_.SE3QuatTransLocalParametrization();
  problem_->AddParameterBlock(&(results[0]), 7, parameterization.get());

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
                               &(results[0]));
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
    // TODO CAM: implement this. See params for more details.
    // if(summary_.ceres_results.initial_losss - final_loss > amount){
    //     return true;
    // }
}

}  // namespace cad_image_markup