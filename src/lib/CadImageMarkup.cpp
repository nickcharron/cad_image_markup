#include <cad_image_markup/CadImageMarkup.h>

#include <boost/filesystem.hpp>


namespace cad_image_markup {

#define LOSS_CONVERGENCE 0 
#define GEO_CONVERGENCE 1

#define P2POINT 1
#define P2PLANE 2

#define DIFF_CONVERGENCE 0
#define ABS_CONVERGENCE 1

//struct Params; //forward declaration of params struct type

bool Params::LoadFromJson(const std::string& path) {
  LOG_INFO("Loading config file from: %s", path.c_str());
  // TODO CAM: add function for loading json config settings. Also add a
  // ConfigDefault.json in the config folder of this repo. Look at examples in
  // libbeam for parsing jsons

  nlohmann::json J;
  std::ifstream file(path);
  file >> J;

  nlohmann::json J_solution_options = J["solver_options"];
  cad_cloud_scale = J_solution_options["cad_cloud_scale"]; 
  max_solution_iterations = J_solution_options["max_solution_iterations"];
  visualize = J_solution_options["visualize"];
  output_results = J_solution_options["output_results"];
  align_centroids = J_solution_options["align centroids"];

  if (J_solution_options["correspondence_type"] == "P2POINT")
    correspondence_type = P2POINT;
  else if (J_solution_options["correspondence_type"] == "P2PLANE")
    correspondence_type = P2PLANE;
  else{
    LOG_ERROR("Invalid correspondence type specified. Exiting...");
    return false;
  }
    
  max_corr_distance = J_solution_options["max_corr_distance"];

  nlohmann::json J_convergence_options = J["solver_options"];
  if (J_convergence_options["convergence_type"] == "LOSS_CONVERGENCE")
    convergence_type = LOSS_CONVERGENCE;
  else if (J_convergence_options["convergence_type"] == "GEO_CONVERGENCE")
    convergence_type = GEO_CONVERGENCE;
  else{
    LOG_ERROR("Invalid convergence type speficied. Exiting...");
    return false;
  }
    

  if (J_convergence_options["convergence_condition"] == "ABS_CONVERGENCE" &&
      convergence_type == GEO_CONVERGENCE){
    LOG_ERROR("Absolute convergence condition is not available for geometric convergence type. Exiting...");
    return false;
  }
  if (J_convergence_options["convergence_condition"] == "ABS_CONVERGENCE" &&
      convergence_type != GEO_CONVERGENCE) {
    convergence_condition = ABS_CONVERGENCE;
  }
  else if (J_convergence_options["convergence_condition"] == "DIFF_CONVERGENCE")
    convergence_condition = DIFF_CONVERGENCE;
  else 
    LOG_ERROR("Invalid convergence condition speficied. Exiting...");

  convergence_condition = J_convergence_options["convergence_condition"];
  converged_differential_cost = J_convergence_options["converged_differential_cost"];
  converged_absolute_cost = J_convergence_options["converged_absolute_cost"];
  converged_differential_translation = J_convergence_options["converged_differential_translation"];
  converged_differential_rotation = J_convergence_options["convergend_differential_rotation"];

  ceres_params_path = J["ceres_config_path"];

  return true;
}

CadImageMarkup::CadImageMarkup(const Inputs& inputs, Params& params) : inputs_(inputs), params_(params){}

bool CadImageMarkup::Run() {
  if (!Setup()) {
    return false;
  }

  if (!LoadData()) {
    return false;
  }

  //if (!Solve()) {
  //  return false;
  //}

  return true;
}

bool CadImageMarkup::Setup() {
  
  //camera_points_CAMFRAME_ = std::make_shared<PointCloud>();
  PointCloud::Ptr camera_points_CAMFRAME_ (new PointCloud);
  //cad_points_CADFRAME_ = std::make_shared<PointCloud>();
  PointCloud::Ptr cad_points_CADFRAME_ (new PointCloud);

  params_.LoadFromJson(inputs_.config_path);

  std::shared_ptr<CameraModel> camera_model = CameraModel::Create(inputs_.intrinsics_path);

  if (!inputs_.config_path.empty()) {
    if (!boost::filesystem::exists(inputs_.config_path)) {
      LOG_ERROR("Invalid path to config file: %s", inputs_.config_path.c_str());
      return false;
    }
    if (!params_.LoadFromJson(inputs_.config_path)) {
      return false;
    }
  }

  Params* params_ptr;

  params_ptr = &params_;

  solver_ = std::make_unique<Solver>(camera_model,params_ptr);

}

bool CadImageMarkup::LoadData() {
  // read image points
  if (!image_buffer_.ReadPoints(inputs_.image_path, camera_points_CAMFRAME_)) {
    LOG_ERROR("Cannot read image file at: %s", inputs_.image_path.c_str());
    return false;
  }

  // read cad model points
  if (!image_buffer_.ReadPoints(inputs_.cad_path, cad_points_CADFRAME_)) {
    LOG_ERROR("Cannot read CAD file at: %s", inputs_.cad_path.c_str());
    return false;
  }

  // densify points
  image_buffer_.DensifyPoints(camera_points_CAMFRAME_, params_.cam_density_index);
  image_buffer_.DensifyPoints(cad_points_CADFRAME_, params_.cad_density_index);

  // TODO CAM: I don't understand why we'd need to do this?
  // Based on our convo: we want to calculate T_WORLD_CAD where the world frame
  // is the centroid of the object, and the cad frame is the top left corner. To
  // calculate this, just get the translation in x and y to the centroid. Then I
  // think we can remove this function

  // CAM NOTE: This is just how I am doing that, the T_WORLD_CAMERA needs to 
  // operate initially on the cad cloud with its centroid aligned with the camera,
  // when the cad cloud and back-projected defects are flattened, shifting back by
  // the origin coodinates puts everything back in CAD frame
  cad_centroid_ = utils::GetCloudCentroid(cad_points_CADFRAME_);
  utils::OriginCloudxy(cad_points_CADFRAME_, cad_centroid_);

  return true;
}

bool CadImageMarkup::Solve() {
  Eigen::Matrix4d T_WORLD_CAMERA_init;
  utils::LoadInitialPose(inputs_.initial_pose_path,T_WORLD_CAMERA_init);


  bool converged = solver_->Solve(cad_points_WORLDFRAME_,
                                 camera_points_CAMFRAME_, 
                                 T_WORLD_CAMERA_init, params_.visualize);

  if (!converged) {
    LOG_ERROR("Solver failed, exiting.");
    return false;
  }
  LOG_INFO("Solver successful.");

  // TODO CAM: output results here? We can just create a new CAD image with the
  // markups, and rename it using the orginal name. We should also output other
  // things like:
  // * T_WORLD_CAMERA_final
  // * T_WORLD_CAMERA_initial
  // * Intrinsics used
  // * config copies (so we can go back and see the results that produced that)
  // Overall structure would be:
  // path_to_cad_in/cad_name.json
  //               /cad_name_results/
  //                                T_WORLD_CAMERA_final.json
  //                                T_WORLD_CAMERA_initial.json
  //                                intrinsics.json
  //                                config.json
  //                                ceres_config.json
  Eigen::Matrix4d T_WORLD_CAMERA = solver_->GetT_WORLD_CAMERA();
  std::cout << "T_WORLD_CAMERA: \n" << T_WORLD_CAMERA << "\n";
}

}  // namespace cad_image_markup