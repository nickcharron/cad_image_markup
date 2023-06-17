#define CATCH_CONFIG_MAIN

#include <cad_image_markup/CadImageMarkup.h>
#include <cad_image_markup/Utils.h>

#include <catch2/catch.hpp>

std::string GetDataPathRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "tests/align_and_markup_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "examples/example_data/";
}

std::string GetConfigPathRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "tests/align_and_markup_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "config/";
}

std::string GetOutputRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "align_and_markup_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "test_outputs/";
}


TEST_CASE("Build test.") {

  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  std::string image_label_path = data_root + "labelled_images/real_view_1_feature_label.json";
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string cad_label_path = data_root + "labelled_images/sim_view_1_feature_label.json";
  std::string cad_image_path = data_root + "cad/real_cad.png";
  std::string defect_path = data_root + "marked_up_images/real_view_1_image.png";
  std::string intrinsics_path = data_root + "Radtan_intrinsics_phone.json"; 
  std::string initial_pose = data_root + "poses/initial_pose.json";
  std::string solution_config_path = config_root + "SolutionParamsDefault.json";
  std::string ceres_config_path = config_root + "CeresParamsDefault.json";


  cad_image_markup::CadImageMarkup::Inputs inputs{
    .image_label_path = image_label_path,
    .image_path = image_path,
    .cad_label_path = cad_label_path,
    .cad_image_path = cad_image_path,
    .defect_path = defect_path,
    .intrinsics_path = intrinsics_path,
    .initial_pose_path = initial_pose,
    .solution_config_path = solution_config_path,
    .ceres_config_path = ceres_config_path};

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(1);
}


// NOTE: To run this test, the Setup() function in the CadImageMarkup class must be 
//       made public temporarily
TEST_CASE("Setup Test.") {

  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  std::string image_label_path = data_root + "labelled_images/real_view_1_feature_label.json";
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string cad_label_path = data_root + "labelled_images/sim_view_1_feature_label.json";
  std::string cad_image_path = data_root + "cad/real_cad.png";
  std::string defect_path = data_root + "marked_up_images/real_view_1_image.png";
  std::string intrinsics_path = data_root + "Radtan_intrinsics_phone.json"; 
  std::string initial_pose = data_root + "poses/initial_pose.json";
  std::string solution_config_path = config_root + "SolutionParamsDefault.json";
  std::string ceres_config_path = config_root + "CeresParamsDefault.json";


  cad_image_markup::CadImageMarkup::Inputs inputs{
    .image_label_path = image_label_path,
    .image_path = image_path,
    .cad_label_path = cad_label_path,
    .cad_image_path = cad_image_path,
    .defect_path = defect_path,
    .intrinsics_path = intrinsics_path,
    .initial_pose_path = initial_pose,
    .solution_config_path = solution_config_path,
    .ceres_config_path = ceres_config_path};

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(markup.Setup());
}

// NOTE: To run this test, the Setup() and LoadData() functions in the CadImageMarkup class must be 
//       made public temporarily
TEST_CASE("Setup and load data test.") {

  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  std::string image_label_path = data_root + "labelled_images/real_view_1_feature_label.json";
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string cad_label_path = data_root + "labelled_images/sim_view_1_feature_label.json";
  std::string cad_image_path = data_root + "cad/real_cad.png";
  std::string defect_path = data_root + "marked_up_images/real_view_1_image.png";
  std::string intrinsics_path = data_root + "Radtan_intrinsics_phone.json"; 
  std::string initial_pose = data_root + "poses/initial_pose.json";
  std::string solution_config_path = config_root + "SolutionParamsDefault.json";
  std::string ceres_config_path = config_root + "CeresParamsDefault.json";


  cad_image_markup::CadImageMarkup::Inputs inputs{
    .image_label_path = image_label_path,
    .image_path = image_path,
    .cad_label_path = cad_label_path,
    .cad_image_path = cad_image_path,
    .defect_path = defect_path,
    .intrinsics_path = intrinsics_path,
    .initial_pose_path = initial_pose,
    .solution_config_path = solution_config_path,
    .ceres_config_path = ceres_config_path};

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(markup.Setup());

  REQUIRE(markup.LoadData());
}

TEST_CASE("Run test.") {

  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  std::string image_label_path = data_root + "labelled_images/real_view_1_feature_label.json";
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string cad_label_path = data_root + "labelled_images/real_cad_feature_label.json";
  std::string cad_image_path = data_root + "cad/real_cad.png";
  std::string defect_path = data_root + "marked_up_images/real_view_1_image.png";
  std::string intrinsics_path = data_root + "Radtan_intrinsics_phone.json"; 
  std::string initial_pose = data_root + "poses/initial_pose.json";
  std::string solution_config_path = config_root + "SolutionParamsDefault.json";
  std::string ceres_config_path = config_root + "CeresParamsDefault.json";

  cad_image_markup::CadImageMarkup::Inputs inputs{
    .image_label_path = image_label_path,
    .image_path = image_path,
    .cad_label_path = cad_label_path,
    .cad_image_path = cad_image_path,
    .defect_path = defect_path,
    .intrinsics_path = intrinsics_path,
    .initial_pose_path = initial_pose,
    .solution_config_path = solution_config_path,
    .ceres_config_path = ceres_config_path};

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(markup.Run());
}

TEST_CASE("Save results test.") {

  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  std::string image_label_path = data_root + "labelled_images/real_view_1_feature_label.json";
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string cad_label_path = data_root + "labelled_images/real_cad_feature_label.json";
  std::string cad_image_path = data_root + "cad/real_cad.png";
  std::string defect_path = data_root + "images/real_view_1_image.png";
  std::string intrinsics_path = data_root + "Radtan_intrinsics_phone.json"; 
  std::string initial_pose = data_root + "poses/initial_pose.json";
  std::string solution_config_path = config_root + "SolutionParamsDefault.json";
  std::string ceres_config_path = config_root + "CeresParamsDefault.json";

  std::string output_directory = GetOutputRoot();

  cad_image_markup::CadImageMarkup::Inputs inputs{
    .image_label_path = image_label_path,
    .image_path = image_path,
    .cad_label_path = cad_label_path,
    .cad_image_path = cad_image_path,
    .defect_path = defect_path,
    .intrinsics_path = intrinsics_path,
    .initial_pose_path = initial_pose,
    .solution_config_path = solution_config_path,
    .ceres_config_path = ceres_config_path};

  cad_image_markup::CadImageMarkup markup(inputs);

  REQUIRE(markup.Run());
  REQUIRE(markup.SaveResults(output_directory));
}

