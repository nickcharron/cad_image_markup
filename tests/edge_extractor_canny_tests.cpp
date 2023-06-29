#define CATCH_CONFIG_MAIN

#include <EdgeExtractorCanny.h>
#include <cad_image_markup/Utils.h>

#include <catch2/catch.hpp>

std::string GetDataPathRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "tests/edge_extractor_canny_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "examples/example_data/";
}

std::string GetConfigPathRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "tests/edge_extractor_canny_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "config/";
}

std::string GetOutputRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "edge_extractor_canny_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "test_outputs/";
}



TEST_CASE("Build test.") {

  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string config_path = config_root + "CannyParamsDefault.json";

  cad_image_markup::EdgeExtractorCanny extractor(image_path, config_path);

  REQUIRE(1);
}

TEST_CASE("Generate Output test.") {

  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  std::string output_root = GetOutputRoot();
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string config_path = config_root + "CannyParamsDefault.json";
  std::string output_path = output_root + "edge_extractor_canny_test_output.json";

  cad_image_markup::EdgeExtractorCanny extractor(image_path, config_path);

  REQUIRE(1);

  extractor.ExtractEdges();

  REQUIRE(extractor.SaveResults(output_path));
}

TEST_CASE("Verify JSON test.") {
  
  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  std::string output_root = GetOutputRoot();
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string config_path = config_root + "CannyParamsDefault.json";
  std::string output_path = output_root + "edge_extractor_canny_test_output.json";

  cad_image_markup::EdgeExtractorCanny extractor(image_path, config_path);

  REQUIRE(1);

  extractor.ExtractEdges();

  REQUIRE(extractor.SaveResults(output_path));

  // check retrieval of edges from JSON file:

  cad_image_markup::PointCloud::Ptr edges_ground_truth = extractor.GetEdgesCloud(); 

  cad_image_markup::PointCloud::Ptr edges_cloud_retrieved = std::make_unique<cad_image_markup::PointCloud>();

  cad_image_markup::utils::ReadPoints(output_path, edges_cloud_retrieved);

  REQUIRE(edges_ground_truth->size() == edges_cloud_retrieved->size());

  bool clouds_equivalent = true;

  // Assuming points read in the same order they are written
  for (int i = 0; i<edges_ground_truth->size(); i++) {
    std::vector<int> ground_point = {(int)edges_ground_truth->at(i).x, (int)edges_ground_truth->at(i).y};
    std::vector<int> retrieved_point = {(int)edges_cloud_retrieved->at(i).x, (int)edges_cloud_retrieved->at(i).y};

    if (ground_point != retrieved_point)
      clouds_equivalent = false;
  }

  REQUIRE(clouds_equivalent);

}