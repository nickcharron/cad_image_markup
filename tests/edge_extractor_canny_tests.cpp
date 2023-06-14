#define CATCH_CONFIG_MAIN

#include <EdgeExtractorCanny.h>

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

TEST_CASE("Output test.") {

  std::string data_root = GetDataPathRoot();
  std::string config_root = GetConfigPathRoot();
  std::string output_root = GetOutputRoot();
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string config_path = config_root + "CannyParamsDefault.json";
  std::string output_path = output_root + "edge_extractor_canny_test_output.json";

  cad_image_markup::EdgeExtractorCanny extractor(image_path, config_path);

  REQUIRE(1);

  REQUIRE(extractor.SaveResults(output_path));
}
