#define CATCH_CONFIG_MAIN

#include <ImageDefectExtractor.h>
#include <cad_image_markup/Utils.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <catch2/catch.hpp>

std::string GetDataPathRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "tests/defect_extractor_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "examples/example_data/";
}

std::string GetOutputRoot() {
  std::string file_location = __FILE__;
  std::string current_file_path = "defect_extractor_tests.cpp";
  file_location.erase(file_location.end() - current_file_path.length(),
                      file_location.end());
  return file_location + "test_outputs/";
}



TEST_CASE("Build test.") {

  std::string data_root = GetDataPathRoot();
  std::string output_root = GetOutputRoot();
  std::string image_path = data_root + "images/real_view_1_image.png";
  std::string output_path = output_root + "defect_extractor_test_output.json";
  std::string defect_color = "red";


  cad_image_markup::ImageDefectExtractor extractor(image_path, defect_color);

  REQUIRE(1);
}

TEST_CASE("Generate Output test.") {

  std::string data_root = GetDataPathRoot();
  std::string output_root = GetOutputRoot();
  std::string image_path = data_root + "marked_up_images/sim_view_2_image_markup.png";
  std::string output_path = output_root + "defect_extractor_test_output.json";
  std::string verification_image_path = output_root + "defect_extractor_verification_image.png";
  std::string defect_color = "red";


  cad_image_markup::ImageDefectExtractor extractor(image_path, defect_color);

  cad_image_markup::PointCloud::Ptr points = std::make_shared<cad_image_markup::PointCloud>();


  REQUIRE(extractor.ExtractDefects());

  REQUIRE(extractor.SaveResults(output_path));

  REQUIRE(cad_image_markup::utils::ReadPoints(output_path,points));

  REQUIRE(cad_image_markup::utils::WriteToImage(points,image_path,verification_image_path,255,255,255));


}
