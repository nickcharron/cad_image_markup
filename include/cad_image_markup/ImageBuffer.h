#pragma once

#include <cstdint>
#include <fstream>
#include <math.h>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <cad_image_markup/nlohmann/json.h>

#include <cad_image_markup/Utils.h>


namespace cad_image_markup {

/**
 * @brief Class containing input/ouput operations for reading and converting
 * labelled image data and writing to ouput images
 */
class ImageBuffer {
 public:
  /**
   * @brief Default constructor
   */
  ImageBuffer() = default;

  /**
   * @brief Default destructor
   */
  ~ImageBuffer() = default;

  /**
   * @brief Method for reading labelled image feature data from a json file
   * @param filename_ absolute path to the json file to read data from
   * @param points pointcloud of points to fill in
   * @return read success
   */
  bool ReadPoints(const std::string& filename,
                  PointCloud::Ptr points);

  /**
   * @brief Method for reading labelled data from a png file 
   * @param filename_ absolute path to the png file to read the data from
   * @param points pointcloud of points to fill in
   * @param color pixel color to extract, options: "red", "green", "blue", "white", "black"
   * @return read success
   */
  bool ReadPointsPNG(const std::string& filename,
                  PointCloud::Ptr points, std::string color);

  /**
   * @brief Method for interpolating points for a more
   * dense outline of a feature - helps to converge minimization solution
   * @param points feature points
   * @param density_index number of points to add for every ten pixels
   */
  void DensifyPoints(PointCloud::Ptr points,
                     uint8_t density_index);

  /**
   * @brief Method for scaling the 2D feature points wrt the origin (top let
   * corner) of the original image
   * @param points pointcloud of feature points
   * @param scale scaling factor (output_point = input_point * scale_)
   */
  void ScalePoints(PointCloud::Ptr points, float scale);

  /**
   * @brief Method for writing 2D point set data to an image by setting
   * corresponding pixels to a specified color
   * @param points 2D point set
   * @param src_file_name absolute name of unannotated image
   * @param target_file_name absolute path of image annotated image to create
   * (unnanotated image with written data overlayed)
   * @param r red uint8_t
   * @param g green uint8_t
   * @param b blue uint8_t
   * @return write success
   * @todo update to interpolate pixels (lines? splines?) to color between
   * points
   */
  bool WriteToImage(PointCloud::Ptr points, const std::string& src_file_name,
                    const std::string& target_file_name,
                    uint8_t r = 0, uint8_t g = 0, uint8_t b = 0);
};

}  // namespace cad_image_markup
