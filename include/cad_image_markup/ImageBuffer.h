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
   * @todo update to handle points from multiple labels in an image
   */
  bool ReadPoints(const std::string& filename,
                  PointCloud::Ptr points);

  /**
   * @brief Method for interpolating points for a more
   * dense outline of a feature - helps to converge minimization solution
   * @param points feature points
   * @param density_index number of points to add for every ten pixels
   * @todo update to add interpolated points at equal intervals rather than at
   * fractions of the length between labelled points
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
   * @param points_ 2D point set
   * @param src_file_name_ absolute name of unannotated image
   * @param target_file_name_ absolute path of image annotated image to create
   * (unnanotated image with written data overlayed)
   * @param color_ color to set pixels in output image (default = "black",
   * options = "red", "green", "blue")
   * @return write success
   * @todo update to interpolate pixels (lines? splines?) to color between
   * points
   */
  bool WriteToImage(std::vector<point>* points_, std::string src_file_name_,
                    std::string target_file_name_,
                    std::string color_ = "black");
};

}  // namespace cad_image_markup
