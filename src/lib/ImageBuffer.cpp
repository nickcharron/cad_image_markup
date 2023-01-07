#include <cad_image_markup/ImageBuffer.h>

#include <boost/filesystem.hpp>

namespace cad_image_markup {

bool ImageBuffer::ReadPoints(const std::string &filename,
                             PointCloud::Ptr points) {
  points->clear();

  if (!boost::filesystem::exists(filename)) {
    LOG_ERROR("Invalid path to points file: %s", filename.c_str());
    return false;
  }
  LOG_INFO("Loading points file: %s", filename.c_str());

  nlohmann::json J;
  std::ifstream file(filename);
  file >> J;

  nlohmann::json J_shapes = J["shapes"];
  for (auto J_point : J_shapes[0]["points"]) {
    std::vector<float> point_vec;
    for (auto val : J_point) {
      point_vec.push_back(val.get<float>());
    }
    if (point_vec.size() != 2) {
      LOG_ERROR("Invalid point in points file.");
      return false;
    }
    pcl::PointXYZ point_pcl(point_vec.at(0), point_vec.at(1), 0);
    points->push_back(point_pcl);
  }

  return true;
}

bool ImageBuffer::ReadPointsPNG(const std::string& filename, PointCloud::Ptr points, std::string color) {
  if (!boost::filesystem::exists(filename)) 
    return false;

  int threshold = 230;

  cv::Mat img = cv::imread(filename, cv::IMREAD_COLOR);

  LOG_INFO("INPUT BUFFER: opened defect file");

  // get all pixels of specified color 
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {

      std::vector<int> pixel_vals = {img.at<cv::Vec3b>(i,j)[2], img.at<cv::Vec3b>(i,j)[1], img.at<cv::Vec3b>(i,j)[0]};
      pcl::PointXYZ point_pcl(j, i, 0); // [TODO]: check x and y here

      if (color == "red") {
        if (pixel_vals[0] >= threshold && pixel_vals[1] < threshold && pixel_vals[2] < threshold) {
          points->push_back(point_pcl);
        }
          
      }
      else if (color == "green") {
        if (pixel_vals[0] < threshold && pixel_vals[1] >= threshold && pixel_vals[2] < threshold)
          points->push_back(point_pcl);
      }
      else if (color == "blue") {
        if (pixel_vals[0] < threshold && pixel_vals[1] < threshold && pixel_vals[2] >= threshold)
          points->push_back(point_pcl);

      } 
      else if (color == "white") {
        if (pixel_vals[0] >= threshold && pixel_vals[1] >= threshold && pixel_vals[2] >= threshold)
          points->push_back(point_pcl);

      }
      else if (color == "black") {
        if (pixel_vals[0] < (250 -threshold) && pixel_vals[1] < (250 -threshold) && pixel_vals[2] < (250 -threshold))
          points->push_back(point_pcl);
      }
      else {
        LOG_WARN("Invalid color selected for defect detection");
        return false;
      }


    }
  }

  LOG_INFO("INPUT BUFFER: Finished Reading defect pixels");
  LOG_INFO("INPUT BUFFER: %ld defect points read", points->size());

  return true;
  
}

void ImageBuffer::DensifyPoints(PointCloud::Ptr points, uint8_t density_index) {

  // add additional point between existing points according to scale
  // will help to converge solution
  uint16_t init_length = points->size();

  for (uint16_t point_index = 0; point_index < init_length; point_index++) {
    pcl::PointXYZ current_start_point = points->at(0);
    pcl::PointXYZ current_end_point = points->at(1);

    points->erase(points->begin());

    // determine angle between points
    float slope, theta;

    if ((current_end_point.x - current_start_point.x) == 0) {
      theta = M_PI / 2;
    } else if ((current_end_point.y - current_start_point.y) == 0) {
      theta = 0;
    } else {
      slope = (float)(current_end_point.y - current_start_point.y) /
              (float)(current_end_point.x - current_start_point.x);
      theta = std::atan(std::abs(slope));
    }

    // determine distance between points
    float dist = std::sqrt(
        std::pow(std::abs(current_end_point.x - current_start_point.x), 2) +
        std::pow(std::abs(current_end_point.y - current_start_point.y), 2));

    float inter_dist = 10;

    // number of points added between each reference point should be the same
    // for both images for 1:1 mapping (in final solution)
    float interval = inter_dist / (density_index + 1);

    // determine delta x and y values based on quadrant
    float dx, dy;

    // first quadrant:
    if ((current_end_point.y - current_start_point.y) >= 0 &&
        (current_end_point.x - current_start_point.x) >= 0) {
      dx = interval * (std::cos(theta));
      dy = interval * (std::sin(theta));
    }
    // second quadrant
    else if ((current_end_point.y - current_start_point.y) >= 0 &&
             (current_end_point.x - current_start_point.x) < 0) {
      dx = -interval * (std::cos(theta));
      dy = interval * (std::sin(theta));
    }
    // third quadrant
    else if ((current_end_point.y - current_start_point.y) < 0 &&
             (current_end_point.x - current_start_point.x) <= 0) {
      dx = -interval * (std::cos(theta));
      dy = -interval * (std::sin(theta));
    }
    // fourth quadrant
    else if ((current_end_point.y - current_start_point.y) < 0 &&
             (current_end_point.x - current_start_point.x) > 0) {
      dx = interval * (std::cos(theta));
      dy = -interval * (std::sin(theta));
    }

    // push the start point first to conserve the order of the vector
    points->push_back(current_start_point);

    // push the rest of the interpolated points, trending toward the current end
    // point
    float current_x_coord = current_start_point.x + dx;
    float current_y_coord = current_start_point.y + dy;

    float current_dist = std::sqrt(
        std::pow(std::abs(current_x_coord - current_start_point.x), 2) +
        std::pow(std::abs(current_y_coord - current_start_point.y), 2));

    while (current_dist < dist) {
      pcl::PointXYZ current_inter_point(current_x_coord, current_y_coord, 0);
      points->push_back(current_inter_point);

      current_x_coord += dx;
      current_y_coord += dy;

      current_dist = std::sqrt(
          std::pow(std::abs(current_x_coord - current_start_point.x), 2) +
          std::pow(std::abs(current_y_coord - current_start_point.y), 2));
    }
  }
}

void ImageBuffer::ScalePoints(PointCloud::Ptr points, float scale) {
  // scale points based on image scale (for CAD images)
  PointCloud points_orig = *points;
  points->clear();
  for (uint16_t point_index = 0; point_index < points_orig.size();
       point_index++) {
    pcl::PointXYZ current_point = points->at(0);
    pcl::PointXYZ scaled_point(current_point.x * scale, current_point.y * scale,
                               0);
    points->push_back(scaled_point);
  }
}

bool ImageBuffer::WriteToImage(PointCloud::Ptr points,
                               const std::string &src_file_name,
                               const std::string &target_file_name, uint8_t r,
                               uint8_t g, uint8_t b) {
  // Note opencv use BGR not RGB
  cv::Vec3b color;
  color[0] = b;
  color[1] = g;
  color[2] = r;

  if (!boost::filesystem::exists(src_file_name)) {
    LOG_ERROR("Invalid path to input image: %s", src_file_name.c_str());
    return false;
  }
  LOG_INFO("Reading image: %s", src_file_name.c_str());

  cv::Mat image;
  image = cv::imread(src_file_name, 1);
  for (uint32_t i = 0; i < points->size(); i++) {
    if (points->at(i).x >= 0 && points->at(i).x <=  image.cols && points->at(i).y >= 0 && points->at(i).y <= image.rows) 
      image.at<cv::Vec3b>(points->at(i).y, points->at(i).x) = color;
  }

  LOG_INFO("Saving image to: %s", target_file_name.c_str());
  bool written = cv::imwrite(target_file_name, image);
  if (!written) {
    LOG_ERROR("Unable to write image.");
  }
  return written;
}

}  // namespace cad_image_markup