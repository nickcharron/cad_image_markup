#include <cad_image_markup/Visualizer.h>

#include <stdio.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <math.h>
#include <mutex>

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

namespace cad_image_markup {

Visualizer::Visualizer(const std::string name) {
  display_name_ = name;
}

Visualizer::~Visualizer() {}

void Visualizer::StartVis(uint16_t coord_size) {
  point_cloud_display_ =
      std::make_shared<pcl::visualization::PCLVisualizer>(display_name_);
  point_cloud_display_->setBackgroundColor(255, 255, 255); //white
  point_cloud_display_->addCoordinateSystem(coord_size);
  point_cloud_display_->initCameraParameters();

  // align the visualizer camera with the image source when the visualizer first displayed
  point_cloud_display_->setCameraPosition(0, 0, -3,    0, 0, 1,   0, -1, 0);

  continue_flag_.test_and_set(std::memory_order_relaxed);

  //start the visualizer spinning in its own thread
  vis_thread_ = std::thread(&Visualizer::Spin, this);
}

void Visualizer::EndVis() {
  continue_flag_.clear(std::memory_order_relaxed);
  vis_thread_.join();
}

void Visualizer::DisplayClouds(PointCloud::ConstPtr image_cloud,
                               PointCloud::Ptr cad_cloud,
                               PointCloud::Ptr projected_cloud,
                               pcl::CorrespondencesConstPtr corrs,
                               std::string id_image, std::string id_cad,
                               std::string id_projected, 
                               std::string source) {
  // get mutex for visulalizer spinning in vis thread and
  // either create a new cloud or update the existing one
  mtx_.lock();

  LOG_INFO("Visualizer: Displaying Clouds");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> image_color(image_cloud, 125, 0, 125);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cad_color(image_cloud, 0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> projected_color(image_cloud, 255, 0, 0);

  // if the visualizer does not already contain the image cloud, add it
  if (!display_called_) {
    point_cloud_display_->addPointCloud(image_cloud, image_color, id_image);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image);
    point_cloud_display_->addPointCloud(cad_cloud, cad_color, id_cad);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_cad);
    point_cloud_display_->addPointCloud(projected_cloud, projected_color, id_projected);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id_projected);
    point_cloud_display_->resetCamera();
  }

  // otherwise, update the existing cloud
  else {
    point_cloud_display_->updatePointCloud(image_cloud, image_color, id_image);
    point_cloud_display_->updatePointCloud(cad_cloud, cad_color, id_cad);
    point_cloud_display_->updatePointCloud(projected_cloud, projected_color, id_projected);
    point_cloud_display_->resetCamera();
  }

  // remove all correspondence lines and redraw
  point_cloud_display_->removeAllShapes();

  uint16_t line_start_index = 0, line_end_index = 1;
  uint16_t line_id = 0;

  // illustrate correspondences
  for (uint16_t i = 0; i < corrs->size(); i++) {

    uint16_t proj_point_index, cam_point_index;

    // source camera
    if (source == "camera") {
      proj_point_index = corrs->at(i).index_match;
      cam_point_index = corrs->at(i).index_query;
    }


    // source CAD
    if (source == "projected") {
      proj_point_index = corrs->at(i).index_query;
      cam_point_index = corrs->at(i).index_match;
    }

    point_cloud_display_->addLine(projected_cloud->at(proj_point_index),
                                  image_cloud->at(cam_point_index), 0, 255, 0,
                                  std::to_string(line_id));
    line_start_index += 2;
    line_end_index += 2;
    line_id++;
  }

  display_called_ = true;

  mtx_.unlock();

}

void Visualizer::Spin() {
  while (continue_flag_.test_and_set(std::memory_order_relaxed) &&
          !(point_cloud_display_->wasStopped())) {
    mtx_.lock();
    point_cloud_display_->spinOnce(3);
    mtx_.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace cad_image_markup