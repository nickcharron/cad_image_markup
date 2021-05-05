#include <stdio.h>

#include <cad_image_markup/Visualizer.h>

namespace cad_image_markup {

Visualizer::Visualizer(const std::string name) {
  display_name_ = name;
  display1_called_ = false;
  display2_called_= false;
  display3_called_ = false;
  display4_called_ = false;
  display5_called_ = false;
  display6_called_ = false;
}

Visualizer::~Visualizer() {}

void Visualizer::StartVis(uint16_t coord_size) {
  point_cloud_display_ =
      boost::make_shared<pcl::visualization::PCLVisualizer>(display_name);
  point_cloud_display_->setBackgroundColor(0, 0, 0);
  point_cloud_display_->addCoordinateSystem(coord_size);
  point_cloud_display_->initCameraParameters();

  continue_flag_.test_and_set(std::memory_order_relaxed);

  // start the visualizer spinning in its own thread
  vis_thread_ = std::thread(&Visualizer::spin, this);
}

void Visualizer::EndVis() {
  continue_flag_.clear(std::memory_order_relaxed);
  vis_thread_.join();
}

void Visualizer::DisplayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               std::string id) {
  // get mutex for visulalizer spinning in vis thread
  // and either create a new cloud or update the existing one
  mtx_.lock();

  // if the visualizer does not already contain the point cloud, add it
  if (!display1_called_) {
    point_cloud_display_->addPointCloud(cloud, id, 0);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id);
    point_cloud_display_->resetCamera();
  }

  // otherwise, update the existing cloud
  else
    point_cloud_display_->updatePointCloud(cloud, id);

  mtx_.unlock();

  display1_called_ = true;
}

void Visualizer::DisplayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
                               std::string id1, std::string id2) {
  // get mutex for visulalizer spinning in vis thread
  // and either create a new cloud or update the existing one
  mtx_.lock();

  // if the visualizer does not already contain the image cloud, add it
  if (!display2_called_) {
    point_cloud_display_->addPointCloud(cloud1, id1);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id1);
    point_cloud_display_->resetCamera();
    point_cloud_display_->addPointCloud(cloud2, id2);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id2);
    point_cloud_display_->resetCamera();
  }

  // otherwise, update the existing cloud
  else {
    point_cloud_display_->updatePointCloud(cloud1, id1);
    point_cloud_display_->updatePointCloud(cloud2, id2);
  }

  mtx_.unlock();

  display2_called_= true;
}

// display three clouds with no correspondences
void Visualizer::DisplayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3,
                               std::string id1, std::string id2,
                               std::string id3) {
  // get mutex for visulalizer spinning in vis thread
  // and either create a new cloud or update the existing one
  mtx_.lock();

  // if the visualizer does not already contain the image cloud, add it
  if (!display3_called_) {
    point_cloud_display_->addPointCloud(cloud1, id1);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id1);
    point_cloud_display_->resetCamera();
    point_cloud_display_->addPointCloud(cloud2, id2);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id2);
    point_cloud_display_->resetCamera();
    point_cloud_display_->addPointCloud(cloud3, id3);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id3);
    point_cloud_display_->resetCamera();
  }
  // otherwise, update the existing cloud
  else {
    point_cloud_display_->updatePointCloud(cloud1, id1);
    point_cloud_display_->updatePointCloud(cloud2, id2);
    point_cloud_display_->updatePointCloud(cloud3, id3);
  }

  mtx_.unlock();

  display3_called_ = true;
}

// display three clouds with no correspondences
void Visualizer::DisplayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4,
                               std::string id1, std::string id2,
                               std::string id3, std::string id4) {
  // get mutex for visulalizer spinning in vis thread and
  // either create a new cloud or update the existing one
  mtx_.lock();

  // if the visualizer does not already contain the image cloud, add it
  if (!display3_called_) {
    point_cloud_display_->addPointCloud(cloud1, id1);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id1);
    point_cloud_display_->resetCamera();
    point_cloud_display_->addPointCloud(cloud2, id2);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id2);
    point_cloud_display_->resetCamera();
    point_cloud_display_->addPointCloud(cloud3, id3);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id3);
    point_cloud_display_->resetCamera();
    point_cloud_display_->addPointCloud(cloud4, id4);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id4);
    point_cloud_display_->resetCamera();
  }

  // otherwise, update the existing cloud
  else {
    point_cloud_display_->updatePointCloud(cloud1, id1);
    point_cloud_display_->updatePointCloud(cloud2, id2);
    point_cloud_display_->updatePointCloud(cloud3, id3);
    point_cloud_display_->updatePointCloud(cloud3, id4);
  }

  mtx_.unlock();

  display4_called_ = true;
}

void Visualizer::DisplayClouds(
    pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud,
    pcl::CorrespondencesConstPtr corrs, std::string id_image,
    std::string id_projected) {
  // get mutex for visulalizer spinning in vis thread and
  // either create a new cloud or update the existing one
  mtx_.lock();

  // if the visualizer does not already contain the image cloud, add it
  if (!display3_called_) {
    point_cloud_display_->addPointCloud(image_cloud, id_image);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image);
    point_cloud_display_->addPointCloud(projected_cloud, id_projected);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_projected);
  }
  // otherwise, update the existing cloud
  else {
    point_cloud_display_->updatePointCloud(image_cloud, id_image);
    point_cloud_display_->updatePointCloud(projected_cloud, id_projected);
  }

  // remove all correspondence lines and redraw
  point_cloud_display_->removeAllShapes();

  uint16_t line_start_index = 0, line_end_index = 1;
  uint16_t line_id = 0;

  // illustrate correspondences
  for (uint16_t i = 0; i < corrs->size(); i++) {
    uint16_t proj_point_index = corrs->at(i).index_match;
    uint16_t cam_point_index = corrs->at(i).index_query;

    point_cloud_display_->addLine(projected_cloud->at(proj_point_index),
                                 image_cloud->at(cam_point_index), 0, 255, 0,
                                 std::to_string(line_id));
    line_start_index += 2;
    line_end_index += 2;
    line_id++;
  }

  mtx_.unlock();

  display5_called_ = true;
}

void Visualizer::DisplayClouds(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud,
    pcl::CorrespondencesConstPtr corrs, std::string id_image,
    std::string id_cad, std::string id_projected) {
  // get mutex for visulalizer spinning in vis thread and
  // either create a new cloud or update the existing one
  mtx_.lock();

  // if the visualizer does not already contain the image cloud, add it
  if (!display6_called_) {
    point_cloud_display_->addPointCloud(image_cloud, id_image);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image);
    point_cloud_display_->addPointCloud(cad_cloud, id_cad);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_cad);
    point_cloud_display_->addPointCloud(projected_cloud, id_projected);
    point_cloud_display_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id_projected);
    point_cloud_display_->resetCamera();
  }

  // otherwise, update the existing cloud
  else {
    point_cloud_display_->updatePointCloud(image_cloud, id_image);
    point_cloud_display_->updatePointCloud(cad_cloud, id_cad);
    point_cloud_display_->updatePointCloud(projected_cloud, id_projected);
    point_cloud_display_->resetCamera();
  }

  // remove all correspondence lines and redraw
  point_cloud_display_->removeAllShapes();

  uint16_t line_start_index = 0, line_end_index = 1;
  uint16_t line_id = 0;

  // illustrate correspondences
  for (uint16_t i = 0; i < corrs->size(); i++) {
    uint16_t proj_point_index = corrs->at(i).index_match;
    uint16_t cam_point_index = corrs->at(i).index_query;

    point_cloud_display_->addLine(projected_cloud->at(proj_point_index),
                                 image_cloud->at(cam_point_index), 0, 255, 0,
                                 std::to_string(line_id));
    line_start_index += 2;
    line_end_index += 2;
    line_id++;
  }

  mtx_.unlock();

  display6_called_ = true;
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