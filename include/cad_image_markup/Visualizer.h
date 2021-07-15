#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <math.h>
#include <mutex>
#include <string>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>

#include <cad_image_markup/nlohmann/json.h>
#include <cad_image_markup/Utils.h>

namespace cad_image_markup { 

/**
 * @brief Interactive visualizer class to display point clouds and correspondences
 * Note: to use: 
 * 1. create visualizer instance
 * 2. call startVis()
 * 3. call desired displayClouds() version
 * 4. call endVis()
 */
class Visualizer{
public: 

    /**
     * @brief Constructor 
     * @param name display name
     */
    Visualizer(const std::string name); 

    /**
     * @brief Empty destructor 
     */
    ~Visualizer(); 

    /**
     * @brief Starts visualizer in a new thread 
     * @param coord_size size of the coordinate axes to display 
     */
    void StartVis(uint16_t coord_size = 100); 

    /**
     * @brief Ends visualizer thread
     * @todo get display window to close when called, currently hangs until end of program 
     */
    void EndVis();

    /**
     * @brief Method to display one point cloud 
     * @param cloud point cloud to display
     * @param id unique cloud id for display
     */
    void DisplayClouds(PointCloud::Ptr cloud, std::string id);

    /**
     * @brief Method to display two point clouds
     * @param cloud1 point cloud to display
     * @param cloud2 point cloud to display
     * @param id1 unique cloud id for display
     * @param id2 unique cloud id for display
     */
    void DisplayClouds(PointCloud::Ptr cloud1,
                            PointCloud::Ptr cloud2,
                            std::string id1,
                            std::string id2);

    /**
     * @brief Method to display three point clouds
     * @param cloud1 point cloud to display
     * @param cloud2 point cloud to display
     * @param cloud3 point cloud to display
     * @param id1 unique cloud id for display
     * @param id2 unique cloud id for display
     * @param id3 unique cloud id for display
     */
    void DisplayClouds(PointCloud::Ptr cloud1, 
                       PointCloud::Ptr cloud2,
                       PointCloud::Ptr cloud3,
                       std::string id1,
                       std::string id2,
                       std::string id3);

    /**
     * @brief Method to display four point clouds
     * @param cloud1 point cloud to display
     * @param cloud2 point cloud to display
     * @param cloud3 point cloud to display
     * @param cloud4 point cloud to display
     * @param id1 unique cloud id for display
     * @param id2 unique cloud id for display
     * @param id3 unique cloud id for display
     * @param id4 unique cloud id for display
     */
    void DisplayClouds(PointCloud::Ptr cloud1, 
                       PointCloud::Ptr cloud2,
                       PointCloud::Ptr cloud3,
                       PointCloud::Ptr cloud4,
                       std::string id1,
                       std::string id2,
                       std::string id3,
                       std::string id4);

    /**
     * @brief Method to display an image cloud, projected cloud and correspondences 
     * @param image_cloud labelled image point cloud
     * @param projected_cloud projected CAD point cloud
     * @param corrs correspondences between projected point cloud and labelled image cloud
     * @param id_image unique cloud id for display
     * @param id_projected unique cloud id for display
     */
    void DisplayClouds(PointCloud::Ptr image_cloud,
                            PointCloud::Ptr projected_cloud,
                            pcl::CorrespondencesConstPtr corrs,
                            std::string id_image,
                            std::string id_projected);

    /**
     * @brief Method to display an image cloud, CAD cloud, projected cloud and correspondences 
     * @param image_cloud labelled image point cloud
     * @param cad_cloud CAD cloud 
     * @param projected_cloud projected CAD point cloud
     * @param corrs correspondences between projected point cloud and labelled image cloud
     * @param id_image unique cloud id for display
     * @param id_cad unique cloud id for display
     * @param id_projected unique cloud id for display
     */
    void DisplayClouds(PointCloud::ConstPtr image_cloud,
                            PointCloud::Ptr cad_cloud,
                            PointCloud::Ptr projected_cloud,
                            pcl::CorrespondencesConstPtr corrs,
                            std::string id_image,
                            std::string id_cad,
                            std::string id_projected);

private: 
    pcl::visualization::PCLVisualizer::Ptr point_cloud_display_;
    std::thread vis_thread_;

    //mutex for the point_cloud_display object, held by the main thread when updating the visualization params
    std::mutex mtx_;

    std::string display_name_;

    std::atomic_flag continue_flag_ = ATOMIC_FLAG_INIT;
    bool display1_called_;
    bool display2_called_;
    bool display3_called_;
    bool display4_called_;
    bool display5_called_;
    bool display6_called_;

    // vis thread method in which the visualizer spins
    void Spin();

};


} // namespace cam_cad
