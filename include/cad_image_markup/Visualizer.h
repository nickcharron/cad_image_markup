#pragma once
#include <string>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>

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
     * @brief Method to display an image cloud, CAD cloud, projected cloud and correspondences 
     * @param image_cloud labelled image point cloud
     * @param cad_cloud CAD cloud 
     * @param projected_cloud projected CAD point cloud
     * @param corrs correspondences between projected point cloud and labelled image cloud
     * @param id_image unique cloud id for display
     * @param id_cad unique cloud id for display
     * @param id_projected unique cloud id for display
     * @param source source cloud for the correspondences, "projected" or "camera"
     */
    void DisplayClouds(PointCloud::ConstPtr image_cloud,
                            PointCloud::Ptr cad_cloud,
                            PointCloud::Ptr projected_cloud,
                            pcl::CorrespondencesConstPtr corrs,
                            std::string id_image,
                            std::string id_cad,
                            std::string id_projected, 
                            std::string source);

private: 
    pcl::visualization::PCLVisualizer::Ptr point_cloud_display_;
    std::thread vis_thread_;

    // mutex for the point_cloud_display object, held by the main thread when updating the visualization params
    std::mutex mtx_;

    bool display_called_;

    std::string display_name_;

    std::atomic_flag continue_flag_ = ATOMIC_FLAG_INIT;

    // vis thread method in which the visualizer spins
    void Spin();

};


} // namespace cam_cad
