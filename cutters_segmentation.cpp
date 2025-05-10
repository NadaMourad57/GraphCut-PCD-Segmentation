#include <iostream>
#include <vector>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/min_cut_segmentation.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/cortex/GraphCut-PCD-Segmentation/med_scaled.pcd", *cloud) == -1)
    {
        std::cerr << "Cloud reading failed." << std::endl;
        return -1;
    }

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);
    std::cout << "Loaded " << cloud->size() << " points, filtered to " << indices->size() << " points.\n";

    float radius = 0.03;
    float vertical_limit = 0.027;

    // Set up segmentation object once
    pcl::MinCutSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud(cloud);
    seg.setIndices(indices);
    seg.setNumberOfNeighbours(30);
    seg.setSigma(0.005);
    seg.setRadius(radius);
    seg.setSourceWeight(2.0);
    // Assign different colors per seed (wraps around if > 6 seeds)
std::vector<std::tuple<float, float, float>> colors = {
    {1.0, 0.0, 0.0},  // Red
    {0.0, 1.0, 0.0},  // Green
    {0.0, 0.0, 1.0},  // Blue
    {1.0, 1.0, 0.0},  // Yellow
    {1.0, 0.0, 1.0},  // Magenta
    {0.0, 1.0, 1.0}   // Cyan
};

float r, g, b;



    // Define seed points
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> seed_points = {
        {0.04197685726428131, -0.10707418202773965, 0.4863020500324206},
        {0.05936154565764844, -0.15338471096914613, 0.4684879290055432},
        {0.07476264655662287, -0.17911226413076417, 0.40912986617474834},
        {0.07557618289699268, -0.17764848487121437, 0.35450979571467334},
        {0.055017018960903386, -0.18004765448747073, 0.31198373704508897}
    };

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("All Seeds Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
        //  create a viwer foreach seed 
        


    int counter = 0;
    for (const auto& point : seed_points)
    {
        std::stringstream cloud_id, sphere_id;
        cloud_id << "filtered_cloud_" << counter;
        sphere_id << "seed_sphere_" << counter;

        // Set seed point for this iteration
        pcl::PointCloud<pcl::PointXYZ>::Ptr seed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        seed_cloud->points.push_back(point);
        seg.setForegroundPoints(seed_cloud);

        std::vector <pcl::PointIndices> clusters;
        seg.extract (clusters);

        pcl::PointCloud<pcl::PointXYZ>::Ptr fg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud (*cloud, clusters[1].indices, *fg_cloud);
        std::cout << "Foreground points: " << fg_cloud->size () << std::endl;

        // Vertical filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_fg(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pt : fg_cloud->points) {
            if (std::abs(pt.z - point.z) <= vertical_limit)
                filtered_fg->points.push_back(pt);
        }
        filtered_fg->width = filtered_fg->points.size();
        filtered_fg->height = 1;
        filtered_fg->is_dense = true;


        // Add filtered cloud to viewer
        viewer->addPointCloud<pcl::PointXYZ>(filtered_fg, cloud_id.str());
        std::tie(r, g, b) = colors[counter % colors.size()];
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                 r, g, b,
                                                 cloud_id.str());

        // Add seed sphere
        viewer->addSphere(point, 0.002, 1.0, 1.0, 0.0, sphere_id.str());  // Yellow

        counter++;
    }
    

    // Display
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}
