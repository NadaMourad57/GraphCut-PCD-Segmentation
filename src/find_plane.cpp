
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv) {
    // Load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/Users/nadamourad/Desktop/GraphCut-PCD-Segmentation/ExtractedCutters/06_02_2024_08_18/filtered_cloud_3.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    // Create segmentation object
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.001);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    std::cout << "Plane coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    // Extract inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    // Visualize
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("RANSAC Plane"));
    pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("surface"));

    viewer->setBackgroundColor(0, 0, 0);

    // Add original cloud (rest)
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rest(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_rest);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_rest, "rest");
    viewer2->addPointCloud<pcl::PointXYZ>(cloud_rest, "rest");
    viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.6, 0.6, 0.6, "rest");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.6, 0.6, 0.6, "rest");

    // Add plane cloud
    viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, "plane");
    viewer2->addPointCloud<pcl::PointXYZ>(cloud_plane, "plane");

    viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0, "plane");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0, "plane");

    // Add the plane as a mesh (optional)
    viewer->addPlane(*coefficients, "fitted_plane");

    // Run viewer
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        viewer2->spinOnce(100);
    }

    return 0;
}
