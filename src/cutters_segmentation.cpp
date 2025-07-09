#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/min_cut_segmentation.h>

// Function to read seed points from OBJ file
std::vector<pcl::PointXYZ> readSeedPointsFromOBJ(const std::string& filename) {
    std::vector<pcl::PointXYZ> seedPoints;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return seedPoints;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        
        if (token == "v") {
            float x, y, z;
            if (iss >> x >> y >> z) {
                seedPoints.push_back(pcl::PointXYZ(x, y, z));
            }
        }
    }
    
    file.close();
    std::cout << "Read " << seedPoints.size() << " seed points from " << filename << std::endl;
    return seedPoints;
}

int main()
{


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std:: string drillbit = "/Users/nadamourad/Desktop/GraphCut-PCD-Segmentation/Dataset/06_02_2024_08_18/";
    std:: string pcd_file_path = drillbit + "med_scaled.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
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

    // Random seed for color generation
    srand(time(NULL));
    
    // Helper function to generate random colors
    auto generateRandomColor = []() {
        return std::make_tuple(
            static_cast<float>(rand()) / static_cast<float>(RAND_MAX),
            static_cast<float>(rand()) / static_cast<float>(RAND_MAX),
            static_cast<float>(rand()) / static_cast<float>(RAND_MAX)
        );
    };

    float r, g, b;



    // Read seed points from OBJ file
    std::string objFilePath = drillbit + "/ordered_blade_0.obj";
    std::vector<pcl::PointXYZ> seed_points = readSeedPointsFromOBJ(objFilePath);



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

        //save filtered cloud
        std::string filtered_cloud_filename = "filtered_cloud_" + std::to_string(counter) + ".pcd";
        pcl::io::savePCDFileASCII(filtered_cloud_filename, *filtered_fg);
        std::cout << "Saved filtered cloud to " << filtered_cloud_filename << std::endl;


        // Add filtered cloud to viewer
        viewer->addPointCloud<pcl::PointXYZ>(filtered_fg, cloud_id.str());
        std::tie(r, g, b) = generateRandomColor();
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
