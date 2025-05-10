#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/min_cut_segmentation.h>

int main ()
{
  pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("/home/cortex/GraphCut-PCD-Segmentation/med_scaled.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*cloud, *indices);
  std::cout << "Loaded " << cloud->size () << " data points from med_scaled.pcd" << std::endl;
  std::cout << "Filtered " << indices->size () << " data points from med_scaled.pcd" << std::endl;
  float radius = 0.022; // radius for filtering points around the seed point
  float vertical_limit = 0.03; // vertical limit for filtering points around the seed point

  pcl::MinCutSegmentation<pcl::PointXYZ> seg;
  seg.setInputCloud (cloud);
  seg.setIndices (indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointXYZ point;

    
  // v 0.04197685726428131 -0.10707418202773965 0.4863020500324206
  // v 0.05936154565764844 -0.15338471096914613 0.4684879290055432
  // v 0.07476264655662287 -0.17911226413076417 0.40912986617474834
  // v 0.07557618289699268 -0.17764848487121437 0.35450979571467334
  // v 0.055017018960903386 -0.18004765448747073 0.31198373704508897    

  // //point1
  // point.x = 0.04197685726428131;
  // point.y = -0.10707418202773965;
  // point.z =0.4863020500324206;


  //point2
  point.x = 0.05936154565764844 ;
  point.y = -0.15338471096914613;
  point.z =0.4684879290055432;

// point3
  // point.x = 0.07476264655662287 ;
  // point.y = -0.17911226413076417;
  // point.z =0.40912986617474834;

  //point4
  // point.x =0.07557618289699268;
  // point.y = -0.17764848487121437;
  // point.z = 0.35450979571467334;
    
  

    //point5
  // point.x = 0.055017018960903386;
  // point.y = -0.18004765448747073;
  // point.z = 0.31198373704508897;







  foreground_points->points.push_back(point);
  seg.setForegroundPoints (foreground_points);


  seg.setNumberOfNeighbours (30); 
  seg.setSigma(0.005);         // (stronger edge cut between distant points) /Make edge cuts happen easily/stronger between points
  seg.setRadius(radius);         // (penalty starts earlier = background grows) Penalize far points, pushing them to background.
  seg.setSourceWeight(2.0);     // seed attraction weight
  
// pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
// pcl::PointXYZ point;
//  point.x = 68.97;
//  point.y = -18.55;
//  point.z = 0.57;
//  foreground_points->points.push_back(point);
//  seg.setForegroundPoints (foreground_points);

//  seg.setSigma (0.25);
// seg.setRadius (3.0433856);
//  seg.setNumberOfNeighbours (14);
//  seg.setSourceWeight (0.8);
  std::vector <pcl::PointIndices> clusters;
  seg.extract (clusters);
  
  
  std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;
std::cout << "Number of clusters is " << clusters.size () << std::endl;


pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();





// get foreground points
  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud (*cloud, clusters[1].indices, *foreground_cloud);
  std::cout << "Foreground points: " << foreground_cloud->size () << std::endl;
  // get background points
  pcl::PointCloud<pcl::PointXYZ>::Ptr background_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud (*cloud, clusters[0].indices, *background_cloud);
  std::cout << "Background points: " << background_cloud->size () << std::endl;


  pcl::PointCloud<pcl::PointXYZ>::Ptr vertically_filtered_fg(new pcl::PointCloud<pcl::PointXYZ>);

  for (const auto& pt : foreground_cloud->points)
  {
      float dz = std::abs(pt.z - point.z);
      if (dz <= vertical_limit)
      {
          vertically_filtered_fg->points.push_back(pt);
      }
  }
  
  vertically_filtered_fg->width = vertically_filtered_fg->points.size();
  vertically_filtered_fg->height = 1;
  vertically_filtered_fg->is_dense = true;
  

  //visualize the clusters
  pcl::visualization::PCLVisualizer viewer1("3D Viewer");
  viewer1.setBackgroundColor (0.0, 0.0, 0.0);
  viewer1.addPointCloud<pcl::PointXYZ> (foreground_cloud, "foreground");
  viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "foreground");
  viewer1.addSphere(
    pcl::PointXYZ(point.x, point.y,point.z), // coordinates of seed point
    0.002,                                // sphere radius (adjust if needed)
    1.0, 1.0, 0.0,                      // RGB color = yellow (R=1, G=1, B=0)
    "seed_sphere"                       // unique ID
);

  pcl::visualization::PCLVisualizer viewer2("filtered 3D Viewer");
  viewer2.setBackgroundColor (0.0, 0.0, 0.0);
  viewer2.addPointCloud<pcl::PointXYZ> (vertically_filtered_fg, "filtered_foreground");
  viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "filtered_foreground");


  viewer2.addSphere(
    pcl::PointXYZ(point.x, point.y,point.z), // coordinates of seed point
    0.002,                                // sphere radius (adjust if needed)
    1.0, 1.0, 0.0,                      // RGB color = yellow (R=1, G=1, B=0)
    "seed_sphere"                       // unique ID
);

  // // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cluster viewer"));
  viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "segmented_cloud");
  
  // Create a single yellow seed point (as a small sphere)
  viewer->addSphere(
      pcl::PointXYZ(point.x, point.y,point.z), // coordinates of seed point
      0.002,                                // sphere radius (adjust if needed)
      1.0, 1.0, 0.0,                      // RGB color = yellow (R=1, G=1, B=0)
      "seed_sphere"                       // unique ID
  );
  
  // Display
  while (!viewer1.wasStopped ()  && !viewer->wasStopped ()&& !viewer2.wasStopped ())
  {
      viewer1.spinOnce(100);
      viewer2.spinOnce(100);
      viewer->spinOnce(100);
  }
  

}
