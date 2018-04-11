#include "CloudComputer.hpp"
#include <cmath>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
//BROKEN
//#include <pcl/visualization/pcl_visualizer.h>


CloudComputer::CloudComputer(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {}, std::move(params) } {
}

void CloudComputer::displayCloud(){
  // pcl::visualization::PCLVisualizer viewer ("POINT CLOUD TEST");
  // pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // pcl::io::loadPCDFile ("test_pcd.pcd", *source_cloud);
  // // Define R,G,B colors for the point cloud
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // // We add the point cloud to the viewer and pass the color handler
  // viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
  // //pcl::io::loadPCDFile ("test_pcd.pcd", *newcloud);
  // //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  // viewer.addCoordinateSystem (1.0, "cloud", 0);
  // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  // //  viewer.showCloud (newcloud);
  // while (!viewer.wasStopped ())
  //   {
  //     viewer.spinOnce ();
  //
  //   }
}
