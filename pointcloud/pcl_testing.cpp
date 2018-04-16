#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <boost/unordered_map.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// #define MAX_BUF 1024

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  std::cout << "INIT?" << std::endl;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int
  main (int argc, char** argv)
{

  struct pt {
    float x;
    float y;
    float z;
  };

  pt new_pt;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Hallbot Point Cloud"));
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud, 0, 255, 0);
  // viewer->setBackgroundColor (0, 0, 0);
  bool cloud_added = false;
  int fd;
  char* myfifo = "../../opencv/myfifo";
  boost::unordered::unordered_map<std::string, pt> pt_map;
  int i = 0;
  std::string s = "";
  while(1){
      /* open, read, and display the message from the FIFO */
    // std::cout << "LOOP BEGIN" << std::endl;

    fd = open(myfifo, O_RDONLY);

    
    //std::cout << "FILE OPEN" << std::endl;

    read(fd, &new_pt, sizeof(pt));

    // std::cout << "PIPE READ" << std::endl;

      /* if new pt, at to hash map */
    s = "";
    s.append(boost::lexical_cast<std::string>(new_pt.x));
    s.append(" ");
    s.append(boost::lexical_cast<std::string>(new_pt.y));
    s.append(" ");
    s.append(boost::lexical_cast<std::string>(new_pt.z));

    if(!pt_map.count(s)) {
      pt_map[s] = new_pt;
      i++;
      // std::cout << "POINT #" << i << " " << new_pt.x << " " << new_pt.y << " " << new_pt.z << std::endl;
      basic_cloud_ptr->push_back (pcl::PointXYZ (new_pt.x, new_pt.y, new_pt.z)); 
      //std::cout << cloud->size() << std::endl;
      //blocks until the cloud is actually rendered
 
      if(cloud_added){
        if(!viewer->updatePointCloud(basic_cloud_ptr, "sample cloud")){
          std::cout << "Cloud UPDATE FAIL" << std::endl;
        }
        
        std::cout << "Cloud Updated" << std::endl;
      } else {
        viewer = customColourVis(basic_cloud_ptr);
        // viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "Hallbot cloud");
        // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Hallbot cloud");
        // viewer->addCoordinateSystem (1.0);
        // viewer->initCameraParameters ();
        std::cout << "Cloud Added" << std::endl;
        cloud_added = true;
      }
    }
    close(fd);

    i++;

    //std::cout << i << std::endl;

  }


  // printf("Received: %s\n", buf);

  

  return (0);
}
