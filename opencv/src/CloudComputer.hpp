#pragma once

#include <opencv2/imgproc.hpp>
#include "IConfigurable.hpp"
#include <chrono>
#include <fstream>

//OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//ORB-SLAM2
#include <System.h>
#include <Map.h>
#include <MapPoint.h>
#include <KeyFrame.h>
#include <pangolin/pangolin.h>

//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
//BROKEN
//#include <pcl/visualization/pcl_visualizer.h>

class CloudComputer : public IConfigurable {

  struct pt {
    float x;
    float y;
    float z;
  };

public:
  CloudComputer(std::vector<IConfigurable::Param>&& params);

  void addPoint(cv::Mat& pt);

  void display2D(ORB_SLAM2::Map* total_map);

  void getPointRanges(ORB_SLAM2::Map* total_map);

  void displayCloud(ORB_SLAM2::Map* total_map);

private:
  pcl::PointCloud<pcl::PointXYZ> cloud;

  char * myfifo;

  int w;

  cv::Mat hallway_image;

  void addCircle( cv::Mat img, cv::Point center , int color);
};
