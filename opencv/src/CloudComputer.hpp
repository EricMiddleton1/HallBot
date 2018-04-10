#pragma once

#include <opencv2/imgproc.hpp>
#include "IConfigurable.hpp"
#include <chrono>

//ORB-SLAM2
#include <System.h>

//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class CloudComputer : public IConfigurable {
public:
  CloudComputer();

  void addPoint(cv::Mat& pt);

  void displayCloud();

private:
  pcl::PointCloud<pcl::PointXYZ> cloud;
};
