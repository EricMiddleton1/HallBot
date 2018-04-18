#pragma once

#include <opencv2/imgproc.hpp>

#include "IConfigurable.hpp"

#include <chrono>
#include <fstream>

//ORB-SLAM2
#include <System.h>
#include <Map.h>
#include <MapPoint.h>
#include <KeyFrame.h>
#include <pangolin/pangolin.h>

class Slammer : public IConfigurable {
public:
  Slammer(std::vector<IConfigurable::Param>&& params);

  cv::Mat process(const cv::Mat& input);

  cv::Mat getLastMapPoint();

  ORB_SLAM2::Map* getMap();

  void saveAllMapPoints();

  int getStateofTrack();

  cv::Mat draw();

  int getTrackingState();

private:
  ORB_SLAM2::System slam;
  std::chrono::steady_clock::time_point startTime;
};
