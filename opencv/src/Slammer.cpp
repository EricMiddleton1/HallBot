#include "Slammer.hpp"

#include <cmath>
#include <iostream>

#include <System.h>
#include <Map.h>
#include <MapPoint.h>
#include <KeyFrame.h>
#include <pangolin/pangolin.h>

Slammer::Slammer(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"vocab_file", "config_file"}, std::move(params) }
  , slam{getParam("vocab_file"), getParam("config_file"), ORB_SLAM2::System::MONOCULAR,
      (getParam("visualizer", "false") == "true") ? true : false}
  , startTime{std::chrono::steady_clock::now()} {
}

cv::Mat Slammer::process(const cv::Mat& input) {
  auto time = std::chrono::steady_clock::now() - startTime;
  double t = std::chrono::duration_cast<std::chrono::microseconds>(time).count()/1000000.;
  
  return slam.TrackMonocular(input, t);
}

cv::Mat Slammer::getLastMapPoint(){
  // std::cout << "function called..." << std::endl;
  auto pts = slam.GetTrackedMapPoints();
  if (pts.empty()){
    //std::cout << "empty found..." << std::endl;
    return {};
  } else {
    if (!pts.back()) {
      //std::cout << "NULL point found..." << std::endl;
      return {};
    }
    //std::cout << "POINT found..." << pts.back()->mnId << std::endl;
    auto pos = pts.back()->GetWorldPos();
    //std::cout << "POSITION got..." << std::endl;
    if (pos.empty()){
      //std::cout << "POINT EMPTY..." << std::endl;
      return {};
    } else {
      //std::cout << "POINT GOOOD..." << std::endl;
      return pos;
    }
  }
}

ORB_SLAM2::Map* Slammer::getMap(){
  return slam.getMap();
}

void Slammer::saveAllMapPoints(){

  ofstream pcl_file;
  pcl_file.open ("pcl_file.pcd");

  ORB_SLAM2::Map* total_map = slam.getMap();
  const vector<ORB_SLAM2::MapPoint*> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint*> &ref_pts = total_map->GetReferenceMapPoints();

  set<ORB_SLAM2::MapPoint*> set_ref_pts(ref_pts.begin(), ref_pts.end());

  if(map_pts.empty()){
    std::cout << "ERROR: No Map Pts to Write" << std::endl;
    return;
  }

  for(size_t i=0, iend=map_pts.size(); i<iend;i++)
  {
      if(map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
          continue;
      cv::Mat pos = map_pts[i]->GetWorldPos();
      pcl_file << pos.at<float>(0) << " "
             << pos.at<float>(1) << " "
             << pos.at<float>(2) << "\n";
  }

  for(set<ORB_SLAM2::MapPoint*>::iterator sit=set_ref_pts.begin(), send=set_ref_pts.end(); sit!=send; sit++)
  {
      if((*sit)->isBad())
          continue;
      cv::Mat pos = (*sit)->GetWorldPos();
      pcl_file << pos.at<float>(0) << " "
             << pos.at<float>(1) << " "
             << pos.at<float>(2) << "\n";

  }


  std::cout << "File Written" << std::endl;

  /*
  auto pts = slam.GetTrackedMapPoints();
  // empty list check
  if(!pts.empty()) {
    for(int i = 0; i < pts.size(); i++){
      // null MapPoint check
      if(pts.at(i)){
        auto pos = pts.at(i)->GetWorldPos();
        // empty WorldPos check
        if(!pos.empty()){
          pcl_file << pos.at<float>(0) << " "
                 << pos.at<float>(1) << " "
                 << pos.at<float>(2) << "\n";
        }
      }
    }
    std::cout << "File Written" << std::endl;
  } else {
    std::cout << "ERROR: SAVE FAILED" << std::endl;
  }

  */

  pcl_file.close();
}


int Slammer::getStateofTrack() {
  return slam.GetTrackingState();
}

cv::Mat Slammer::draw() {
  return slam.DrawFrame();
}

int Slammer::getTrackingState() {
  return slam.GetTrackingState();
}
