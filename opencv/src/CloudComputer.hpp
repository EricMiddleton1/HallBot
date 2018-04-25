#pragma once

#include <opencv2/imgproc.hpp>
#include "IConfigurable.hpp"
#include <chrono>
#include <fstream>
#include <math.h>

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

#define PI 3.14159265

class CloudComputer : public IConfigurable
{

  struct pt
  {
    float x;
    float y;
    float z;
  };

public:
  CloudComputer(std::vector<IConfigurable::Param> &&params);

  void addPoint(cv::Mat &pt);

  void display2D(ORB_SLAM2::Map *total_map);

  void getPtVector(ORB_SLAM2::Map *total_map);

  cv::Mat rotateWithTheta(cv::Mat pos);

  cv::Mat autoRotate(cv::Mat pos);

  void curHallwayHistogram();

  float distToFacingWall();

  void getPointRanges(ORB_SLAM2::Map *total_map);

  cv::Vec4f getGreenLine();

  float getGreenTheta();

  void setCameraPos(cv::Vec2f pos);
  cv::Vec2f getHallPosition();
  float getWidth();

private:
  char *myfifo;

  int w, regression_type, how_recent, auto_adjust_angle, left_wall, right_wall;

  int green_theta_count; // used in filter

  float theta, auto_theta, stored_green_theta;

  cv::Mat hallway_image, bw_hallway_image;

  //Array of Points used for regression
  vector<cv::Point3f> pts_vector_3d;
  vector<cv::Point> pts_vector;
  vector<cv::Point> raw_pts_vector; // REVIEW
  vector<cv::Mat> raw_mat_vector;

  // auto rotate 3D line
  cv::Vec6f compass_line;
  cv::Vec4f long_term_line;

  // camera position
  cv::Vec2f cam_pos;

  bool wall_alert, enough_pts_already, adjusted_3d;

  void displayPoints();

  void addCircle(cv::Mat img, cv::Point center, int color);

  void drawLine(cv::Mat img, cv::Vec4f line, int thickness, cv::Scalar color);

  cv::Vec4f convertLine2D(cv::Vec6f a3dline);

  int convertPoint2D(auto num, int axis);
  float undoConvertPoint2D(int num, int axis);
  cv::Mat euler2rot(const cv::Mat &euler);

  void updatePointVectors(cv::Mat pos);
  void clearPointVectors();
  void steerPoints();
  std::vector<int> getProjection(const cv::Mat &image);

  void makeGreenLine();
  void displayCamera();

  bool m_display;
};
