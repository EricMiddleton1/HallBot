#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <raspicam/raspicam_cv.h>

#include <string>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "GRANSAC.hpp"
#include "IntersectModel.hpp"

#include "iRobot.hpp"
#include "PID.hpp"

using namespace cv;
using namespace std;

using P_Line = pair<cv::Point, cv::Point>;

GRANSAC::RANSAC<IntersectModel,2> estimator;

const int min_threshold = 50;
const int max_trackbar = 150;
const int min_theta_trackbar = 0;
const int max_theta_trackbar = 180;
const int max_ransac_threshold = 500;
const int max_ransac_iterations = 1000;

const int TARGET_FRAME_HEIGHT = 480;

const string PORT = "/dev/ttyUSB0";

int s_trackbar = 40; //60
int v_theta_trackbar = 10;
int h_theta_trackbar = 5;
int ransac_threshold_trackbar = 10;
int ransac_iterations_trackbar = 100;

void printHelp(const string& str);
void updateRansac(int, void*);
cv::Point detectVanishingPoint(const Mat& edges, Mat& hough, cv::Point& avgPoint);
cv::Point intersectionPoint(const vector<P_Line>& inLines, cv::Point& avgPoint);

int main( int argc, char** argv )
{
  const string WINDOW_NAME = "Vanishing Point Detector";
  //const string DEBUG_WINDOW_NAME = "Vanishing Point Detector - Debug";
  
  if(argc < 2) {
    printHelp(argv[0]);
    return 1;
  }

  VideoCapture cap;
  VideoWriter output;
  raspicam::RaspiCam_Cv camera;
  bool pi = false;
  
  if(string(argv[1]) == "--webcam") {
    if(argc < 3) {
      printHelp(argv[0]);
      return 1;
    }
    
    cap.open(stoi(argv[2]));
    if(!cap.isOpened()) {
      std::cerr << "[Error] Failed to open webcam '" << argv[2] << "'" << std::endl;
      return 1;
    }
  }
  else if(string(argv[1]) == "--raspberrypi") {
    camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    camera.set(CV_CAP_PROP_EXPOSURE, 5);
    camera.set(CV_CAP_PROP_BRIGHTNESS, 50);
    camera.set(CV_CAP_PROP_GAIN, 100);
    camera.set(CV_CAP_PROP_CONTRAST, std::stoi(argv[2]));
    if(!camera.isOpened()) {
	    std::cerr << "[Error] Failed to open raspberry pi camera" << std::endl;
      return 1;
    }
    
    output.open(argv[3], CV_FOURCC('C','R','A','M'), 30, Size(640,480));
    if(!output.isOpened()) {
      std::cerr << "[Error] Failed to create video output" << std::endl;
    }

    pi = true;
  }
  else {
    cap.open(argv[1]);
    if(!cap.isOpened()) {
      std::cerr << "[Error] Failed to open video file '" << argv[1] << "'" << std::endl;
      return 1;
    }
  }

  float hallwayX = 0.5f;
  PID steerPID{200.f, 0.f, 100.f};
  steerPID.set(0.5f);
  
  boost::asio::io_service ioService;
  boost::asio::io_service::work ioWork(ioService);
  
  iRobot bot{ioService, PORT};
  bot.start();
  this_thread::sleep_for(chrono::milliseconds(100));
  std::thread botThread([&ioService]() {
    ioService.run();
    std::cerr << "[Error] ioService thread exit" << std::endl;
  });
  
  namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
  //namedWindow(DEBUG_WINDOW_NAME, WINDOW_AUTOSIZE);
/*
  createTrackbar("Hough Threshold", DEBUG_WINDOW_NAME, &s_trackbar, max_trackbar,
    nullptr);
  /*
  createTrackbar("Vertical Angle Threshold (degrees)", DEBUG_WINDOW_NAME,
    &v_theta_trackbar, max_theta_trackbar, nullptr);
  createTrackbar("Horizontal Angle Threshold (degrees)", DEBUG_WINDOW_NAME,
    &h_theta_trackbar, max_theta_trackbar, nullptr);
  */
  /*
  createTrackbar("RANSAC Threshold", DEBUG_WINDOW_NAME, &ransac_threshold_trackbar,
    max_ransac_threshold, updateRansac);
  createTrackbar("RANSAC Max Iterations", DEBUG_WINDOW_NAME, &ransac_iterations_trackbar,
    max_ransac_iterations, updateRansac);
*/
  updateRansac(0, 0);

  while(waitKey(1) == -1) {
    Mat frame, resized, frame_gray, frame_edges, frame_hough;
    
    if(pi) {
      camera.grab();
      camera.retrieve(frame_gray);
    }
    else {
      cap >> frame;
      if(frame.empty()) {
        //Restart video
        cap.set(CAP_PROP_POS_AVI_RATIO, 0);
        
        continue;
      }
      auto frameSize = frame.size();
      if(frameSize.height > TARGET_FRAME_HEIGHT) {
        float aspectRatio = static_cast<float>(frameSize.width) / frameSize.height;
        frameSize = Size(TARGET_FRAME_HEIGHT*aspectRatio, TARGET_FRAME_HEIGHT);
        resize(frame, resized, frameSize, 0, 0, INTER_CUBIC);
      }
      else {
        resized = frame;
      }
      cvtColor(resized, frame_gray, COLOR_RGB2GRAY);
    }
    /*
    auto frameSize = frame_gray.size();
    Canny(frame_gray, frame_edges, 50, 200, 3);
    
    try {
      cv::Point avgPoint;
      auto vanishingPoint = detectVanishingPoint(frame_edges, frame_hough, avgPoint);

      auto curX = static_cast<float>(vanishingPoint.x) / frameSize.width;
      hallwayX = 0.1f*curX + 0.9f*hallwayX;
      auto actuation = steerPID.update(hallwayX, 0.015);
      actuation = std::max(-100.f, std::min(100.f, actuation));

      circle(frame_gray, {hallwayX*frameSize.width, vanishingPoint.y}, 15,
        Scalar(0, 0, 255), -1);
      circle(frame_gray, vanishingPoint, 15, Scalar(255, 0, 0),
        -1);

      std::cout << "[Info] Hallway X = " << hallwayX << ", wheel actuation = "
        << actuation << std::endl;

      bot.setWheels(50 - actuation, 50 + actuation);
    }
    catch(const exception& e) {
      bot.setWheels(0, 0);
    }
    */
    //imshow(WINDOW_NAME, frame);
    imshow(WINDOW_NAME, frame_gray);
    //imshow(DEBUG_WINDOW_NAME, frame_hough);

    if(pi) {
      output.write(frame_gray);
    }
  }
  
  return 0;
}

void printHelp(const string& programName) {
  std::cout << "Usage:\n\t" << programName << " file_name\n\t"
    << programName << " --webcam webcam_id" << std::endl;
}

void updateRansac(int, void*) {
  estimator.Initialize(ransac_threshold_trackbar, ransac_iterations_trackbar);
}

cv::Point detectVanishingPoint(const Mat& edges, Mat& hough, cv::Point& avgPoint) {
  cv::Point vanishingPoint{0, 0};

  vector<Vec2f> s_lines;
  vector<P_Line> pointLines;
  
  cvtColor(edges, hough, COLOR_GRAY2BGR);
  HoughLines(edges, s_lines, 1, CV_PI/180, min_threshold + s_trackbar, 0, 0 );

  for(size_t i = 0; i < s_lines.size(); i++ ) {
    float r = s_lines[i][0], t = s_lines[i][1];
    float v_rad = v_theta_trackbar * CV_PI/180,
      h_rad = h_theta_trackbar * CV_PI/180;

    if( (t < v_rad) || (t > (CV_PI - v_rad)) || (std::abs(t - CV_PI/2) < h_rad) ) {
      continue;
    }

    double cos_t = cos(t), sin_t = sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    cv::Point pt1(cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t));
    cv::Point pt2(cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t));
        
    pointLines.emplace_back(pt1, pt2);

    line(hough, pt1, pt2, Scalar(255,0,0), 2, LINE_AA);
  }

  vanishingPoint = intersectionPoint(pointLines, avgPoint);
  circle(hough, avgPoint, 50, Scalar(0, 0, 255),
    -1);
  circle(hough, vanishingPoint, 15, Scalar(255, 0, 0),
    -1);

  return vanishingPoint;
}

cv::Point intersectionPoint(const vector<P_Line>& inLines, cv::Point& avgPoint) {
  struct LineParam {
    float a, b, c;
  };

  Vec2f point(0.f, 0.f);

  vector<shared_ptr<GRANSAC::AbstractParameter>> lines;
  lines.reserve(inLines.size());
  for(auto& line : inLines) {
    float a = line.second.y - line.first.y;
    float b = line.first.x - line.second.x;
    float c = b*line.first.y + a*line.first.x;

    lines.push_back(make_shared<Line>(a, b, c));
  }

  estimator.Estimate(lines);
  auto bestModel = estimator.GetBestModel();
  if(bestModel) {
    auto intersectionPoint = bestModel->getIntersectionPoint();
    auto avgIntersectionPoint = bestModel->getIntersectionPointAvg();

    point[0] = intersectionPoint.x;
    point[1] = intersectionPoint.y;

    avgPoint.x = cvRound(avgIntersectionPoint.x);
    avgPoint.y = cvRound(avgIntersectionPoint.y);
  }
  else {
    throw std::runtime_error("intersectionPoint: No intersections found");
  }

  return {cvRound(point[0]), cvRound(point[1])};
}
