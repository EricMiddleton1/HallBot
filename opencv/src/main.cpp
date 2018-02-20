#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

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

GRANSAC::RANSAC<IntersectModel> estimator;

const int min_threshold = 50;
const int max_trackbar = 150;
const int min_theta_trackbar = 0;
const int max_theta_trackbar = 180;
const int max_ransac_threshold = 500;
const int max_ransac_iterations = 1000;

const int TARGET_FRAME_HEIGHT = 480;

const string PORT = "/dev/ttyUSB0";

int s_trackbar = 28; //60
int v_theta_trackbar = 10;
int h_theta_trackbar = 5;
int ransac_threshold_trackbar = 10;
int ransac_iterations_trackbar = 100;

void printHelp(const string& str);
void updateRansac(int, void*);
cv::Point detectVanishingPoint(const Mat& edges, Mat& hough);
cv::Point intersectionPoint(const vector<P_Line>& inLines);

int main( int argc, char** argv )
{
  const string WINDOW_NAME = "Vanishing Point Detector";
  const string DEBUG_WINDOW_NAME = "Vanishing Point Detector - Debug";
  const string OPT_FLOW_WINDOW_NAME = "Optical Flow Detector";
  
  if(argc < 2) {
    printHelp(argv[0]);
    return 1;
  }

  VideoCapture cap;
  
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
  else {
    cap.open(argv[1]);
    if(!cap.isOpened()) {
      std::cerr << "[Error] Failed to open video file '" << argv[1] << "'" << std::endl;
      return 1;
    }
  }

  float hallwayX = 0.5f;
  PID steerPID{200.f, 0.f, 0.f};
  steerPID.set(0.5f);
  
  boost::asio::io_service ioService;
  boost::asio::io_service::work ioWork(ioService);
  
  iRobot bot{ioService, PORT};
  bot.start();
  this_thread::sleep_for(chrono::milliseconds(100));


  bot.setWheels(500, 0);
  
  std::thread botThread([&ioService]() {
    ioService.run();
    std::cerr << "[Error] ioService thread exit" << std::endl;
  });

  namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
  namedWindow(DEBUG_WINDOW_NAME, WINDOW_AUTOSIZE);
  namedWindow(OPT_FLOW_WINDOW_NAME, WINDOW_AUTOSIZE);
  
  createTrackbar("Hough Threshold", DEBUG_WINDOW_NAME, &s_trackbar, max_trackbar,
    nullptr);
  createTrackbar("Vertical Angle Threshold (degrees)", DEBUG_WINDOW_NAME,
    &v_theta_trackbar, max_theta_trackbar, nullptr);
  createTrackbar("Horizontal Angle Threshold (degrees)", DEBUG_WINDOW_NAME,
    &h_theta_trackbar, max_theta_trackbar, nullptr);
  createTrackbar("RANSAC Threshold", DEBUG_WINDOW_NAME, &ransac_threshold_trackbar,
    max_ransac_threshold, updateRansac);
  createTrackbar("RANSAC Max Iterations", DEBUG_WINDOW_NAME, &ransac_iterations_trackbar,
    max_ransac_iterations, updateRansac);

  updateRansac(0, 0);

  // previous angle from iRobot
  float prev_bot_theta = 0
  
  while(waitKey(10) == -1) {
    int startTime = cv::getTickCount();

    Mat frame, resized, img_flo, frame_gray, frame_edges, frame_hough;
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
    Canny(frame_gray, frame_edges, 50, 200, 3);
    
    try {
      auto vanishingPoint = detectVanishingPoint(frame_edges, frame_hough);

      auto curX = static_cast<float>(vanishingPoint.x) / frameSize.width;

      // Angle between direction of robot and vantage point based on camera.
      // Camera is 54 x 41 degrees.
      // Theta will be positive if VP is to the right or negative if VP is to the left. 
      float cam_theta = (curX - 0.5) * (26 * CV_PI / 180);

      // Angle from iRobot
      float bot_theta = bot.getAngle();

      // Complementary filter
      float alpha = 0.9f;
      float d_bot_theta = bot_theta - prev_bot_theta;
      float angle_n = (alpha * (prev_bot_theta + (d_bot_theta * 0.015))) + ((1 - alpha) * cam_theta);
      
      std::cout << "Camera angle: " << cam_theta << "\niRobot angle: " << bot_theta << "\n";
      	
      //hallwayX = 0.5f*curX + 0.5f*hallwayX;
      auto actuation = steerPID.update(angle_n, 0.015);

      circle(resized, {hallwayX*frameSize.width, vanishingPoint.y}, 15,
        Scalar(0, 0, 255), -1);
      circle(resized, vanishingPoint, 15, Scalar(255, 0, 0),
        -1);

      //std::cout << "[Info] Hallway X = " << hallwayX << ", wheel actuation = "
      //	<< actuation << std::endl;

      bot.setWheels(50 - actuation, 50 + actuation);
    }
    catch(const exception& e) {
      bot.setWheels(0, 0);
    }

		int endTime = cv::getTickCount();

		std::cout << "[Info] Processed frame in " << static_cast<float>(endTime - startTime)
			/ cv::getTickFrequency()*1000.f << "ms" << std::endl;

    imshow(WINDOW_NAME, resized);
    imshow(DEBUG_WINDOW_NAME, frame_hough);
    imshow(OPT_FLO_WINDOW_NAME, img_flo);
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

cv::Point detectVanishingPoint(const Mat& edges, Mat& hough) {
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

  vanishingPoint = intersectionPoint(pointLines);
  circle(hough, vanishingPoint, 15, Scalar(0, 0, 255),
    -1);

  return vanishingPoint;
}

cv::Point intersectionPoint(const vector<P_Line>& inLines) {
  struct LineParam {
    float a, b, c;
  };

  Vec2f point(0.f, 0.f);

  vector<Line> lines;
  lines.reserve(inLines.size());
  for(auto& line : inLines) {
    float a = line.second.y - line.first.y;
    float b = line.first.x - line.second.x;
    float c = b*line.first.y + a*line.first.x;

    lines.emplace_back(a, b, c);
  }

  estimator.Estimate(lines);
  auto* bestModel = estimator.GetBestModel();
  if(bestModel != nullptr) {
		auto intersectionPoint = bestModel->getIntersectionPoint();

  	point[0] = intersectionPoint.x;
	  point[1] = intersectionPoint.y;
	}
	else {
    throw std::runtime_error("intersectionPoint: No intersections found");
  }

  return {cvRound(point[0]), cvRound(point[1])};
}
