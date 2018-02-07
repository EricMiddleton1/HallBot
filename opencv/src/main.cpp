#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <string>
#include <iostream>
#include <stdexcept>

#include "GRANSAC.hpp"
#include "IntersectModel.hpp"

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

int s_trackbar = 60;
int s_theta_trackbar = 10;
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

  namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
  namedWindow(DEBUG_WINDOW_NAME, WINDOW_AUTOSIZE);

  createTrackbar("Hough Threshold", DEBUG_WINDOW_NAME, &s_trackbar, max_trackbar,
    nullptr);
  createTrackbar("Angle Threshold (degrees)", DEBUG_WINDOW_NAME, &s_theta_trackbar,
    max_theta_trackbar, nullptr);
  createTrackbar("RANSAC Threshold", DEBUG_WINDOW_NAME, &ransac_threshold_trackbar,
    max_ransac_threshold, updateRansac);
  createTrackbar("RANSAC Max Iterations", DEBUG_WINDOW_NAME, &ransac_iterations_trackbar,
    max_ransac_iterations, updateRansac);

  updateRansac(0, 0);

  while(waitKey(10) == -1) {
    Mat frame, frame_gray, frame_edges, frame_hough;
    cap >> frame;

    if(frame.empty()) {
      //Restart video
      cap.set(CAP_PROP_POS_AVI_RATIO, 0);
      
      continue;
    }

    cvtColor(frame, frame_gray, COLOR_RGB2GRAY);
    Canny(frame_gray, frame_edges, 50, 200, 3);
    
    try {
      auto vanishingPoint = detectVanishingPoint(frame_edges, frame_hough);
      circle(frame, vanishingPoint, 50, Scalar(0, 0, 255),
        -1);
    }
    catch(const exception& e) {
    
    }

    imshow(WINDOW_NAME, frame);
    imshow(DEBUG_WINDOW_NAME, frame_hough);
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
    float rad_min = s_theta_trackbar * CV_PI/180;

    if( (t < rad_min) || (t > (CV_PI - rad_min)) ) {
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
  circle(hough, vanishingPoint, 50, Scalar(0, 0, 255),
    -1);

  return vanishingPoint;
}

cv::Point intersectionPoint(const vector<P_Line>& inLines) {
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

    point[0] = intersectionPoint.x;
    point[1] = intersectionPoint.y;
  }
  else {
    throw std::runtime_error("intersectionPoint: No intersections found");
  }

  return {cvRound(point[0]), cvRound(point[1])};
}
