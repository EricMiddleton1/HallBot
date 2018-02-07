/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdexcept>

using namespace cv;
using namespace std;

using Line = pair<Point, Point>;

/// Global variables

/** General variables */
Mat src, edges;
Mat src_gray;
Mat standard_hough;
int min_threshold = 50;
int max_trackbar = 150;

const int min_theta_trackbar = 0;
const int max_theta_trackbar = 180;

const char* standard_name = "Standard Hough Lines Demo";

int s_trackbar = max_trackbar;
int s_theta_trackbar = min_theta_trackbar;

/// Function Headers
void help();
void Standard_Hough( int, void* );
Vec2f intersectionPoint(const vector<Line>& inLines);

/**
 * @function main
 */
int main( int argc, char** argv )
{
   // Read the image
    String imageName("../data/building.jpg"); // by default
    if (argc > 1)
    {
        imageName = argv[1];
    }
    src = imread( imageName, IMREAD_COLOR );

   if( src.empty() )
     { help();
       return -1;
     }

   /// Pass the image to gray
   cvtColor( src, src_gray, COLOR_RGB2GRAY );

   /// Apply Canny edge detector
   Canny( src_gray, edges, 50, 200, 3 );

   /// Create Trackbars for Thresholds
   char thresh_label[50];
   sprintf( thresh_label, "Thres: %d + input", min_threshold );

   namedWindow( standard_name, WINDOW_AUTOSIZE );
   createTrackbar( thresh_label, standard_name, &s_trackbar, max_trackbar,
    Standard_Hough);
   createTrackbar("Angle Threshold (degrees)", standard_name, &s_theta_trackbar,
    max_theta_trackbar, Standard_Hough);

   /// Initialize
   Standard_Hough(0, 0);
   waitKey(0);
   return 0;
}

/**
 * @function help
 * @brief Indications of how to run this program and why is it for
 */
void help()
{
  printf("\t Hough Transform to detect lines \n ");
  printf("\t---------------------------------\n ");
  printf(" Usage: ./HoughLines_Demo <image_name> \n");
}



/**
 * @function Standard_Hough
 */
void Standard_Hough( int, void* )
{
  vector<Vec2f> s_lines;
  vector<Line> pointLines;
  cvtColor( edges, standard_hough, COLOR_GRAY2BGR );

  /// 1. Use Standard Hough Transform
  HoughLines( edges, s_lines, 1, CV_PI/180, min_threshold + s_trackbar, 0, 0 );

  /// Show the result
  for( size_t i = 0; i < s_lines.size(); i++ )
     {
      float r = s_lines[i][0], t = s_lines[i][1];
      float rad_min = s_theta_trackbar * CV_PI/180;

      if( (t < rad_min) || (t > (CV_PI - rad_min)) ) {
        continue;
      }

      double cos_t = cos(t), sin_t = sin(t);
      double x0 = r*cos_t, y0 = r*sin_t;
      double alpha = 1000;

       Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
       Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
        
       pointLines.emplace_back(pt1, pt2);

       line( standard_hough, pt1, pt2, Scalar(255,0,0), 3, LINE_AA);
     }
    
   try {
     auto intersection = intersectionPoint(pointLines);
     circle(standard_hough, {intersection[0], intersection[1]}, 50, Scalar(0, 0, 255),
      -1);
   }
   catch(const std::exception& e) {
   }

   imshow( standard_name, standard_hough );
}

Vec2f intersectionPoint(const vector<Line>& inLines) {
  struct LineParam {
    float a, b, c;
  };

  Vec2f point(0.f, 0.f);
  int count = 0;

  vector<LineParam> pLines;
  pLines.reserve(inLines.size());
  for(auto& line : inLines) {
    float a = line.second.y - line.first.y;
    float b = line.first.x - line.second.x;
    float c = b*line.first.y + a*line.first.x;

    pLines.push_back({a, b, c});
  }

  for(int i = 0; i < pLines.size(); ++i) {
    const auto& l0 = pLines[i];
    for(int j = i+1; j < pLines.size(); ++j) {
      const auto& l1 = pLines[j];
      
      float det = l0.a*l1.b - l1.a*l0.b;
      if(det != 0) {
        float x = (l1.b*l0.c - l0.b*l1.c) / det;
        float y = (l0.a*l1.c - l1.a*l0.c) / det;

        point[0] += x;
        point[1] += y;

        count++;
      }
    }
  }

  if(count == 0) {
    throw std::runtime_error("intersectionPoint: No intersections found");
  }
  else {
    point[0] /= count;
    point[1] /= count;
  }

  return point;
}
