#include <jni.h>
#include <string>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "IntersectModel.hpp"
#include "GRANSAC.hpp"

using namespace cv;
using namespace std;

extern "C" {
using P_Line = pair<cv::Point, cv::Point> ;

GRANSAC::RANSAC<IntersectModel> estimator;

const int min_threshold = 50;
//const int max_trackbar = 150;
//const int min_theta_trackbar = 0;
//const int max_theta_trackbar = 180;
//const int max_ransac_threshold = 500;
//const int max_ransac_iterations = 1000;
//
//const int TARGET_FRAME_HEIGHT = 480;
//
//const string PORT = "/dev/ttyUSB0";

int s_trackbar = 28; //60
int v_theta_trackbar = 10;
int h_theta_trackbar = 5;
//int ransac_threshold_trackbar = 10;
//int ransac_iterations_trackbar = 100;
}
extern "C" cv::Point detectVanishingPoint(const Mat& edges, Mat& hough);
extern "C" cv::Point intersectionPoint(const vector<P_Line>& inLines);

extern "C" {
JNIEXPORT jstring

JNICALL
Java_cpre575_hallbot2_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

jstring
Java_cpre575_hallbot2_MainActivity_validate(
        JNIEnv *env,
        jobject /* this */,
        jlong addrGray,
        jlong addrRgba) {
    cv::Rect();
    cv::Mat();
    std::string hello2 = "Hello from validate";
    return env->NewStringUTF(hello2.c_str());
}
void
Java_cpre575_hallbot2_OpenCVTest0_canny(
        JNIEnv *env,
        jobject /* this */,
        jlong matrixImgAddr) {
    Mat *mat = (Mat *) matrixImgAddr;
    Mat frame_gray;
    cvtColor(*mat, frame_gray, COLOR_RGB2GRAY);
    Canny(frame_gray, *mat, 50, 200, 3);
}

void
Java_cpre575_hallbot2_OpenCVTestHough_vantagePoint(
        JNIEnv *env,
        jobject /* this */,
        jlong matrixImgAddr) {
    Mat *frame_edges = (Mat *) matrixImgAddr;
    Mat frame_hough;
    auto vanishingPoint = detectVanishingPoint(*frame_edges, frame_hough);
}

}
extern "C"
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
extern "C"
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

