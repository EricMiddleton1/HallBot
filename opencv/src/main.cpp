#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <string>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <cmath>

#include <yaml-cpp/yaml.h>

#include "Config.hpp"
#include "DeviceManager.hpp"
#include "EdgeDetector.hpp"
#include "HoughTransform.hpp"
#include "VanishingPointDetector.hpp"
#include "Slammer.hpp"
#include "iRobot.hpp"
#include "Driver.hpp"
#include "PID.hpp"
#include "CloudComputer.hpp"

const std::string WINDOW_NAME = "Vanishing Point Detector";
const std::string DEBUG_WINDOW_NAME = "Vanishing Point Detector - Debug";

Driver::Mode getDrivingMode(int trackingState);

float distanceToLine(const cv::Vec4f &line, const cv::Vec2f point);
float getRotationY(const cv::Mat &rotation);

void drawHallway(cv::Mat &image, const cv::Vec4f &line, float width, float scale,
                 cv::Point offset);

int main(void)
{
  //Load YAML configuration file
  Config config{"config.yml"};

  //Initialize objects based on YAML configuration parameters
  auto videoParams = config.getParams("video_device");
  auto videoName = std::find_if(videoParams.begin(), videoParams.end(),
                                [](const auto &param) { return param.first == "name"; });
  if (videoName == videoParams.end())
  {
    std::cerr << "[Error] Missing parameter video_device/name" << std::endl;
    return 1;
  }
  auto videoDevice =
      device_cast<VideoDevice>(DeviceManager::build(videoName->second,
                                                    std::move(videoParams)));
  auto edgeDetector{std::make_unique<EdgeDetector>(
      config.getParams("edge_detector"))};
  auto houghTransformer{std::make_unique<HoughTransform>(
      config.getParams("hough_transform"))};
  auto cloudComp{std::make_unique<CloudComputer>(
      config.getParams("cloud_comp"))};
  auto vpDetector{std::make_unique<VanishingPointDetector>(
      config.getParams("vanishing_point_detector"))};

  std::shared_ptr<iRobot> bot;
  std::unique_ptr<Driver> driver;
  if (config.hasEntry("robot"))
  {
    bot = std::make_shared<iRobot>(config.getParams("robot"));
    driver = std::make_unique<Driver>(config.getParams("driver"));
    driver->setRobot(bot);
  }

  auto slammer{std::make_unique<Slammer>(
      config.getParams("slam"))};
  auto steerPID{std::make_unique<PID>(config.getParams("steer_pid"))};

  //Set PID setpoint
  steerPID->set(0.f);

  cv::Vec2f lastBotPos = {0.f, 0.f};

  //Wait to start until play button pressed
  if (bot)
  {
    while (!bot->getButtonPress())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  auto startTime = cv::getTickCount();

  if (bot)
  {
    driver->start();
  }

  while (cv::waitKey(1) == -1)
  {
    cv::Mat frame, frame_edges, frameAnotated;
    cv::Vec2f cameraPos; //Current position of camera

    //Try to grab frame from video device
    if (!videoDevice->getFrame(frame))
    {
      std::cerr << "[Warning] Failed to fetch frame" << std::endl;
      continue;
    }
    auto frameSize = frame.size();

    //Pass image into SLAM system
    auto pose = slammer->process(frame);
    frameAnotated = slammer->draw();

    if (!pose.empty())
    {
      auto R = pose(cv::Rect(0, 0, 3, 3));
      auto T = pose(cv::Rect(3, 0, 1, 3));
      float angle;

      cv::Mat camera3dPos = -(R.inv()) * T;

      cameraPos = {camera3dPos.at<float>(0, 0), camera3dPos.at<float>(0, 2)};
      angle = getRotationY(R);

      //float botX = -pose.at<float>(2, 3), botY = pose.at<float>(0, 3);
      //float botX = cameraPos.at<float>(0, 0), botY = cameraPos.at<float>(0, 2);
      //std::cout << "[Info] Camera position: (" << botX << ", " << botY << ")\n";
      //std::cout << "[Info] Camera position: " << cameraPos << std::endl;

      if (bot)
      {
        bot->setCameraPose(cameraPos, angle);
      }

      //Update cloud computer with current camera position
      cloudComp->setCameraPos(cameraPos);
      /*
      std::cout << "[Info] Position: (" << cameraPos[0] << ", " << cameraPos[1]
        << ", " << angle*180.f/3.14f << "), scale: " << bot->getCameraScale()
        << std::endl;
        */
    }

    if (bot)
    {
      auto botPos = bot->getPosition();

      auto hallwayLine = cloudComp->getGreenLine();

      if (hallwayLine[0] != 0.f)
      {
        float hallwayAngle = cloudComp->getGreenTheta();
        float hallwayWidth = -cloudComp->getWidth();
        float distToHallwayEnd = cloudComp->distToFacingWall();
        auto hallPos = -cloudComp->getHallPosition()[1];

        //std::cout << hallPos << ", " << hallwayWidth << std::endl;
        
        driver->hallwayWidth(hallwayWidth);
        driver->posInHallway(hallPos);
        driver->hallwayAngle(hallwayAngle);
        driver->setDistanceToHallwayEnd(distToHallwayEnd);
      }
    }

    if (driver)
    {
      driver->mode(getDrivingMode(slammer->getTrackingState()));
      driver->update();
    }

    //Display raw rame and edges/hough lines frame
    if (vpDetector->enabled())
    {
      imshow(WINDOW_NAME, frame);
      imshow(DEBUG_WINDOW_NAME, frame_edges);
    }
    //imshow("SLAM Frame", frameAnotated);
    //imshow("Camera Track", track);

    //Stop loop stopwatch
    auto endTime = cv::getTickCount();
    /*
		std::cout << "[Info] Processed frame in " << static_cast<float>(endTime - startTime)
			/ cv::getTickFrequency()*1000.f << "ms" << std::endl; */
    startTime = endTime;

    // if tracking is OK
    if (slammer->getStateofTrack() == 2)
    {
      cloudComp->display2D(slammer->getMap());
    }
  }

  return 0;
}

Driver::Mode getDrivingMode(int trackingState)
{
  Driver::Mode mode;

  switch (trackingState)
  {
  case ORB_SLAM2::Tracking::OK:
    mode = Driver::Mode::Tracking;
    break;

  case ORB_SLAM2::Tracking::LOST:
    mode = Driver::Mode::Retracing;
    break;

  default:
    mode = Driver::Mode::Initializing;
    break;
  }

  return mode;
}

float distanceToLine(const cv::Vec4f &line, const cv::Vec2f point)
{
  cv::Vec2f l_pt1{line[2], line[3]}, l_pt2{line[2] + line[0], line[3] + line[1]};

  //m = (P2.y - P1.y) / (P2.x - P1.x)
  float m = (l_pt2[1] - l_pt1[1]) / (l_pt2[0] - l_pt1[0]);
  //d = P1.y - m*P1.x;
  float d = l_pt1[1] - m * l_pt1[0];

  float a = m;
  float b = -1.f;
  float c = d;

  float denom = std::sqrt(a * a + b * b);
  float numer = std::fabs(a * point[0] + b * point[1] + c);

  return numer / denom;
}

float sqr(float x)
{
  return x * x;
}

float getRotationY(const cv::Mat &rotation)
{
  //Create forward vector
  cv::Mat forward(3, 1, CV_32F);
  forward.at<float>(0) = 0.f;
  forward.at<float>(1) = 0.f;
  forward.at<float>(2) = 1.f;

  //Rotate forward vector by matrix
  cv::Mat rotated = rotation * forward;

  //Extract X, Y coordinates
  float x = rotated.at<float>(0), z = rotated.at<float>(2);

  //Get angle from forward (Z=1)
  return std::atan2(x, z);
}

void drawHallway(cv::Mat &image, const cv::Vec4f &hallwayLine, float width, float scale,
                 cv::Point offset)
{

  //Calculate hallway line vectors
  cv::Vec2f lineDir{hallwayLine[0], hallwayLine[1]},
      linePt{hallwayLine[2], hallwayLine[3]},
      v1 = (linePt - 800 * lineDir), v2 = (linePt + 800 * lineDir);

  //Calculate hallway normal, border vectors
  float widthScale = width * scale / 2.f;
  cv::Vec2f hallwayNormal{-hallwayLine[1], hallwayLine[0]};
  cv::Vec2f wall1v1{v1 + widthScale * hallwayNormal},
      wall1v2{v2 + widthScale * hallwayNormal},
      wall2v1{v1 - widthScale * hallwayNormal}, wall2v2{v2 - widthScale * hallwayNormal};

  //Convert vectors to points
  cv::Point pt1 = {v1[0], -v1[1]}, pt2 = {v2[0], -v2[1]},
            wall1pt1{wall1v1[0], -wall1v1[1]}, wall1pt2{wall1v2[0], -wall1v2[1]},
            wall2pt1{wall2v1[0], -wall2v1[1]}, wall2pt2{wall2v2[0], -wall2v2[1]};

  //Draw hallway lines
  cv::line(image, pt1 + offset, pt2 + offset, cv::Scalar(0, 255, 0), 1);
  cv::line(image, wall1pt1 + offset, wall1pt2 + offset, cv::Scalar(0, 255, 0), 2);
  cv::line(image, wall2pt1 + offset, wall2pt2 + offset, cv::Scalar(0, 255, 0), 2);
}
