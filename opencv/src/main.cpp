#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "Config.hpp"
#include "DeviceManager.hpp"
#include "EdgeDetector.hpp"
#include "HoughTransform.hpp"
#include "VanishingPointDetector.hpp"
#include "Slammer.hpp"
#include "iRobot.hpp"
#include "PID.hpp"

const std::string WINDOW_NAME = "Vanishing Point Detector";
const std::string DEBUG_WINDOW_NAME = "Vanishing Point Detector - Debug";

iRobot::State convertTrackingState(int trackingState);

int main(void)
{
  //Load YAML configuration file
  Config config{"config.yml"};
  
  //Initialize objects based on YAML configuration parameters
  auto videoParams = config.getParams("video_device");
  auto videoName = std::find_if(videoParams.begin(), videoParams.end(),
    [](const auto& param) { return param.first == "name"; });
  if(videoName == videoParams.end()) {
    std::cerr << "[Error] Missing parameter video_device/name" << std::endl;
    return 1;
  }
  auto videoDevice =
    device_cast<VideoDevice>(DeviceManager::build(videoName->second,
    std::move(videoParams)));
  auto edgeDetector{std::make_unique<EdgeDetector>(config.getParams("edge_detector"))};
  auto houghTransformer{std::make_unique<HoughTransform>(
    config.getParams("hough_transform"))};
  auto vpDetector{std::make_unique<VanishingPointDetector>(
    config.getParams("vanishing_point_detector"))};
  auto slammer{std::make_unique<Slammer>(
    config.getParams("slam"))};
  auto steerPID{std::make_unique<PID>(config.getParams("steer_pid"))};

  std::unique_ptr<iRobot> bot;
  if(config.hasEntry("robot")) {
    bot = std::make_unique<iRobot>(config.getParams("robot"));
  }

  //Set PID setpoint
  steerPID->set(0.f);

  cv::Mat track(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Vec2f lastBotPos = {0.f, 0.f};
  
  auto startTime = cv::getTickCount();

  if(bot) {
    bot->setWheels(1.f, 1.f);
  }

  while(cv::waitKey(1) == -1) {
    float steer = 0.f;

    cv::Mat frame, frame_edges, frameAnotated;

    //Try to grab frame from video device
    if(!videoDevice->getFrame(frame)) {
      std::cerr << "[Warning] Failed to fetch frame" << std::endl;
      continue;
    }
    auto frameSize = frame.size();

    //Pass image into SLAM system
    auto pose = slammer->process(frame);
    frameAnotated = slammer->draw();

    auto oldState = bot->getState();
    bot->setState(convertTrackingState(slammer->getTrackingState()));
    auto newState = bot->getState();
    if(newState != oldState) {
      if(newState != iRobot::State::Retracing) {
        bot->setWheels(1.f, 1.f);
      }
    }

    if(newState == iRobot::State::Retracing) {
      std::cout << "[Info] Retracing..." << std::endl;
      bot->retraceStep();
    }
    
    if(!pose.empty()) {
      auto R = pose(cv::Rect(0, 0, 3, 3));
      auto T = pose(cv::Rect(3, 0, 1, 3));

      cv::Mat cameraPos = -(R.inv()) * T;

      //float botX = -pose.at<float>(2, 3), botY = pose.at<float>(0, 3);
      float botX = cameraPos.at<float>(0, 0), botY = cameraPos.at<float>(0, 2);
      std::cout << "[Info] Camera position: (" << botX << ", " << botY << ")\n";
      //std::cout << "[Info] Camera position: " << cameraPos << std::endl;

      bot->setCameraPose({botX, botY}, 0.f);
    }
    cv::Vec2f newBotPos = bot->getPosition();
    cv::Vec2f trackP1 = lastBotPos * 300.f/4.f;
    cv::Vec2f trackP2 = newBotPos * 300.f/4.f;
    cv::line(track, {300+trackP1[0], 300+trackP1[1]},
      {300+trackP2[0], 300+trackP2[1]}, cv::Scalar(255, 0, 0), 5);
    lastBotPos = newBotPos;

/*
    //Run through vanishing point pipeline
    if(vpDetector->enabled()) {
      frame_edges = edgeDetector->process(frame);
      auto lines = houghTransformer->process(frame_edges);
      cv::Vec2f vanishingPoint;
      auto vpConfidence = vpDetector->process(lines, vanishingPoint);
      
      //Draw hough lines over frame_edges image
      houghTransformer->drawLines(lines, frame_edges);
    
      //Update steer PID if vanishing point confidence > 0 (it detected a vanishing point)
      if(vpConfidence > 0.f) {
        auto curX = vanishingPoint[0] / frameSize.width;
        hallwayX = 0.1f*curX + 0.9f*hallwayX;
        steer = steerPID->update(hallwayX, 0.015);
        steer = std::max(-100.f, std::min(100.f, steer));

        circle(frame, {cvRound(hallwayX*frameSize.width), cvRound(vanishingPoint[1])}, 15,
          cv::Scalar(0, 0, 255), -1);
        circle(frame, {cvRound(vanishingPoint[0]), cvRound(vanishingPoint[1])}, 15,
          cv::Scalar(255, 0, 0), -1);

        std::cout << "[Info] Hallway X = " << hallwayX << ", wheel actuation = "
          << steer << std::endl;
      }
    }
*/
    //Send latest robot wheel values to bot (if used)
    if(bot) {
      bot->setWheels(1.f - steer, 1.f + steer);
    }

    //Display raw rame and edges/hough lines frame
    if(vpDetector->enabled()) {
      imshow(WINDOW_NAME, frame);
      imshow(DEBUG_WINDOW_NAME, frame_edges);
    }
    imshow("SLAM Frame", frameAnotated);
    imshow("Camera Track", track);

    //Stop loop stopwatch
		auto endTime = cv::getTickCount();
    /*
		std::cout << "[Info] Processed frame in " << static_cast<float>(endTime - startTime)
			/ cv::getTickFrequency()*1000.f << "ms" << std::endl; */
    startTime = endTime;

  }

  return 0;
}


iRobot::State convertTrackingState(int trackingState) {
  iRobot::State state;

  switch(trackingState) {
    case ORB_SLAM2::Tracking::OK:
      state = iRobot::State::Tracking;
    break;

    case ORB_SLAM2::Tracking::LOST:
      state = iRobot::State::Retracing;
    break;

    default:
      state = iRobot::State::Initializing;
    break;
  }

  return state;
}
