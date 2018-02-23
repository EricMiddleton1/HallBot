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
#include "iRobot.hpp"
#include "PID.hpp"

const std::string WINDOW_NAME = "Vanishing Point Detector";
const std::string DEBUG_WINDOW_NAME = "Vanishing Point Detector - Debug";

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
  auto steerPID{std::make_unique<PID>(config.getParams("steer_pid"))};

  std::unique_ptr<iRobot> bot;
  if(config.hasEntry("robot")) {
    bot = std::make_unique<iRobot>(config.getParams("robot"));
  }
 
  // previous angle from iRobot
  float prev_bot_theta = 0;
  
  steerPID->set(0.5f);

  while(cv::waitKey(1) == -1) {
    //Start loop stopwatch
		int startTime = cv::getTickCount();

    cv::Mat frame, frame_edges;

    //Try to grab frame from video device
    if(!videoDevice->getFrame(frame)) {
      std::cerr << "[Warning] Failed to fetch frame" << std::endl;
      continue;
    }
    auto frameSize = frame.size();
    
    //Run through image processing pipeline
    frame_edges = edgeDetector->process(frame);
    auto lines = houghTransformer->process(frame_edges);
    cv::Vec2f vanishingPoint;
    auto vpConfidence = vpDetector->process(lines, vanishingPoint);

    //Draw hough lines over frame_edges image
    houghTransformer->drawLines(lines, frame_edges);
   	
		float steer = 0.f;
 
    //Update steer PID if vanishing point confidence > 0 (it detected a vanishing point)
    if(vpConfidence > 0.f) {
      auto curX = vanishingPoint[0] / frameSize.width;
      
      // Angle between direction of robot and vantage point based on camera.
      // Camera is 54 x 41 degrees.
      // Theta will be positive if VP is to the right or negative if VP is to the left. 
      float cam_theta = (curX - 0.5) * (54 * CV_PI / 180);

      // Angle from iRobot
      float bot_theta = 0.f, alpha = 0.f;
      
      if(bot) {
        bot_theta = bot->getAngle();
        alpha = 0.9f;
      }

      // Complementary filter
      float d_bot_theta = bot_theta - prev_bot_theta;
      float angle_n = (alpha * (prev_bot_theta + (d_bot_theta * 0.015))) +
        ((1 - alpha) * cam_theta);
      std::cout << "Camera angle: " << 180.f*cam_theta/CV_PI << "\niRobot angle: "
				<< 180.f*bot_theta/CV_PI << "\n";
      	
      steer = steerPID->update(angle_n, 0.015);
      steer = std::max(-100.f, std::min(100.f, steer));

      circle(frame, {cvRound(vanishingPoint[0]), cvRound(vanishingPoint[1])}, 15,
        cv::Scalar(255, 0, 0), -1);
    }

    //Send latest robot wheel values to bot (if used)
    if(bot) {
      bot->setWheels(100 - steer, 100 + steer);
    }

    //Stop loop stopwatch
		int endTime = cv::getTickCount();
		std::cout << "[Info] Processed frame in " << static_cast<float>(endTime - startTime)
			/ cv::getTickFrequency()*1000.f << "ms" << std::endl;

    //Display raw rame and edges/hough lines frame
    imshow(WINDOW_NAME, frame);
    imshow(DEBUG_WINDOW_NAME, frame_edges);
  }

  return 0;
}
