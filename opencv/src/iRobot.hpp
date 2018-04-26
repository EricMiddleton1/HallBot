#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <deque>

#include <opencv2/core/core.hpp>

#include <boost/asio.hpp>

#include "SerialPort.hpp"
#include "PeriodicTimer.hpp"

#include "IConfigurable.hpp"

class iRobot : public IConfigurable {
public:
  iRobot(std::vector<IConfigurable::Param>&& params);
  ~iRobot();

  void setWheels(int left, int right);

  float getDistance() const;

  cv::Vec2f getPosition() const;
  float getAngle() const;

  void setCameraPose(const cv::Vec2f&, float angle);
  void setCameraAngle(float angle);

	void resetCameraScaler();
	bool hasCameraScale() const;
  float getCameraScale() const;
  
  bool getButtonPress() const;

  bool retraceStep();

  static void draw(cv::Mat& image, const cv::Point& pos, float angle, int size = 10.f);

private:
  const int SENSOR_UPDATE_RATE = 100; //milliseconds

  enum class Command {
    Start = 128,
    FullMode = 132,
    Drive = 137,
    DriveDirect = 145,
    LEDs = 139,
    Sensors = 142,
  };

  struct Sensors {
    uint8_t ir;

    union {
      struct {
        uint8_t play : 1;
        uint8_t unused1 : 1;
        uint8_t advance : 1;
        uint8_t unused0 : 5;
      } button;
      uint8_t button_packed;
    };

    int16_t distance;
    int16_t angle;
  };

  void start();
  void recvHandler(std::vector<uint8_t> data);
  void sensorUpdate();
  bool parseSensorStream(Sensors& sensors);
  void processSensorUpdate(const Sensors& sensors);
  uint32_t getTime() const;

  static int16_t parseInt16(const std::vector<uint8_t>& buffer, int start);

  static float dist(const cv::Vec2f& p1, const cv::Vec2f& p2);

  boost::asio::io_service ioService;
  std::unique_ptr<boost::asio::io_service::work> ioWork;
  SerialPort port;

  std::vector<uint8_t> sensorStream;

  mutable float drivenDistance;

  cv::Vec2f pos;
  float angle;
  float botDistAccum, camDistAccum, cameraScale;
	size_t cameraScaleCount, cameraScaleCountMax;

  bool buttonState;
  mutable bool buttonPress;

  mutable std::mutex mutex;

  PeriodicTimer sensorTimer;

  std::thread asyncThread;
};
