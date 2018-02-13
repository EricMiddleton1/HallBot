#pragma once

#include "VideoDevice.hpp"
#include "DeviceBuilder.hpp"

class VideoManager {
public:
  static void registerDevice(DeviceBuilder&& builder);

  static std::unique_ptr<VideoDevice> build(const std::string& name,
    const std::vector<VideoDevice::Param>& params);

  static std::vector<std::string> enumerateDevices();

private:
  static std::vector<DeviceBuilder>* builders_;
};
