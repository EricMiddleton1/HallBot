#include "DeviceRegistration.hpp"

#include "VideoManager.hpp"

DeviceRegistration::DeviceRegistration(DeviceBuilder&& builder) {
  VideoManager::registerDevice(std::move(builder));
}
