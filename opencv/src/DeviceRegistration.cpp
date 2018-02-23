#include "DeviceRegistration.hpp"

#include "DeviceManager.hpp"

DeviceRegistration::DeviceRegistration(DeviceBuilder&& builder) {
  DeviceManager::registerDevice(std::move(builder));
}
