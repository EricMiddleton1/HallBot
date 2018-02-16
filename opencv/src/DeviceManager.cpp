#include "DeviceManager.hpp"

std::vector<DeviceBuilder>* DeviceManager::builders_ = nullptr;

void DeviceManager::registerDevice(DeviceBuilder&& builder) {
  static std::vector<DeviceBuilder> localBuilders;

  if(builders_ == nullptr) {
    builders_ = &localBuilders;
  }

  localBuilders.emplace_back(std::move(builder));
}

std::unique_ptr<IConfigurable> DeviceManager::build(const std::string& name,
  std::vector<IConfigurable::Param>&& params) {

  if(builders_ == nullptr) {
    return nullptr;
  }
  
  auto found = std::find_if(builders_->begin(), builders_->end(),
    [&name](const auto& builder) {
      return builder.name() == name;
    });

  if(found == builders_->end()) {
    return nullptr;
  }
  else {
    return found->build(std::move(params));
  }
}

std::vector<std::string> DeviceManager::enumerateDevices() {
  if(builders_ == nullptr) {
    return {};
  }

  std::vector<std::string> devices;
  devices.reserve(builders_->size());

  for(const auto& builder : *builders_) {
    devices.push_back(builder.name());
  }

  return devices;
}
