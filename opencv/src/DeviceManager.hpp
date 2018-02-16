#pragma once

#include "VideoDevice.hpp"
#include "DeviceBuilder.hpp"

class DeviceManager {
public:
  static void registerDevice(DeviceBuilder&& builder);

  static std::unique_ptr<IConfigurable> build(const std::string& name,
    std::vector<IConfigurable::Param>&& params);

  static std::vector<std::string> enumerateDevices();

private:
  static std::vector<DeviceBuilder>* builders_;
};

template<typename Derived, typename Base, typename Del>
std::unique_ptr<Derived>
device_cast(std::unique_ptr<Base, Del>&& ptr) {
	if(Derived *result = dynamic_cast<Derived*>(ptr.get())) {
		ptr.release();
		return std::unique_ptr<Derived>(result);
	}
	else {
		return std::unique_ptr<Derived>(nullptr);
	}
}
