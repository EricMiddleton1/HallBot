#include "DeviceBuilder.hpp"

DeviceBuilder::DeviceBuilder(std::string&& name, BuildFn&& buildFn)
  : name_{std::move(name)}
  , buildFn_{std::move(buildFn)} {
}

const std::string& DeviceBuilder::name() const {
  return name_;
}

std::unique_ptr<VideoDevice> DeviceBuilder::build(const std::vector<Param>& params) {
  return buildFn_(params);
}
