#include "DeviceBuilder.hpp"

DeviceBuilder::DeviceBuilder(std::string&& name, BuildFn&& buildFn)
  : name_{std::move(name)}
  , buildFn_{std::move(buildFn)} {
}

const std::string& DeviceBuilder::name() const {
  return name_;
}

std::unique_ptr<IConfigurable> DeviceBuilder::build(std::vector<IConfigurable::Param>&&
  params) {
  return buildFn_(std::move(params));
}
