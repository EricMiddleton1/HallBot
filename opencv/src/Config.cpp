#include "Config.hpp"

#include <stdexcept>

Config::Config(const std::string& file)
  : node{YAML::LoadFile(file)} {
}

bool Config::hasEntry(const std::string& name) const {
  return node[name];
}

std::vector<IConfigurable::Param> Config::getParams(const std::string& name) const {
  std::vector<IConfigurable::Param> params;

  auto subNode = node[name];
  if(!subNode) {
    throw std::runtime_error(std::string("Config: Entry ") + name + " not found");
  }

  for(const auto& param : subNode) {
    auto key = param.first.as<std::string>();
    auto value = param.second.as<std::string>();
    params.emplace_back(key, value);
  }

  return params;
}
