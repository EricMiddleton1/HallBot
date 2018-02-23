#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

#include "IConfigurable.hpp"

class Config {
public:
  Config(const std::string& file);
  
  bool hasEntry(const std::string& name) const;
  std::vector<IConfigurable::Param> getParams(const std::string& name) const;

private:
  YAML::Node node;
};
