#pragma once

#include <functional>
#include <vector>
#include <utility>
#include <string>
#include <memory>

#include "IConfigurable.hpp"

class DeviceBuilder {
public:
  using BuildFn = std::function<std::unique_ptr<IConfigurable>(
    std::vector<IConfigurable::Param>&&)>;

  DeviceBuilder(std::string&& name, BuildFn&& buildFn);

  const std::string& name() const;

  std::unique_ptr<IConfigurable> build(std::vector<IConfigurable::Param>&& params);

private:
  std::string name_;
  BuildFn buildFn_;
};
