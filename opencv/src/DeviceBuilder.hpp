#pragma once

#include <functional>
#include <vector>
#include <utility>
#include <string>
#include <memory>

#include "VideoDevice.hpp"

class DeviceBuilder {
public:
  using Param = std::pair<std::string, std::string>;
  using BuildFn = std::function<std::unique_ptr<VideoDevice>(
    const std::vector<Param>&)>;

  DeviceBuilder(std::string&& name, BuildFn&& buildFn);

  const std::string& name() const;

  std::unique_ptr<VideoDevice> build(const std::vector<Param>& params);

private:
  std::string name_;
  BuildFn buildFn_;
};
