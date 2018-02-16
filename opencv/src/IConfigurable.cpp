#include "IConfigurable.hpp"

#include <algorithm>

IConfigurable::IConfigurable(const std::vector<std::string>& requiredParams,
  std::vector<Param>&& params)
  : params_(std::move(params)) {

  for(const auto& param : requiredParams) {
    if(!paramExists(param)) {
      throw ParamMissingException(param);
    }
  }
}

bool IConfigurable::paramExists(const std::string& key) const {
  return paramIndex(key) != -1;
}

const std::string& IConfigurable::getParam(const std::string& key) const {
  auto index = paramIndex(key);
  if(index == -1) {
    throw ParamMissingException(key);
  }
  else {
    return params_[index].second;
  }
}

std::string IConfigurable::getParam(const std::string& key,
  const std::string& defaultValue) const {
  auto index = paramIndex(key);
  if(index == -1) {
    return defaultValue;
  }
  else {
    return params_[index].second;
  }
}

int IConfigurable::paramIndex(const std::string& key) const {
  auto found = std::find_if(params_.begin(), params_.end(),
    [&key](const auto& param) {
      return param.first == key;
    });

  if(found == params_.end()) {
    return -1;
  }
  else {
    return found - params_.begin();
  }
}

ParamMissingException::ParamMissingException(const std::string& key)
  : key_{key}
  , msg_{std::string("Missing Parameter '") + key + "'"} {
}

const std::string& ParamMissingException::key() const {
  return key_;
}

const char* ParamMissingException::what() const noexcept {
  return msg_.c_str();
}
