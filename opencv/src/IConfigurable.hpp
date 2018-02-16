#pragma once

#include <vector>
#include <utility>
#include <string>
#include <exception>

class IConfigurable {
public:
  using Param = std::pair<std::string, std::string>;

  IConfigurable(const std::vector<std::string>& requiredParams,
    std::vector<Param>&& params);
  virtual ~IConfigurable() {}

protected:
	bool paramExists(const std::string& key) const;
	const std::string& getParam(const std::string& key) const;
  std::string getParam(const std::string& key, const std::string& defaultValue) const;

private:
	int paramIndex(const std::string& key) const;
	std::vector<Param> params_;
};

class ParamMissingException : public std::exception {
public:
  ParamMissingException(const std::string& key);

  const std::string& key() const;

  const char* what() const noexcept override;
private:
  std::string key_;
  std::string msg_;
};
