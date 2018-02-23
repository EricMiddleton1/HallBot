#pragma once

#include "IConfigurable.hpp"

class ComplementaryFilter : public IConfigurable{
public:
  ComplementaryFilter(std::vector<IConfigurable::Param>&& params);

  void set(float value);
  float filter(float delta, float fixed, float dt);

  float get() const;
  
private:
  float alpha, value;
};
