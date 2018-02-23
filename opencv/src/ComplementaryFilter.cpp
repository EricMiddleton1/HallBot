#include "ComplementaryFilter.hpp"

ComplementaryFilter::ComplementaryFilter(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"alpha"}, std::move(params) }
  , alpha  {std::stof(getParam("alpha"))}
  , value  {0.f} {

}

void ComplementaryFilter::set(float _value) {
  value = _value;
}

float ComplementaryFilter::filter(float delta, float fixed, float dt) {
  value = alpha*(value + delta*dt) + (1.f-alpha)*fixed;

  return value;
}

float ComplementaryFilter::get() const {
  return value;
}
