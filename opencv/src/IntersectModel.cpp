#include "IntersectModel.hpp"

#include <limits>
#include <cmath>
#include <iostream>

Line::Line() {
}

Line::Line(float _a, float _b, float _c)
  : a{_a}
  , b{_b}
  , c{_c} {
}

Point::Point()
  : x{std::numeric_limits<float>::infinity()}
  , y{std::numeric_limits<float>::infinity()} {
}

Point::Point(float _x, float _y)
  : x{_x}
  , y{_y} {
}

Point::operator bool() const {
  return (x != std::numeric_limits<float>::infinity()) &&
    (y != std::numeric_limits<float>::infinity());
}

float Point::distanceTo(const Point& other) const {
  auto dx = x - other.x,
    dy = y - other.y;

  return std::sqrt(dx*dx + dy*dy);
}

IntersectModel::IntersectModel(
  const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& params) {

  Initialize(params);
}

void IntersectModel::Initialize(
  const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& params) {

  if(params.size() != 2) {
    throw std::runtime_error("IntersectModel: Number of input parameters should be 2");
  }

  auto pl1 = std::dynamic_pointer_cast<Line>(params[0]),
    pl2 = std::dynamic_pointer_cast<Line>(params[1]);
  if(pl1 == nullptr || pl2 == nullptr) {
    throw std::runtime_error("IntersectModel: params type mismatch (should be Line)");
  }
  line1 = *pl1;
  line2 = *pl2;

  std::copy(params.begin(), params.end(), m_MinModelParams.begin());

  intersection = findIntersection(line1, line2);
  intersectionAvg = intersection;
}

std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>>
  IntersectModel::Evaluate(
    const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& params,
    GRANSAC::VPFloat threshold) {
  
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> inliers;
  
  if(intersection) {
    for(const auto& param : params) {
      auto line = std::dynamic_pointer_cast<Line>(param);
      if(line == nullptr) {
        throw std::runtime_error("IntersectModel::Evaluate() Invalid parameter");
      }

      auto ip1 = findIntersection(line1, *line),
        ip2 = findIntersection(line2, *line);

      float d1 = ip1.distanceTo(intersection),
        d2 = ip2.distanceTo(intersection);
      
      if(std::min(d1, d2) < threshold) {
        inliers.push_back(param);
        
        if(d1 == d2) {
          intersectionAvg.x = (intersectionAvg.x + ip1.x + ip2.x)/3.f;
          intersectionAvg.y = (intersectionAvg.y + ip1.y + ip2.y)/3.f;
        }
        else if(d1 < d2) {
          intersectionAvg.x = (intersectionAvg.x + ip1.x)/2.f;
          intersectionAvg.y = (intersectionAvg.y + ip1.y)/2.f;
        }
        else {
          intersectionAvg.x = (intersectionAvg.x + ip2.x)/2.f;
          intersectionAvg.y = (intersectionAvg.y + ip2.y)/2.f;
        }
      }
    }
  }

  GRANSAC::VPFloat inlierFraction = GRANSAC::VPFloat(inliers.size()) /
    GRANSAC::VPFloat(params.size());

  return {inlierFraction, inliers};
}

Point IntersectModel::getIntersectionPoint() const {
  return intersection;
}

Point IntersectModel::getIntersectionPointAvg() const {
  return intersectionAvg;
}

GRANSAC::VPFloat IntersectModel::ComputeDistanceMeasure(
  std::shared_ptr<GRANSAC::AbstractParameter> param [[gnu::unused]]) {

  throw std::runtime_error("IntersectMethod::ComputeDistanceMeasure(): "
    "Unimplemented method");
}

Point IntersectModel::findIntersection(const Line& l0, const Line& l1) {
	Point p;

	float det = l0.a*l1.b - l1.a*l0.b;
	if(det != 0) {
		p.x = (l1.b*l0.c - l0.b*l1.c) / det;
		p.y = (l0.a*l1.c - l1.a*l0.c) / det;
	}

  return p;
}
