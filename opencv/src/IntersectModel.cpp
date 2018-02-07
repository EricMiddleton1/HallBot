#include "IntersectModel.hpp"

#include <limits>
#include <cmath>

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


IntersectModel::IntersectModel(
  const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& params) {

  Initialize(params);
}

void IntersectModel::Initialize(
  const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& params) {

  if(params.size() != 2) {
    throw std::runtime_error("IntersectModel: Number of input parameters should be 2");
  }

  auto line1 = std::dynamic_pointer_cast<Line>(params[0]),
    line2 = std::dynamic_pointer_cast<Line>(params[1]);
  if(line1 == nullptr || line2 == nullptr) {
    throw std::runtime_error("IntersectModel: params type mismatch (should be Line)");
  }

  std::copy(params.begin(), params.end(), m_MinModelParams.begin());

  intersection = findIntersection(*line1, *line2);
}

std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>>
  IntersectModel::Evaluate(
    const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& params,
    GRANSAC::VPFloat threshold) {

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> inliers;
  for(const auto& param : params) {
    if(ComputeDistanceMeasure(param) < threshold) {
      inliers.push_back(param);
    }
  }

  GRANSAC::VPFloat inlierFraction = GRANSAC::VPFloat(inliers.size()) /
    GRANSAC::VPFloat(params.size());

  return {inlierFraction, inliers};
}

Point IntersectModel::getIntersectionPoint() const {
  return intersection;
}

GRANSAC::VPFloat IntersectModel::ComputeDistanceMeasure(
  std::shared_ptr<GRANSAC::AbstractParameter> param) {

  auto line = std::dynamic_pointer_cast<Line>(param);
  if(line == nullptr) {
    throw std::runtime_error("IntersectModel::ComputeDistanceMeasure() "
      "Invalid parameter");
  }

  //Distance from line to model intersect point
  //http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
  return std::abs(line->a*intersection.x + line->b*intersection.y + line->c) /
    std::sqrt((line->a * line->a) + (line->b * line->b));
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
