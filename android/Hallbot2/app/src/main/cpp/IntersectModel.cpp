#include "IntersectModel.hpp"

#include <limits>
#include <cmath>

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

IntersectModel::IntersectModel() {
}

IntersectModel::IntersectModel(const std::array<Param*,ParamCount>& params)
	:	line1{params[0]}
	,	line2{params[1]} {

	intersection = findIntersection(line1, line2);
}

std::pair<double, std::vector<IntersectModel::Param*>> IntersectModel::Evaluate(
	const std::vector<Param*>& params, double threshold) {

  std::vector<Param*> inliers;
  
  if(intersection) {
    for(const auto line : params) {
      auto ip1 = findIntersection(line1, line),
        ip2 = findIntersection(line2, line);

      float d1 = ip1.distanceTo(intersection),
        d2 = ip2.distanceTo(intersection);
      
      if(std::min(d1, d2) < threshold) {
        inliers.push_back(line);
      }
    }
  }

  auto inlierFraction = static_cast<double>(inliers.size()) / params.size();

  return {inlierFraction, inliers};
}

Point IntersectModel::getIntersectionPoint() const {
  return intersection;
}

Point IntersectModel::findIntersection(const Line* l0, const Line* l1) {
	Point p;

	float det = l0->a*l1->b - l1->a*l0->b;
	if(det != 0) {
		p.x = (l1->b*l0->c - l0->b*l1->c) / det;
		p.y = (l0->a*l1->c - l1->a*l0->c) / det;
	}

  return p;
}
