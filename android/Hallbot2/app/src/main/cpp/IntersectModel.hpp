#pragma once

#include <array>
#include <vector>

struct Line {
  Line();
  Line(float a, float b, float c);

  float a, b, c;
};

struct Point {
  float x, y;

  Point();
  Point(float x, float y);

  operator bool() const;

  float distanceTo(const Point& other) const;
};

class IntersectModel {
public:
	using Param = Line;
	static const int ParamCount = 2;
	
	IntersectModel();
  IntersectModel(const std::array<Param*,2>& params);

  std::pair<double, std::vector<Param*>> Evaluate(const std::vector<Param*>& params,
      double threshold);

  Point getIntersectionPoint() const;

private:
  static Point findIntersection(const Line* l0, const Line* l1);

  Line *line1, *line2;
  Point intersection;
};
