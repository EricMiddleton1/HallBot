#pragma once

#include "AbstractModel.hpp"


struct Line : public GRANSAC::AbstractParameter {
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

class IntersectModel : public GRANSAC::AbstractModel<2> {
public:
  IntersectModel(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& params);

  void Initialize(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>&
    params) override;

  std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>>
    Evaluate(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& params,
      GRANSAC::VPFloat threshold) override;

  Point getIntersectionPoint() const;

private:
  GRANSAC::VPFloat ComputeDistanceMeasure(std::shared_ptr<GRANSAC::AbstractParameter>
    params) override;

  static Point findIntersection(const Line& l0, const Line& l1);

  Line line1, line2;
  Point intersection;
};
