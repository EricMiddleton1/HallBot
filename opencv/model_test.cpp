#include <iostream>

#include "GRANSAC.hpp"
#include "IntersectModel.hpp"

struct PointLine {
  Point p0, p1;
};

int main() {
  std::vector<PointLine> pLines = {
    { {0.f, 0.f}, {1.f, 1.f} },
    { {1.f, 100.5f}, {1.f, 2.f} },
    { {1.f, 1.5f}, {2.f, 1.f} },
    { {1000.f, 0.f}, {0.f, 1000.f} }
  };

  GRANSAC::RANSAC<IntersectModel,2> estimator;
  estimator.Initialize(20, 100);

  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> lines;

  for(const auto& pLine : pLines) {
    float a = pLine.p1.y - pLine.p0.y;
    float b = pLine.p0.x - pLine.p1.x;
    float c = b*pLine.p0.y + a*pLine.p0.x;

    lines.push_back(std::make_shared<Line>(a, b, c));
  }

  estimator.Estimate(lines);
  auto bestModel = estimator.GetBestModel();
  if(bestModel) {
    auto intersectionPoint = bestModel->getIntersectionPoint();

    std::cout << "[Info] Intersection Point: ("
      << intersectionPoint.x << ", " << intersectionPoint.y << ")" << std::endl;
  }
  else {
    std::cout << "[Error] Couldn't find best model" << std::endl;
  }

  std::cout << "[Info] Done" << std::endl;

  return 0;
}
