#pragma once

#include "GRANSAC.hpp"

typedef std::array<GRANSAC::VPFloat, 2> Vector2VP;

class Point2D
{
public:
    Point2D() {}
    Point2D(GRANSAC::VPFloat x, GRANSAC::VPFloat y)
    {
	    m_Point2D[0] = x;
	    m_Point2D[1] = y;
    };

    Vector2VP m_Point2D;
};


class Line2DModel
{
public:
  using Param = Point2D;
  static const int ParamCount = 2;
		
		Line2DModel() {}

    Line2DModel(const std::array<Param*, ParamCount> &InputParams)
    {
	Initialize(InputParams);
    };

    void Initialize(const std::array<Param*, ParamCount> &InputParams)
    {
      std::copy(InputParams.begin(), InputParams.end(), m_parameters.begin());

      auto& Point1 = *InputParams[0];
      auto& Point2 = *InputParams[1];
	// Compute the line parameters
	m_m = (Point2.m_Point2D[1] - Point1.m_Point2D[1]) / (Point2.m_Point2D[0] - Point1.m_Point2D[0]); // Slope
	m_d = Point1.m_Point2D[1] - m_m * Point1.m_Point2D[0]; // Intercept
	// m_d = Point2->m_Point2D[1] - m_m * Point2->m_Point2D[0]; // Intercept - alternative should be the same as above

	// mx - y + d = 0
	m_a = m_m;
	m_b = -1.0;
	m_c = m_d;

	m_DistDenominator = sqrt(m_a * m_a + m_b * m_b); // Cache square root for efficiency
    };

    std::pair<GRANSAC::VPFloat, std::vector<Param*>> Evaluate(const std::vector<Param*> &EvaluateParams, GRANSAC::VPFloat Threshold)
    {
	std::vector<Param*> Inliers;
	int nTotalParams = EvaluateParams.size();
	int nInliers = 0;

	for(auto& p : EvaluateParams)
	{
	    if(ComputeDistanceMeasure(p) < Threshold)
	    {
		Inliers.push_back(p);
		nInliers++;
	    }
	}

	GRANSAC::VPFloat InlierFraction = GRANSAC::VPFloat(nInliers) / GRANSAC::VPFloat(nTotalParams); // This is the inlier fraction

	return std::make_pair(InlierFraction, Inliers);
    };

		template<size_t i>
		Param* GetModelParams() {
			static_assert(i < ParamCount, "Line2DModel::GetModelParams<i>: i must be < ParamCount");

			return m_parameters[i];
		}

protected:
    // Model parameters
		std::array<Param*, ParamCount> m_parameters;
    
		// Parametric form
		GRANSAC::VPFloat m_a, m_b, m_c; // ax + by + c = 0
    GRANSAC::VPFloat m_DistDenominator; // = sqrt(a^2 + b^2). Stored for efficiency reasons

    // Another parametrization y = mx + d
    GRANSAC::VPFloat m_m; // Slope
    GRANSAC::VPFloat m_d; // Intercept

    virtual GRANSAC::VPFloat ComputeDistanceMeasure(Param* ExtPoint2D)
    {
	// Return distance between passed "point" and this line
	// http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
	GRANSAC::VPFloat Numer = fabs(m_a * ExtPoint2D->m_Point2D[0] + m_b * ExtPoint2D->m_Point2D[1] + m_c);
	GRANSAC::VPFloat Dist = Numer / m_DistDenominator;

	// // Debug
	// std::cout << "Point: " << ExtPoint2D->m_Point2D[0] << ", " << ExtPoint2D->m_Point2D[1] << std::endl;
	// std::cout << "Line: " << m_a << " x + " << m_b << " y + "  << m_c << std::endl;
	// std::cout << "Distance: " << Dist << std::endl << std::endl;

	return Dist;
    };
};

