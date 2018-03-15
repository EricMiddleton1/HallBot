#pragma once

#include <iostream>
#include <cmath>
#include <string>
#include <random>
#include <algorithm>
#include <vector>
#include <omp.h>

namespace GRANSAC
{
  using VPFloat = double;

	// T - Model
	template <class T>
	class RANSAC
	{
	private:
		std::vector<T> m_SampledModels; // Vector of all sampled models
		std::vector<typename T::Param> m_params; //vector of all params (from call to Estimate())
		T* m_BestModel; // Pointer to the best model, valid only after Estimate() is called
		std::vector<typename T::Param*> m_BestInliers;

		int m_MaxIterations; // Number of iterations before termination
		VPFloat m_Threshold; // The threshold for computing model consensus
		VPFloat m_BestModelScore; // The score of the best model

		std::vector<std::mt19937> m_RandEngines; // Mersenne twister high quality RNG that support *OpenMP* multi-threading

	public:
		RANSAC(void)
			:	m_BestModel{nullptr}
		{
			int nThreads = std::max(1, omp_get_max_threads());
			//std::cout << "[ INFO ]: Maximum usable threads: " << nThreads << std::endl;
			for (int i = 0; i < nThreads; ++i)
			{
				std::random_device SeedDevice;
				m_RandEngines.push_back(std::mt19937(SeedDevice()));
			}

			Reset();
		};

		~RANSAC(void) {};

		void Reset(void)
		{
			// Clear sampled models, etc. and prepare for next call. Reset RANSAC estimator state
			m_SampledModels.clear();

			m_BestModelScore = 0.0;
		};

		void Initialize(VPFloat Threshold, int MaxIterations = 1000)
		{
			m_Threshold = Threshold;
			m_MaxIterations = MaxIterations;
		};

		T* GetBestModel(void) { return m_BestModel; };
		const std::vector<typename T::Param*>& GetBestInliers(void) { return m_BestInliers; };

		bool Estimate(const std::vector<typename T::Param>& Data)
		{
			if (Data.size() <= (T::ParamCount))
			{
				std::cerr << "[ WARN ]: RANSAC - Number of data points is too less. Not doing anything." << std::endl;
				return false;
			}

			m_params = Data;

			int DataSize = m_params.size();
			std::uniform_int_distribution<int> UniDist(0, int(DataSize - 1)); // Both inclusive

			std::vector<VPFloat> InlierFractionAccum(m_MaxIterations);
			std::vector<std::vector<typename T::Param*>> InliersAccum(m_MaxIterations);
			m_SampledModels.resize(m_MaxIterations);

			//Precompute vector of param pointers
			std::vector<typename T::Param*> ParamPointers(DataSize);
			for(int i = 0; i < DataSize; ++i) {
				ParamPointers[i] = &m_params[i];
			}

			int nThreads = std::max(1, omp_get_max_threads());
			omp_set_dynamic(0); // Explicitly disable dynamic teams
			omp_set_num_threads(nThreads);
#pragma omp parallel for
			for (int i = 0; i < m_MaxIterations; ++i)
			{
				// Select T::ParamCount random samples
				std::array<typename T::Param*, T::ParamCount> RandomSamples;
				auto RemainderSamples = ParamPointers; // Without the chosen random samples

				std::shuffle(RemainderSamples.begin(), RemainderSamples.end(), m_RandEngines[omp_get_thread_num()]); // To avoid picking the same element more than once
				std::copy(RemainderSamples.begin(), RemainderSamples.begin() + T::ParamCount, RandomSamples.begin());
				RemainderSamples.erase(RemainderSamples.begin(), RemainderSamples.begin() + T::ParamCount);

				m_SampledModels[i] = T(RandomSamples);

				// Check if the sampled model is the best so far
				std::pair<VPFloat, std::vector<typename T::Param*>> EvalPair = m_SampledModels[i].Evaluate(RemainderSamples, m_Threshold);
				InlierFractionAccum[i] = EvalPair.first;
				InliersAccum[i] = EvalPair.second;
			}

			for (int i = 0; i < m_MaxIterations; ++i)
			{
				if (InlierFractionAccum[i] > m_BestModelScore)
				{
					m_BestModelScore = InlierFractionAccum[i];
					m_BestModel = &m_SampledModels[i];
					m_BestInliers = InliersAccum[i];
				}
			}

			Reset();

			return true;
		};
	};
} // namespace GRANSAC
