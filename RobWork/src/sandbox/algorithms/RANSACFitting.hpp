/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_ALGORITHMS_RANSACFitting_HPP
#define RW_ALGORITHMS_RANSACFitting_HPP

#include <vector>
#include <algorithm>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Math.hpp>

#include <boost/foreach.hpp>

#include "RANSACModel.hpp"



namespace rwlibs {
namespace algorithms {



/**
 * @brief A class for doing RANSAC fitting
 */
class RANSACFitting {
	public:

        /**
         * @brief Find models fitting a set of observations.
         * 
         * @param data [in] a set of observations
         * @param maxIterations [in] the maximum number of iterations allowed in the algorithm
         * @param dataRequired [in] the number of close data values required to assert that a model fits well to data
         * @param dataThreshold [in] a threshold value for determining when a datum fits a model
         * @param modelThreshold [in] a threshold value for determining if two models are the same
         * 
         * @return a vector of models fitting the data
         * 
         * MODEL_T should provide following methods:
         * - int getMinReqData(), returning the number of data required to fit the model
         * - static MODEL_T make(data), creating a model from a set of data
         * - ...
        */
		template <class MODEL_T, class DATA>
		static std::vector<MODEL_T> fit(std::vector<DATA>& data, int maxIterations, int dataRequired, double dataThreshold, double modelThreshold)
		{
			using namespace rw::math;
			
			int n = MODEL_T::getMinReqData();
			int iterations = 0;
			
			MODEL_T bestModel;
			std::vector<std::pair<MODEL_T, int> > models; // pair containing a model and a number of inliers
			std::vector<DATA> bestConsensusSet;
			std::vector<DATA> maybeInliers;
			
			while (++iterations < maxIterations) {
				std::vector<DATA> consensusSet;

				// generate n randomly selected values from data
				// V- this is wrong...
				/*//for(int i=0; i<n; i++){
				int idx = Math::ranI(0,data.size());
				maybeInliers[0] = data[idx];
				while(true){
					int idxSecond = Math::ranI(0,data.size());
					if(idx != idxSecond) { // && (data[idxSecond]-data[idx]).norm2() > 10E-5 ){
						maybeInliers[1] = data[idxSecond];
						break;
					}
				}
				//}*/
				std::random_shuffle(data.begin(), data.end());
				maybeInliers = std::vector<DATA>(data.begin(), data.begin()+n);

				// create a model based on the maybeInliers
				MODEL_T maybeModel;
				try { 
					maybeModel = MODEL_T::make(maybeInliers);

					if (maybeModel.invalid()) {
						continue;
					}
				} catch(...) {
					continue;
				}
				
				// add the maybe inliers to the conensus set
				for (int i = 0; i < n; i++){
					consensusSet.push_back(maybeInliers[i]);
				}

				// check if any point in data fits the model with an error smaller than t
				for (size_t i = 0; i < data.size(); i++) {
					if (maybeModel.fitError(data[i]) < dataThreshold) {
						consensusSet.push_back(data[i]);
					}
				}

				if (consensusSet.size() > dataRequired) {
					models.push_back(std::pair<MODEL_T, int>(maybeModel, consensusSet.size()));
				}

			}
			
			// merging models
			if (models.size() < 0) {
				return std::vector<MODEL_T>();
			}
            if (models.size() == 1) {
                return std::vector<MODEL_T>(1, models[0].first);
			}
			
			// merge models that are closely related
			std::vector<std::pair<MODEL_T*, int> > modelsPtr(models.size());
			
			for (size_t i = 0; i < models.size(); i++) {
				modelsPtr[i].first = &(models[i].first);
				modelsPtr[i].second = models[i].second;
			}

			std::vector<MODEL_T> newModels;
			for (size_t i = 0; i < modelsPtr.size() - 1; i++) {

				if (modelsPtr[i].first == NULL) {
					continue;
				}
				
				std::pair<MODEL_T*, int> bestCloseModel((MODEL_T*)NULL, 0);
				for (size_t j = i + 1; j < modelsPtr.size(); j++) {
					
					if (modelsPtr[j].first == NULL) {
						continue;
					}

					bool res = models[i].first.same(models[j].first, modelThreshold);
					if (!res) {
						continue;
					}

					if (bestCloseModel.second<modelsPtr[j].second){
					    bestCloseModel = modelsPtr[j];
					}
					modelsPtr[j].first = NULL;
				}
				
				if (bestCloseModel.first == NULL) {
					continue;
				}

				std::vector<DATA> consensusSet;
                for (size_t i = 0; i < data.size(); i++) {
					if (bestCloseModel.first->fitError(data[i]) < dataThreshold) {
                        consensusSet.push_back(data[i]);
                    }
                }
                
                double error;
				try {
				    error = bestCloseModel.first->refit(consensusSet);
				} catch (...) {
					continue;
				}
				
				std::cout << "BestModel: " << consensusSet.size() << std::endl; 
				bestCloseModel.first->setQuality(consensusSet.size(),  error);
				newModels.push_back(*bestCloseModel.first);
			}

			std::cout << "Nr of models found: " << models.size() << std::endl;
			std::cout << "filtered to       : " << newModels.size() << std::endl;

			return newModels;
		}

	};
}
}

#endif /* RW_ALGORITHMS_RANSACFitting_HPP */
