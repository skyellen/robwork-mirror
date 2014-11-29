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
 
 

#ifndef RWLIBS_ALGORITHMS_RANSACMODEL_HPP
#define RWLIBS_ALGORITHMS_RANSACMODEL_HPP



/**
 * @file RansacModel.hpp
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Math.hpp>

#include <iostream>
#include <vector>



namespace rwlibs { namespace algorithms {



/**
 * @brief An interface for RANSAC model fitting.
 * 
 * @todo A model needs to remember the indices of inliers from the set of data...
 */
template <class MODEL, class DATA>
class RANSACModel
{
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<RANSACModel<MODEL, DATA> > Ptr;
		
	public: // constructors
		//! @brief Constructor.
		RANSACModel() {}
		
		/**
		 * @brief Creates a new model of this type using provided data.
		 */
		virtual MODEL& make(const std::vector<DATA>& data) const {
			typename MODEL::Ptr newModel = new MODEL();
			
			newModel->refit(data);
			
			return *newModel;
		}
		
		//! @brief Destructor.
		virtual ~RANSACModel() {}
		
		/**
		 * @brief Find models fitting a set of observations.
		 */
		static std::vector<MODEL> findModels(const std::vector<DATA>& data, int maxIterations, int dataRequired, double dataThreshold, double modelThreshold) {
			//std::vector<DATA> samples(data); // copy data into the local vector <- argh
			
			// create a vector of indices used for shuffling and picking random points so they don't repeat
			std::vector<size_t> indices;
			for (size_t i = 0; i < data.size(); ++i) {
				indices.push_back(i);
			}
			std::vector<std::pair<MODEL, int> > models; // pair containing a model and a number of inliers
			
			int iterations = 0;
			while (iterations++ < maxIterations) {
				
				int n = MODEL().getMinReqData();
				
				std::random_shuffle(indices.begin(), indices.end());
				std::vector<DATA> maybeInliers;
				for (size_t i = 0; i < n; ++i) {
					maybeInliers.push_back(data[indices[i]]);
				}
				//maybeInliers.insert(maybeInliers.end(), samples.begin(), samples.begin()+n);

				// create a model based on the maybeInliers
				MODEL maybeModel;
				try { 
					maybeModel = maybeModel.make(maybeInliers);

					if (maybeModel.invalid()) {
						continue;
					}
				} catch(...) {
					continue;
				}
				
				// add the maybe inliers to the conensus set
				std::vector<DATA> consensusSet(maybeInliers);

				// check if any point in data fits the model with an error smaller than t
				for (size_t i = 0; i < data.size(); i++) {
					if (maybeModel.fitError(data[i]) < dataThreshold) {
						consensusSet.push_back(data[i]);
					}
				}

				// if consensus set size is large enough, we have a model
				if (consensusSet.size() > dataRequired) {
					//std::cout << "We have a model!" << std::endl;
					models.push_back(std::pair<MODEL, int>(maybeModel, consensusSet.size()));
				}

			}
			
			// merging models
			//std::cout << "Merging models" << std::endl;
			if (models.size() == 0) {
				return std::vector<MODEL>();
			}
            if (models.size() == 1) {
                return std::vector<MODEL>(1, models[0].first);
			}
			
			// merge models that are closely related
			std::vector<std::pair<MODEL*, int> > modelsPtr(models.size());
			
			for (size_t i = 0; i < models.size(); i++) {
				modelsPtr[i].first = &(models[i].first);
				modelsPtr[i].second = models[i].second;
			}

			std::vector<MODEL> newModels;
			// for all found models...
			for (size_t i = 0; i < modelsPtr.size() - 1; i++) {

				if (modelsPtr[i].first == NULL) {
					continue;
				}
				
				// compare with all following models...
				std::pair<MODEL*, int> bestCloseModel(modelsPtr[i].first, modelsPtr[i].second);
				for (size_t j = i + 1; j < modelsPtr.size(); j++) {
					
					//std::cout << "Comparing model " << i << " with model " << j;
					
					if (modelsPtr[j].first == NULL) {
						// the model was looked at already
						//std::cout << " 1st == null" << std::endl;
						continue;
					}

					// (disregard, if those models are different)
					bool res = models[i].first.same(models[j].first, modelThreshold);
					if (!res) {
						//std::cout << " m1 != m2" << std::endl;
						continue;
					}

					// ...to see which model has more inliers
					// merge those models
					if (bestCloseModel.second < modelsPtr[j].second){
					    bestCloseModel = modelsPtr[j];
					}
					
					// mark the model as processed
					modelsPtr[j].first = NULL;
					//std::cout << " merged" << std::endl;
				}
				
				if (bestCloseModel.first == NULL) {
					//std::cout << " best == null" << std::endl;
					continue;
				}

				// re-fit data to the best close model found
				std::vector<DATA> consensusSet;
				std::vector<size_t> consensusSetIndices;
                for (size_t k = 0; k < data.size(); ++k) {
					if (bestCloseModel.first->fitError(data[k]) < dataThreshold) {
                        consensusSet.push_back(data[k]);
                        consensusSetIndices.push_back(k);
                    }
                }
                
				try {
				    bestCloseModel.first->refit(consensusSet);
				    bestCloseModel.first->_indices = consensusSetIndices;
				    
				    if (bestCloseModel.first->invalid()) {
						continue;
					}
				} catch (...) {
					//std::cout << " crash" << std::endl;
					continue;
				}
				
				//std::cout << "BestModel: " << consensusSet.size() << std::endl; 
				newModels.push_back(*bestCloseModel.first);
				//std::cout << " saved" << std::endl;
			}

			//std::cout << "Nr of models found: " << models.size() << std::endl;
			//std::cout << "filtered to       : " << newModels.size() << std::endl;

			return newModels;
		}
		
		/**
		 * @brief Select the model with the biggest number of inliers.
		 */
		static MODEL bestModel(const std::vector<MODEL>& models)	{
			if (models.size() == 0) {
				return *(new MODEL());
			}
			
			size_t inliers = 0;
			size_t idx = 0;
			
			for (size_t i = 0; i < models.size(); ++i) {
				size_t curInliers = models[i].getNumberOfInliers();
				
				if (curInliers > inliers) {
					inliers = curInliers;
					idx = i;
				}
			}
			
			return models[idx];
		}
		
		/**
		 * @brief Select a model randomly, with a chance based on the number of inliers.
		 */
		static MODEL likelyModel(const std::vector<MODEL>& models) {
			if (models.size() == 0) {
				return *(new MODEL());
			}
			
			size_t total_inliers = 0;
			std::vector<size_t> thresholds;
			
			for (size_t i = 0; i < models.size(); ++i) {
				size_t inliers = models[i].getNumberOfInliers();
				
				total_inliers += inliers;
				thresholds.push_back(total_inliers);
			}
			
			size_t pick = rw::math::Math::ran(0.0, 1.0) * total_inliers;
			
			size_t idx = 0; // model index
			for (size_t i = 0; i < models.size(); ++i) {
				if (pick < thresholds[i]) {
					idx = i;
					break;
				}
			}
			
			return models[idx];
		}

	public: // methods		
		/**
		 * @brief Calculates the fitting error of a sample.
		 */
		virtual double fitError(const DATA& sample) const = 0;
		
		/**
		 * @brief Check whether a sample belongs to the model.
		 * 
		 * Returns \b true when the sample is within a threshold distance of the model.
		 */
		virtual bool belongsTo(const DATA& sample, double threshold) const { return fitError(sample) <= threshold; }
		
		/**
		 * @brief Checks whether the model is invalid.
		 */
		virtual bool invalid() const = 0;
		
		/**
		 * @brief Recalculates the model based on provided samples.
		 * 
		 * @return Fit error on a set of provided samples.
		 */
		virtual double refit(const std::vector<DATA>& data) = 0;
		
		/**
		 * @brief Returns the number of samples required to create the model.
		 */
		virtual int getMinReqData() const = 0;
		
		/**
		 * @brief Tests whether the model is same to a threshold of another model.
		 */
		virtual bool same(const MODEL& model, double threshold) const = 0;
		
		/**
		 * @brief Get the number of inliers.
		 */
		size_t getNumberOfInliers() const { return _data.size(); }
		
		//! @brief Get the model quality.
		double getQuality() const { return _quality; }
		
		/**
		 * @brief Set the model quality.
		 *
		 * @param quality [in] fitting error of the model
		 */
		void setQuality(double quality) { _quality = quality; }
		
		//! @brief Access data.
		std::vector<DATA>& getData() { return _data; }
		
		//! @brief Access data.
		const std::vector<DATA>& getData() const { return _data; }
		
		/**
		 * @brief Get the vector of inlier indices
		 * 
		 * @return a vector of indices of inliers in the data vector supplied to findModels function.
		 */
		std::vector<size_t>& getInlierIndices() { return _indices; }
	
	protected: // body
		std::vector<DATA> _data;
		double _quality;
		
		/* <hack>
		 * For some calculations, it is neccesary to know which data points given to findModels()
		 * function were actualy fitted in the model. This vector holds the indices of inliers.
		 * Use at your own risk, if you know what you are doing.
		 * </hack>
		 */
		std::vector<size_t> _indices;
};



}} // /namespaces

#endif // include guard
