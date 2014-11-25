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

#ifndef RWSIM_UTIL_POINTRANSACFITTING_HPP_
#define RWSIM_UTIL_POINTRANSACFITTING_HPP_

#include <vector>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Line2DPolar.hpp>

#include <boost/foreach.hpp>

namespace rw {
namespace geometry {

	class RANSACFitting {
	public:

        /**
         * @brief fits a number of models to the set of data points
         * @param k [in] the maximum number of iterations allowed in the algorithm
         * @param d [in] the number of close data values required to assert that a model fits well to data
         * @param t [in] a threshold value for determining when a datum fits a model
         */
/*
	    static std::vector<rw::math::Line2DPolar>
            fit(std::vector<rw::math::Vector2D<> >& data, int k, int d, double t, double s )
        {

            return fit2<rw::math::Line2DPolar, rw::math::Vector2D<> >( data, k ,d ,t, s);
        }
*/
        /**
         * @brief
        input:
            data - a set of observations
            model - a model that can be fitted to data
            n - the minimum number of data required to fit the model
            k - the maximum number of iterations allowed in the algorithm
            t - a threshold value for determining when a datum fits a model
            d - the number of close data values required to assert that a model fits well to data

        output:
            best_model - model parameters which best fit the data (or nil if no good model is found)
            best_consensus_set - data point from which this model has been estimated
            best_error - the error of this model relative to the data
        */
		template <class MODEL_T, class DATA>
		static std::vector<MODEL_T> fit2(std::vector<DATA>& data, int k, int d, double t, double s){
			using namespace rw::math;
			//std::cout << "Fitting data! " << data.size() << std::endl;
			int n = MODEL_T::getMinReqData(); // the minimum number of data required to fit the model
	/*
			std::cout << "- n: " << n << std::endl
					  << "- k: " << k << std::endl
					  << "- t: " << t << std::endl
					  << "- d: " << d << std::endl
						<< "- size: " << data.size() << std::endl;
	*/
			int iterations = 0;
			MODEL_T bestModel;
			std::vector<std::pair<MODEL_T, int> > models;
			std::vector<DATA> bestConsensusSet;
			std::vector<DATA> maybeInliers(n);
			//double bestError = 100000.0;
			while( ++iterations < k ){
				std::vector<DATA> consensusSet;

				// generate n randomly selected values from data

				//for(int i=0; i<n; i++){
				int idx = Math::ranI(0,data.size());
				maybeInliers[0] = data[idx];
				while(true){
					int idxSecond = Math::ranI(0,data.size());
					if(idx != idxSecond) { // && (data[idxSecond]-data[idx]).norm2() > 10E-5 ){
						maybeInliers[1] = data[idxSecond];
						break;
					}
				}
				//}

				MODEL_T maybeModel;
				// create a model based on the maybeInliers
				try {
					maybeModel = MODEL_T::make( maybeInliers );

					if( maybeModel.invalid() )
						continue;
				} catch(...) {
					continue;
				}
				// add the maybe inliers to the conensus set
				for(int i=0; i<n; i++){
					consensusSet.push_back( maybeInliers[i] );
				}

				// check if any point in data fits the model with an error smaller than t
				for( size_t i=0; i<data.size(); i++){
					if( maybeModel.fitError( data[i] ) < t ){
						consensusSet.push_back( data[i] );
					}
				}

				if( (int)consensusSet.size()>d ){
					//std::cout << "- Maybe model: "<< consensusSet.size() << std::endl;
					//maybeModel.print();
					//maybeModel.print();

				    //double error = maybeModel.refit( consensusSet );

					models.push_back( std::pair<MODEL_T, int>(maybeModel, consensusSet.size()) );

					/*if( error< bestError ){
						bestModel = maybeModel;
						bestConsensusSet = consensusSet;
						bestError = error;
					}*/
				}

			}
			//std::cout << "Merging Models: "<< models.size() << std::endl;
			if(models.size()<0)
				return std::vector<MODEL_T>();
            if(models.size()==1)
                return std::vector<MODEL_T>(1,models[0].first);

			// merge models that are closely related
			std::vector<std::pair<MODEL_T*,int> > modelsPtr(models.size());
			for(size_t i=0;i<models.size();i++){
				modelsPtr[i].first = &(models[i].first);
				modelsPtr[i].second = models[i].second;
			}

			std::vector<MODEL_T> newModels;
			for(size_t i=0;i<modelsPtr.size()-1;i++){

				if( modelsPtr[i].first==NULL )
					continue;
				//std::vector<std::pair<MODEL_T*, int> > closeModels;
				std::pair<MODEL_T*, int> bestCloseModel((MODEL_T*)NULL, 0);
				for(size_t j=i+1;j<modelsPtr.size();j++){
					//std::cout << "J: " << j << std::endl;
					if( modelsPtr[j].first==NULL )
						continue;

					bool res = models[i].first.same( models[j].first, s );
					if(!res)
						continue;

					if( bestCloseModel.second<modelsPtr[j].second ){
					    bestCloseModel = modelsPtr[j];
					}
					modelsPtr[j].first = NULL;
				}
				if(bestCloseModel.first==NULL)
					continue;

				std::vector<DATA> consensusSet;
                for( size_t i=0; i<data.size(); i++){
					if( bestCloseModel.first->fitError( data[i] ) < t ){
                        consensusSet.push_back( data[i] );
                    }
                }
                double error;
				try {
				    error = bestCloseModel.first->refit( consensusSet );
				} catch (...){
					continue;
				}
				std::cout << "BestModel: " << consensusSet.size() << std::endl; 
				bestCloseModel.first->setQuality( consensusSet.size(),  error);
				newModels.push_back(*bestCloseModel.first);
			}

			std::cout << "Nr of models found: " << models.size() << std::endl;
			std::cout << "filtered to       : " << newModels.size() << std::endl;
		/*
			PlaneModel defPlane;
			BOOST_FOREACH(Vector3D<> &p, data){
				std::cout << " " << defPlane.fitError(p) << std::endl;
			}
			*/
			/*BOOST_FOREACH(MODEL_T &m, newModels){
				m.print();
			}*/
			/*
			std::cout << "Best Model \n\r-";
			bestModel.print();
			BOOST_FOREACH(Vector3D<> &p, bestConsensusSet){
				//std::cout << " " << bestModel.fitError(p) << std::endl;
			}
			*/
			return newModels;

		}


	};
}
}

#endif /* PLANEFITTING_HPP_ */
