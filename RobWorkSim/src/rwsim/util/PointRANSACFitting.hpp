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
#include "PlaneModel.hpp"
#include <rw/math/Vector3D.hpp>
#include <rw/math/Random.hpp>

//#include <boost/foreach.hpp>

namespace rwsim {
namespace util {


	class PointRANSACFitting {
	public:

		/**
		 * @brief fits a number of models to the set of data points
		 * @param k [in] the maximum number of iterations allowed in the algorithm
		 * @param d [in] the number of close data values required to assert that a model fits well to data
		 * @param t [in] a threshold value for determining when a datum fits a model
		 */
		static std::vector<PlaneModel>
			fit(std::vector<rw::math::Vector3D<> >& data, int k, int d, double t );



		template <class MODEL_T>
		static std::vector<MODEL_T>
		fit2(std::vector<rw::math::Vector3D<> >& data, int k, int d, double t ){
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
			std::vector<MODEL_T> models;
			std::vector<Vector3D<> > bestConsensusSet;
			std::vector<Vector3D<> > maybeInliers(n);
			//double bestError = 100000.0;
			while( ++iterations < k ){
				std::vector<Vector3D<> > consensusSet;

				// generate n randomly selected values from data
				for(int i=0; i<n; i++){
					int idx = Random::ranI(0,data.size());
					maybeInliers[i] = data[idx];
				}

				// create a model based on the maybeInliers
				MODEL_T maybeModel( maybeInliers );
				if( maybeModel.invalid() )
					continue;

				// add the maybe inliers to the conensus set
				for(int i=0; i<n; i++){
					consensusSet.push_back( maybeInliers[i] );
				}

				// check if any point in data fits the model with an error smaller than t
				for( int i=0; i<data.size(); i++){
					if( maybeModel.fitError( data[i] ) < t ){
						consensusSet.push_back( data[i] );
					}
				}

				if( consensusSet.size()>d ){
					//std::cout << "- Maybe model: "<< consensusSet.size() << std::endl;
					//maybeModel.print();
					//maybeModel.print();
					//double error = maybeModel.refit( consensusSet );
					maybeModel.refit( consensusSet );

					models.push_back( maybeModel );

					/*if( error< bestError ){
						bestModel = maybeModel;
						bestConsensusSet = consensusSet;
						bestError = error;
					}*/
				}

			}
			//std::cout << "Merging Models: "<< models.size() << std::endl;
			if(models.size()<=1)
				return models;
			// merge models that are closely related
			std::vector<MODEL_T*> modelsPtr(models.size());
			for(int i=0;i<models.size();i++){
				modelsPtr[i] = &models[i];
			}

			std::vector<MODEL_T> newModels;
			for(int i=0;i<modelsPtr.size()-1;i++){

				if( modelsPtr[i]==NULL )
					continue;
				std::vector<MODEL_T*> closeModels;
				for(int j=i+1;j<modelsPtr.size();j++){
					//std::cout << "J: " << j << std::endl;
					if( modelsPtr[j]==NULL )
						continue;

					bool res = models[i].same( models[j], 0.01 );
					if(!res)
						continue;
					modelsPtr[j] = NULL;
					closeModels.push_back( &models[j] );
				}
				// TODO: merge all close models into one model
				if(closeModels.size()>0)
					newModels.push_back(*closeModels[0]);
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
