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

#include "PointRANSACFitting.hpp"

using namespace rwsim::util;
using namespace rw::math;

/*

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
std::vector<PlaneModel>
	PointRANSACFitting::fit( std::vector<Vector3D<> >& data, int k, int d, double t) {
	//std::cout << "Fitting data! " << data.size() << std::endl;
    int n = PlaneModel::getMinReqData(); // the minimum number of data required to fit the model
    //int k = 100; // the maximum number of iterations allowed in the algorithm
    //double t = 0.01;  // a threshold value for determining when a datum fits a model
    //int d = 20; // the number of close data values required to assert that a model fits well to data

    /*std::cout << "- n: " << n << std::endl
			  << "- k: " << k << std::endl
			  << "- t: " << t << std::endl
			  << "- d: " << d << std::endl
				<< "- size: " << data.size() << std::endl;
*/
    int iterations = 0;
    PlaneModel bestModel;
    std::vector<PlaneModel> models;
    std::vector<Vector3D<> > bestConsensusSet;
    std::vector<Vector3D<> > maybeInliers(n);
    //double bestError = 100000.0;
    while( ++iterations < k ){
        std::vector<Vector3D<> > consensusSet;

        // generate n randomly selected values from data
        for(int i=0; i<n; i++){
            int idx = Random::ranI(0,(int)data.size());
            maybeInliers[i] = data[idx];
        }

        // create a model based on the maybeInliers
        PlaneModel maybeModel( maybeInliers );
        if( maybeModel.invalid() )
        	continue;

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

        if( consensusSet.size()>(size_t)d ){
        	//std::cout << "- Maybe model: "<< consensusSet.size() << std::endl;
        	//maybeModel.print();
        	//maybeModel.print();
            /*double error =*/ maybeModel.refit( consensusSet );

            models.push_back( maybeModel );

            //if( error< bestError ){
            //    bestModel = maybeModel;
            //    bestConsensusSet = consensusSet;
            //    bestError = error;
            //}
        }
    }
    //std::cout << "Merging Models: " << std::endl;
    // merge models that are closely related
    std::vector<PlaneModel*> modelsPtr(models.size());
    for(size_t i=0;i<models.size();i++){
    	modelsPtr[i] = &models[i];
    }
    std::vector<PlaneModel> newModels;
    for(size_t i=0;i<modelsPtr.size()-1;i++){
		if( modelsPtr[i]==NULL )
			continue;
    	std::vector<PlaneModel*> closeModels;
    	for(size_t j=i+1;j<modelsPtr.size();j++){
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

/*
    std::cout << "Nr of models found: " << models.size() << std::endl;
    std::cout << "filtered to       : " << newModels.size() << std::endl;
    PlaneModel defPlane;
	BOOST_FOREACH(Vector3D<> &p, data){
		std::cout << " " << defPlane.fitError(p) << std::endl;
	}
	*/
	/*std::cout << "Models " << std::endl;
	BOOST_FOREACH(PlaneModel &m, newModels){
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
