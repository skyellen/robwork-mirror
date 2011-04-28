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


#include "ProximityStrategyRW.hpp"

#include <float.h>
#include <vector>

#include <rw/geometry/TriMesh.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Exception.hpp>
#include <rw/models/Accessor.hpp>



#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

using namespace rwlibs::proximitystrategies;

//----------------------------------------------------------------------
// ProximityStrategyRW

ProximityStrategyRW::ProximityStrategyRW()

{
	clearStats();
}

rw::proximity::ProximityModel::Ptr ProximityStrategyRW::createModel()
{
    RWProximityModel *model = new RWProximityModel(this);
    return ownedPtr(model);
}

void ProximityStrategyRW::destroyModel(rw::proximity::ProximityModel* model){
	// when model gets deleted it should cleanup itself
	// TODO: though the models should probably be removed from cache
}

bool ProximityStrategyRW::addGeometry(ProximityModel* model, const Geometry& geom) {


    return true;
}

bool ProximityStrategyRW::removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId){
	return false;
}

ProximityStrategyRW::QueryData ProximityStrategyRW::initQuery(ProximityModel::Ptr& aModel, ProximityModel::Ptr& bModel, ProximityStrategyData &data){
    QueryData qdata;
    return qdata;
}



bool ProximityStrategyRW::inCollision(ProximityModel::Ptr aModel,
	const Transform3D<>& wTa,
	ProximityModel::Ptr bModel,
	const Transform3D<>& wTb,
	ProximityStrategyData &pdata)
{
    QueryData qdata = initQuery(aModel,bModel,pdata);

    CollisionResult &data = pdata.getCollisionData();
    data.clear();

    size_t nrOfCollidingGeoms = 0, geoIdxA=0, geoIdxB=0;
    bool col_res = false;
    bool firstContact = pdata.getCollisionQueryType() == FirstContact;

    BOOST_FOREACH(Model::Ptr ma, qdata.a->models) {
        BOOST_FOREACH(Model::Ptr mb, qdata.b->models) {

/*
            _numBVTests += qdata.cache->_collideResult.NumBVTests();
            _numTriTests += qdata.cache->_collideResult.NumTriTests();
            if (qdata.cache->_collideResult.Colliding() != 0){
            	data.a = aModel;
            	data.b = bModel;
            	data._aTb = fromRapidTransform(qdata.cache->_collideResult.R,qdata.cache->_collideResult.T);

            	nrOfCollidingGeoms++;

            	// copy data to collision data res
           		data._collisionPairs.resize(nrOfCollidingGeoms);

            	data._collisionPairs[nrOfCollidingGeoms-1].geoIdxA = geoIdxA;
            	data._collisionPairs[nrOfCollidingGeoms-1].geoIdxB = geoIdxB;

            	int startIdx = data._geomPrimIds.size();
            	int size = qdata.cache->_collideResult.num_pairs;
            	data._collisionPairs[nrOfCollidingGeoms-1].startIdx = startIdx;
            	data._collisionPairs[nrOfCollidingGeoms-1].size = size;

            	data._geomPrimIds.resize(startIdx+size);

            	for(int j=0;j<size;j++){
            		data._geomPrimIds[startIdx+j].first = qdata.cache->_collideResult.pairs[j].id1;
            		data._geomPrimIds[startIdx+j].second= qdata.cache->_collideResult.pairs[j].id2;
            	}
                if(firstContact)
                    return true;

            	col_res = true;
            }

            */
            geoIdxB++;
        }
        geoIdxA++;
    }
    return col_res;
}

void ProximityStrategyRW::clear()
{
    _modelCache.clear();
    _geoIdToModelIdx.clear();
    _allmodels.clear();

    clearFrames();
}
