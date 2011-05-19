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
#define RW_DEBUG_ENABLE
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


    //Model(std::string id, rw::math::Transform3D<> trans, rw::proximity::BinaryBVTree<rw::geometry::OBB<> >::Ptr obbtree):
    //            geoid(id),t3d(trans),tree(obbtree){}

    RWProximityModel *pmodel = (RWProximityModel*) model;


    // check if geomid is in model. remove it if it has
    BOOST_FOREACH(Model::Ptr &m, pmodel->models){
        if( m->geoid==geom.getId() ){
            removeGeometry( model, geom.getId() );
            break;
        }
    }

    Model::Ptr rwmodel;
    // check if model is in
    CacheKey key(geom.getId(),geom.getScale());
    if( _modelCache.has(key) ){
        rwmodel = _modelCache.get(key);
    } else {
        GeometryData::Ptr gdata = geom.getGeometryData();
        TriMesh::Ptr mesh = gdata->getTriMesh(false);
        if(mesh->getSize()==0)
            return false;

        // TODO: we don't use scale for now
        const double scale = geom.getScale();

        BVTreeFactory treefactory;
        BinaryBVTree<OBB<> >::Ptr tree = treefactory.makeTopDownOBBTreeCovarMedian(mesh,1);

        rwmodel = ownedPtr( new Model(geom.getId(), geom.getTransform(), tree) );
        rwmodel->ckey = key;
        rwmodel->scale = scale;
        _modelCache.add(key, rwmodel);
    }

    pmodel->models.push_back( rwmodel );
    //_allmodels.push_back(pmodel->models.back());
    //_geoIdToModelIdx[geom.getId()].push_back(_allmodels.size()-1);
    return true;
}

bool ProximityStrategyRW::removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId){
    RWProximityModel *pmodel = (RWProximityModel*) model;
    size_t idx=-1;
    for(size_t i=0;i<pmodel->models.size();i++)
        if(pmodel->models[i]->geoid==geomId){
            idx = i;
            break;
        }
    // if the geometry id was not found then it is not located in the model
    if(idx<0){
        //RW_THROW("No geometry with id: \""<< geomId << "\" exist in proximity model!");
        return false;
    }

    // it was found so we remove it from cache
    _modelCache.remove(pmodel->models[idx]->ckey);
    std::vector<Model::Ptr>::iterator iter = pmodel->models.begin();
    for(;iter!=pmodel->models.end();++iter){
        if((*iter)->geoid==geomId){
            pmodel->models.erase(iter);
            return true;
        }
    }
    return false;
}

ProximityStrategyRW::QueryData ProximityStrategyRW::initQuery(ProximityModel::Ptr& aModel, ProximityModel::Ptr& bModel, ProximityStrategyData &data){
    QueryData qdata;

    if(data.getCache()==NULL || data.getCache()->_owner!=this)
        data.getCache() = ownedPtr( new PCache(this));

    qdata.cache = static_cast<PCache*>(data.getCache().get());
    if(qdata.cache->tcollider==NULL)
        qdata.cache->tcollider = ownedPtr( BVTreeColliderFactory::makeBalancedDFSColliderOBB<BinaryBVTree<OBB<> > >() );

    qdata.a = (RWProximityModel*)aModel.get();
    qdata.b = (RWProximityModel*)bModel.get();
    return qdata;
}


std::vector<std::string> ProximityStrategyRW::getGeometryIDs(rw::proximity::ProximityModel* model){
    return model->getGeometryIDs();
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

    BOOST_FOREACH(Model::Ptr &ma, qdata.a->models) {
        BOOST_FOREACH(Model::Ptr &mb, qdata.b->models) {

            bool res = qdata.cache->tcollider->collides(wTa, *ma->tree, wTb, *mb->tree);

            //std::cout << res << std::endl;
            //_numBVTests += qdata.cache->_collideResult.NumBVTests();
            //_numTriTests += qdata.cache->_collideResult.NumTriTests();

            if(res==true){
                data.a = aModel;
                data.b = bModel;
                data._aTb = inverse(wTa)*wTb;
                nrOfCollidingGeoms++;
                //data._collisionPairs.resize(nrOfCollidingGeoms);
                //data._collisionPairs[nrOfCollidingGeoms-1].geoIdxA = geoIdxA;
                //data._collisionPairs[nrOfCollidingGeoms-1].geoIdxB = geoIdxB;
                //int startIdx = data._geomPrimIds.size();
                //int size = qdata.cache->_collideResult.num_pairs;
                //data._collisionPairs[nrOfCollidingGeoms-1].startIdx = startIdx;
                //data._collisionPairs[nrOfCollidingGeoms-1].size = size;
                //data._geomPrimIds.resize(startIdx+size);

                //for(int j=0;j<size;j++){
                //    data._geomPrimIds[startIdx+j].first = qdata.cache->_collideResult.pairs[j].id1;
                //    data._geomPrimIds[startIdx+j].second= qdata.cache->_collideResult.pairs[j].id2;
                //}


                if(firstContact)
                    return true;
                col_res = true;
            }
            geoIdxB++;
        }
        geoIdxA++;
    }
    return col_res;
}

void ProximityStrategyRW::clear()
{
    _modelCache.clear();
    _allModels.clear();
    clearFrames();
}
