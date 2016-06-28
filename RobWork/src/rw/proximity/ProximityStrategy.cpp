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


#include "CollisionStrategy.hpp"

#include <vector>

#include <rw/kinematics/Frame.hpp>
//#include <rw/common/macros.hpp>
//#include <rw/common/Exception.hpp>
#include <rw/models/Object.hpp>
#include <rw/models/RigidObject.hpp>
#include <boost/foreach.hpp>

using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::common;
using namespace rw::models;

ProximityStrategy::ProximityStrategy():
    _frameToModel(NULL,100)
{}

ProximityStrategy::~ProximityStrategy()
{}

ProximityModel::Ptr ProximityStrategy::getModel(const rw::kinematics::Frame* frame)
{

    ProximityModel::Ptr model =  _frameToModel[*frame];
    //if(model==NULL){
    //    if( hasModel(frame) ){
    //        if( addModel(frame) )
    //            model = _frameToModel[*frame];
    //    }
    //}
    return model;
}

bool ProximityStrategy::addModel(rw::models::Object::Ptr object)
{
    if(RigidObject::Ptr robj = object.cast<RigidObject>()){
		std::vector<Geometry::Ptr> geoms = robj->getGeometry();
		BOOST_FOREACH(Geometry::Ptr geom, geoms){
			Frame* geoframe = geom->getFrame();
			if(!hasModel(geoframe))
				_frameToModel[*geoframe] = createModel();
			ProximityModel::Ptr model = getModel(geoframe);
			addGeometry(model.get(), geom);
		}
		return true;
    }
    return false;
}

/*
bool ProximityStrategy::addModel(const Frame* frame)
{
    std::vector<CollisionModelInfo> modelInfos = CollisionModelInfo::get(frame);
    if( modelInfos.size()==0 ){
        return false;
    }

	ProximityModel::Ptr model = _frameToModel[*frame];
    if( model==NULL && hasModel(frame)){
        model = createModel();
        _frameToModel[*frame] = model;
    } else {
        return false;
    }

    BOOST_FOREACH(CollisionModelInfo &info, modelInfos) {
        try {
			Geometry::Ptr geom = GeometryFactory::getGeometry(info.getGeoString());
            if(geom==NULL)
                continue;

            geom->setTransform( info.getTransform() );
            geom->setScale( info.getGeoScale() );
            std::cout << model->getGeometryIDs().size() << std::endl;
            addGeometry(model.get(), *geom);
        } catch (const rw::common::Exception& exp) {
            RW_WARN("Unable to load geometry "<<info.getGeoString()<<" with message "<<exp);
        }
    }
    return true;
}
*/

bool ProximityStrategy::addModel(const Frame* frame, const rw::geometry::Geometry& geom)
{
	ProximityModel::Ptr model = getModel(frame);
    if(model==NULL){
        model = createModel();
    }

    bool res = addGeometry(model.get(), geom);
    if(res){
        _frameToModel[*frame] = model;
    }
    return res;
}

bool ProximityStrategy::addModel(const Frame* frame, rw::geometry::Geometry::Ptr geom, bool forceCopy)
{
    ProximityModel::Ptr model = getModel(frame);
    if(model==NULL){
        model = createModel();
    }

    bool res = addGeometry(model.get(), geom, forceCopy);
    if(res){
        _frameToModel[*frame] = model;
    }
    return res;
}

bool ProximityStrategy::hasModel(const rw::kinematics::Frame* frame){
    if( !_frameToModel.has( *frame ) || _frameToModel[*frame]==NULL){
        //if (CollisionModelInfo::get(frame).size()>0)
        //    return true;
        return false;
    }
    return true;
}

void ProximityStrategy::clearFrame(const rw::kinematics::Frame* frame){
    if( !_frameToModel.has( *frame ) || _frameToModel[*frame]==NULL )
        return;
	ProximityModel::Ptr model = _frameToModel[*frame];
    std::cout << "clear frame" << std::endl;
    if( model == NULL )
    	return;
    _frameToModel[*frame] = NULL;
    destroyModel(model.get());
}

void ProximityStrategy::clearFrames(){
    _frameToModel.clear();
}
