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

#include <rw/geometry/Face.hpp>
#include <rw/geometry/FaceArrayFactory.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Exception.hpp>
#include <rw/models/Accessor.hpp>

#include <rw/geometry/GeometryFactory.hpp>

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

ProximityModelPtr ProximityStrategy::getModel(const rw::kinematics::Frame* frame)
{
    ProximityModelPtr model =  _frameToModel[*frame];
    if(model==NULL){
        if( hasModel(frame) ){
            if( addModel(frame) )
                model = _frameToModel[*frame];
        }
    }
    return model;
}

bool ProximityStrategy::addModel(const Frame* frame)
{
    if (!Accessor::collisionModelInfo().has(*frame)) {
        return false;
    }

    std::vector<CollisionModelInfo> modelInfos = Accessor::collisionModelInfo().get(*frame);
    if( modelInfos.size()==0 ){
        return false;
    }

    ProximityModelPtr model = _frameToModel[*frame];
    if( model==NULL && hasModel(frame)){
        model = createModel();
        _frameToModel[*frame] = model;
    } else {
        return false;
    }

    BOOST_FOREACH(CollisionModelInfo &info, modelInfos){
        GeometryPtr geom = GeometryFactory::getGeometry(info.getId());
        if(geom==NULL)
            continue;

        geom->setTransform( info.getTransform() );
        geom->setScale( info.getGeoScale() );

        addGeometry(model, *geom);
    }
    return true;
}

namespace {
    class GeometryFaceWrap: public rw::geometry::Geometry {
      public:

          GeometryFaceWrap(const std::string& id, const std::vector<Face<float> > &faces):
              rw::geometry::Geometry(id),_faces(faces){}

          virtual ~GeometryFaceWrap(){}

          virtual const std::vector<Face<float> >& getFaces() const{ return _faces;};
      private:
          const std::vector<Face<float> > &_faces;
      };
}

bool ProximityStrategy::addModel(const Frame* frame, const std::vector<Face<float> >& faces)
{
    ProximityModelPtr model = getModel(frame);
    if(model==NULL){
        model = createModel();
    }

    GeometryFaceWrap gface( frame->getName(), faces );
    bool res = addGeometry(model, gface);
    return res;
}

bool ProximityStrategy::hasModel(const rw::kinematics::Frame* frame){
    if( !_frameToModel.has( *frame ) || _frameToModel[*frame]==NULL){
        if (Accessor::collisionModelInfo().has(*frame))
            if(Accessor::collisionModelInfo().get(*frame).size()>0)
                return true;
        return false;
    }
    return true;
}

void ProximityStrategy::clearFrame(const rw::kinematics::Frame* frame){
    if( !_frameToModel.has( *frame ) || _frameToModel[*frame]==NULL )
        return;
    ProximityModelPtr model = _frameToModel[*frame];
    std::cout << "clear frame" << std::endl;
    if( model == NULL )
    	return;
    _frameToModel[*frame] = NULL;
    destroyModel(model);
}

void ProximityStrategy::clearFrames(){
    _frameToModel.clear();
}
