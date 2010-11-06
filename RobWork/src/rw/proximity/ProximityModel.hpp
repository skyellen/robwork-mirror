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

#ifndef PROXIMITYMODEL_HPP_
#define PROXIMITYMODEL_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/geometry/Geometry.hpp>

namespace rw {
namespace proximity {

    class ProximityStrategy;

    class ProximityModel {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<ProximityModel> Ptr;

		ProximityModel(rw::common::Ptr<ProximityStrategy> pOwner):
            owner(pOwner)
        {}

        virtual ~ProximityModel();

        std::vector<std::string> getGeometryIDs();

        //void addModel(const CollisionModelInfo& model){
        //    _collisionModels.push_back();
        //}

        //std::vector<CollisionModelInfo> _collisionModels;


        bool addGeometry(const rw::geometry::Geometry& geom);
        bool removeGeometry(const std::string& geoid);

        rw::kinematics::Frame* getFrame(){ return _frame;};
        void setFrame(rw::kinematics::Frame* frame){ _frame = frame; }

		rw::common::Ptr<ProximityStrategy> owner;
    private:
		rw::kinematics::Frame* _frame;
    };
#ifdef RW_USE_DEPRECATED
    typedef rw::common::Ptr<ProximityModel> ProximityModelPtr;
#endif
}
}

#endif /* PROXIMITYMODEL_HPP_ */
