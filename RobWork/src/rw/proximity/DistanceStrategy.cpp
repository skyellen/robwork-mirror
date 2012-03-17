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


#include "DistanceStrategy.hpp"

#include "ProximityStrategyData.hpp"

namespace rw { namespace proximity {

	DistanceStrategy::DistanceStrategy() {}
	DistanceStrategy::~DistanceStrategy() {}

	DistanceStrategy::Result DistanceStrategy::distance(
                          const kinematics::Frame* a,
                          const math::Transform3D<>& wTa,
                          const kinematics::Frame* b,
                          const math::Transform3D<>& wTb)
    {
        if(getModel(a)==NULL)			
			RW_THROW("Frame "<<a->getName()<<" has no Collision model attached!");

        if(getModel(b)==NULL)			
			RW_THROW("Frame "<<b->getName()<<" has no Collision model attached!");

        ProximityStrategyData data;
	    return distance(getModel(a), wTa, getModel(b), wTb, data);
    }

    DistanceStrategy::Result& DistanceStrategy::distance(
                          const kinematics::Frame* a,
                          const math::Transform3D<>& wTa,
                          const kinematics::Frame* b,
                          const math::Transform3D<>& wTb,
                          ProximityStrategyData &data)
    {
        if(getModel(a)==NULL)			
			RW_THROW("Frame "<<a->getName()<<" has no Collision model attached!");

        if(getModel(b)==NULL)			
			RW_THROW("Frame "<<b->getName()<<" has no Collision model attached!");

        return distance(getModel(a), wTa, getModel(b), wTb, data);
    }

} }
