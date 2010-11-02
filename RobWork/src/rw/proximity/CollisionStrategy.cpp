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

#include <list>
#include <stack>

using namespace rw::proximity;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;

namespace
{
    class ToleranceWrapper: public rw::proximity::CollisionStrategy
    {
    private:
		CollisionToleranceStrategy::Ptr _strategy;
        double _tolerance;

    public:
        ToleranceWrapper(
			CollisionToleranceStrategy::Ptr strategy,
            double tolerance)
            :
            _strategy(strategy),
            _tolerance(tolerance)
        {}

        void setFirstContact(bool b) {}

		ProximityModel::Ptr createModel(){ return _strategy->createModel();};

        void destroyModel(ProximityModel* model){ _strategy->destroyModel(model);};

        virtual bool addGeometry(ProximityModel* model, const rw::geometry::Geometry& geom)
        { return _strategy->addGeometry(model,geom);};

        virtual bool removeGeometry(ProximityModel* model, const std::string& geomId)
        { return _strategy->removeGeometry(model, geomId);}

        virtual std::vector<std::string> getGeometryIDs(ProximityModel* model)
        { return _strategy->getGeometryIDs(model);}

		bool collides(ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb)
        {
            return _strategy->collides(a, wTa, b, wTb, _tolerance);
        }

        void clear()
        {
            _strategy->clear();
        }

    };
}

CollisionStrategy::CollisionStrategy() {}
CollisionStrategy::~CollisionStrategy() {}

CollisionStrategy::Ptr CollisionStrategy::make(
	CollisionToleranceStrategy::Ptr strategy,
    double tolerance)
{
    return ownedPtr(new ToleranceWrapper(strategy, tolerance));
}

bool CollisionStrategy::inCollision(
    const Frame* a, const Transform3D<>& wTa,
    const Frame *b, const Transform3D<>& wTb)
{
    if( getModel(a)==NULL || getModel(b)==NULL)
        return false;
    return collides(getModel(a), wTa, getModel(b), wTb);
}
