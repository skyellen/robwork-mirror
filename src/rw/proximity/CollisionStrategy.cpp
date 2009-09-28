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
        CollisionToleranceStrategyPtr _strategy;
        double _tolerance;

    public:
        ToleranceWrapper(
            CollisionToleranceStrategyPtr strategy,
            double tolerance)
            :
            _strategy(strategy),
            _tolerance(tolerance)
        {}

        void setFirstContact(bool b) {}

        ProximityModelPtr createModel(){ return _strategy->createModel();};

        void destroyModel(ProximityModelPtr model){ _strategy->destroyModel(model);};

        virtual bool addGeometry(ProximityModelPtr model, const rw::geometry::Geometry& geom)
        { return _strategy->addGeometry(model,geom);};

        virtual bool removeGeometry(ProximityModelPtr model, const std::string& geomId)
        { return _strategy->removeGeometry(model, geomId);}

        bool collides(ProximityModelPtr a,
                         const rw::math::Transform3D<>& wTa,
                         ProximityModelPtr b,
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

CollisionStrategyPtr CollisionStrategy::make(
    CollisionToleranceStrategyPtr strategy,
    double tolerance)
{
    return ownedPtr(new ToleranceWrapper(strategy, tolerance));
}

bool CollisionStrategy::inCollision(
    const Frame* a, const Transform3D<>& wTa,
    const Frame *b, const Transform3D<>& wTb)
{
    return collides(getModel(a), wTa, getModel(b), wTb);
}
