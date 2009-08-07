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

using namespace rw::proximity::sandbox;
using namespace rw::common;

namespace
{
    class ToleranceWrapper: public rw::proximity::CollisionStrategy
    {
    private:
        rw::proximity::CollisionToleranceStrategyPtr _strategy;
        double _tolerance;

    public:
        ToleranceWrapper(
        	rw::proximity::CollisionToleranceStrategyPtr strategy,
            double tolerance)
            :
            _strategy(strategy),
            _tolerance(tolerance)
        {}

        bool addModel(const rw::kinematics::Frame *frame)
        {
            return _strategy->addModel(frame);
        }

        bool addModel(
            const rw::kinematics::Frame* frame,
            const std::vector<rw::geometry::Face<float> >& faces)
        {
            return _strategy->addModel(frame, faces);
        }

        void setFirstContact(bool b) {}

        bool inCollision(
            const rw::kinematics::Frame* a,
            const rw::math::Transform3D<>& wTa,
            const rw::kinematics::Frame *b,
            const rw::math::Transform3D<>& wTb)
        {
            return _strategy->inCollision(a, wTa, b, wTb, _tolerance);
        }

        void clear()
        {
            _strategy->clear();
        }

        void clearFrame(const rw::kinematics::Frame* frame)
        {
            _strategy->clearFrame(frame);
        }

        bool hasModel(const rw::kinematics::Frame* frame)
        {
            return _strategy->hasModel(frame);
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
