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


#include "StateConstraint.hpp"
#include <rw/common/macros.hpp>
#include <rw/common/Log.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <boost/foreach.hpp>

using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;

namespace
{
    class FromCollisionDetector : public StateConstraint
    {
    public:
        FromCollisionDetector(CollisionDetector::Ptr detector) :
            _detector(detector)
        {}

    private:
        bool doInCollision(const State& state) const
        {
			if (_log == NULL)
				return _detector->inCollision(state, 0, true);
			else {
				CollisionDetector::QueryResult res;
				bool inCollision = _detector->inCollision(state, &res, true);
				BOOST_FOREACH(FramePair pair, res.collidingFrames) {
					_log->debug()<<RW_MSG("Colliding Frames: "<<pair.first->getName()<<" - "<<pair.second->getName());
				}
				return inCollision;
			}	
        }

		void doSetLog(Log::Ptr log) {
			_log = log;
		}

    private:
		rw::common::Ptr<CollisionDetector> _detector;
		Log::Ptr _log;
    };

    class FromConstraints : public StateConstraint
    {
    public:
        FromConstraints(
			const std::vector<StateConstraint::Ptr>& constraints) :
            _constraints(constraints)
        {}

    private:
        bool doInCollision(const State& state) const
        {
			BOOST_FOREACH(const StateConstraint::Ptr& sc, _constraints) {
                if (sc->inCollision(state))
                    return true;
            }
            return false;
        }

		void doSetLog(Log::Ptr log) {
			BOOST_FOREACH(const StateConstraint::Ptr& sc, _constraints) {
                sc->setLog(log);
			}
		}

    private:
		std::vector<StateConstraint::Ptr> _constraints;
    };
}

bool StateConstraint::inCollision(const rw::kinematics::State& state) const
{
    return doInCollision(state);
}

void StateConstraint::setLog(rw::common::Log::Ptr log) {
	doSetLog(log);
}

StateConstraint::Ptr StateConstraint::make(CollisionDetector::Ptr detector)
{
    return ownedPtr(new FromCollisionDetector(detector));
}

StateConstraint::Ptr StateConstraint::make(
	const std::vector<StateConstraint::Ptr>& constraints)
{
    return ownedPtr(new FromConstraints(constraints));
}
