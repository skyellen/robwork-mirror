/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "StateConstraint.hpp"
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
        FromCollisionDetector(
            CollisionDetectorPtr detector) :
            _detector(detector)
        {}

    private:
        bool doInCollision(const State& state) const
        {
            return _detector->inCollision(state);
        }

    private:
        Ptr<CollisionDetector> _detector;
    };

    class FromConstraints : public StateConstraint
    {
    public:
        FromConstraints(
            const std::vector<StateConstraintPtr>& constraints) :
            _constraints(constraints)
        {}

    private:
        bool doInCollision(const State& state) const
        {
            BOOST_FOREACH(const StateConstraintPtr& sc, _constraints) {
                if (sc->inCollision(state))
                    return true;
            }
            return false;
        }

    private:
        std::vector<StateConstraintPtr> _constraints;
    };

    typedef std::auto_ptr<StateConstraint> T;
}

bool StateConstraint::inCollision(const rw::kinematics::State& state) const
{
    return doInCollision(state);
}

T StateConstraint::make(CollisionDetectorPtr detector)
{
    return T(new FromCollisionDetector(detector));
}

T StateConstraint::make(
    const std::vector<StateConstraintPtr>& constraints)
{
    return T(new FromConstraints(constraints));
}
