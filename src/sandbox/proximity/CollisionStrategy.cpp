/*********************************************************************
 * RobWork Version 0.3
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
