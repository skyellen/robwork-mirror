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

#include "QEdgeConstraint.hpp"
#include "PlannerUtil.hpp"

using namespace rw::math;
using namespace rw::models;
using namespace rw::pathplanning;

QEdgeConstraint::QEdgeConstraint(
    const Q& start,
    const Q& end)
    :
    _start(start),
    _end(end)
{}

QEdgeConstraint::~QEdgeConstraint() {}

//----------------------------------------------------------------------
// Forwarding to the subclass methods.

double QEdgeConstraint::inCollisionCost() const
{
    return doInCollisionCost();
}

std::auto_ptr<QEdgeConstraint> QEdgeConstraint::instance(
    const Q& start,
    const Q& end) const
{
    return doClone(start, end);
}

bool QEdgeConstraint::inCollision(
    const rw::math::Q& start,
    const rw::math::Q& end) const
{
    return doInCollision(start, end);
}

bool QEdgeConstraint::inCollision()
{
    return doInCollision();
}

bool QEdgeConstraint::inCollisionPartialCheck()
{
    return doInCollisionPartialCheck();
}

bool QEdgeConstraint::isFullyChecked() const
{
    return doIsFullyChecked();
}

void QEdgeConstraint::reset(const rw::math::Q& start, const rw::math::Q& end)
{
    _start = start;
    _end = end;
    doReset();
}

//----------------------------------------------------------------------
// Default implementations of the subclass methods.

bool QEdgeConstraint::doInCollision(
    const rw::math::Q& start,
    const rw::math::Q& end) const
{
    return instance(start, end)->inCollision();
}

bool QEdgeConstraint::doInCollision()
{
    while (!isFullyChecked())
        if (inCollisionPartialCheck())
            return true;
    return false;
}

bool QEdgeConstraint::doInCollisionPartialCheck()
{
    return inCollision();
}

//----------------------------------------------------------------------
// Here comes the specific implementations of the edge constraint.

namespace
{
    class DiscreteLinear : public QEdgeConstraint
    {
    public:
        DiscreteLinear(
            const Q& start,
            const Q& end,
            QMetricPtr metric,
            double resolution,
            QConstraintPtr constraint)
            :
            QEdgeConstraint(start, end),
            _metric(metric),
            _resolution(resolution),
            _constraint(constraint)
        {
            doReset();
        }

    private:
        void doReset()
        {
            _len = _metric->distance(getStart(), getEnd());
            _startToEnd = getEnd() - getStart();
            _steps = 1;
            _inCollision = false;
        }

        double doInCollisionCost() const
        {
            return _len / _steps;
        }

        bool doInCollisionPartialCheck()
        {
            if (_inCollision) return true;

            const Q move = _startToEnd / _steps;
            const Q indent = 0.5 * move;
            Q pos = getStart() + 0.5 * move;

            for (int i = 0; i < _steps; i++, pos += move) {
                if (_constraint->inCollision(pos)) {
                    _inCollision = true;
                    return true;
                }
            }

            _steps *= 2;
            return false;
        }

        bool doIsFullyChecked() const
        {
            return _len / _steps < _resolution;
        }

        std::auto_ptr<QEdgeConstraint> doClone(
            const Q& from,
            const Q& to) const
        {
            typedef std::auto_ptr<QEdgeConstraint> T;
            return T(
                new DiscreteLinear(
                    from, to, _metric, _resolution, _constraint));
        }

    private:
        // These are fixed.
        QMetricPtr _metric;
        double _resolution;
        QConstraintPtr _constraint;

        // These are updated as the path is being verified.
        int _steps;
        double _len;
        Q _startToEnd;
        bool _inCollision;
    };
}

std::auto_ptr<QEdgeConstraint> QEdgeConstraint::make(
    QConstraintPtr constraint,
    QMetricPtr metric,
    double resolution)
{
    typedef std::auto_ptr<QEdgeConstraint> T;
    return T(new DiscreteLinear(Q(), Q(), metric, resolution, constraint));
}

std::auto_ptr<QEdgeConstraint> QEdgeConstraint::makeDefault(
    QConstraintPtr constraint,
    DevicePtr device)
{
    // We can be much more clever here, but this is what we are currently using:
    QMetricPtr metric = PlannerUtil::normalizingInfinityMetric(
        device->getBounds());
    const double resolution = 0.005;

    return QEdgeConstraint::make(constraint, metric, resolution);
}
