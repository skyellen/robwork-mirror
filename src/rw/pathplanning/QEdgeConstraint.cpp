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

using namespace rw::common;
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

QEdgeConstraintPtr QEdgeConstraint::instance(
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
    // Implements ceil(log_2(n)) exactly for n > 0.
    int ceil_log2(const int n)
    {
        RW_ASSERT(n > 0);
        int cnt = 0;
        int i = n;
        int a = 1;
        while (i != 1) {
            a <<= 1;
            i >>= 1;
            ++cnt;
        }
        if (a == n) return cnt;
        else return cnt + 1;
    }

    // This check should pass (and it does).
    int test_ceil_log2()
    {
        const bool ok =
            ceil_log2(1) == 0 &&
            ceil_log2(2) == 1 &&
            ceil_log2(3) == 2 &&
            ceil_log2(4) == 2 &&
            ceil_log2(5) == 3 &&
            ceil_log2(8) == 3 &&
            ceil_log2(9) == 4;
        RW_ASSERT(ok);
        return 0;
    }

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
            RW_ASSERT(resolution > 0);
            doReset();
        }

    private:
        void doReset()
        {
            _knownCollisionFree = false;
            _knownInCollision = false;

            /*
              These are the variables:

              Real position:

              0                   len1        len2
              |--------------------|------------|

              Integer position (number of steps of e = resolution):

              0          n = floor(len1 / e)  2^maxLevel
              |------------------|--------------|
              
              maxLevel is the smallest integer for which 2^maxLevel - 1 >= n.

              We only check for positions of i in the range 1 ... n.
            */

            const double len1 = _metric->distance(getStart(), getEnd());

            _maxPos = (int)floor(len1 / _resolution);

            _collisionChecks = 0;
            _level = 1;
            _maxLevel = ceil_log2(_maxPos + 1);

            _cost = pow(2.0, (double)_maxLevel);

            if (_level > _maxLevel) _knownCollisionFree = true;
            else {
                // Because of the above check this shouldn't be a division by
                // zero.
                _dir = (getEnd() - getStart()) / len1;
            }
        }

        double doInCollisionCost() const
        {
            // The number of collision checks that remain to be performed:
            // return _maxPos - _collisionChecks;

            // The length of the segments to check:
            return _cost;
            // This is the better cost to use.
        }

        bool doInCollisionPartialCheck()
        {
            if (_knownInCollision) return true;
            if (_knownCollisionFree) return false;

            int pos = 1 << (_maxLevel - _level);
            const int step = 2 * pos;

            while (pos <= _maxPos) {
                const Q q = getStart() + (pos * _resolution) * _dir;

                ++_collisionChecks;
                if (_constraint->inCollision(q)) {
                    _knownInCollision = true;
                    return true;
                } else {
                    pos += step;
                }
            }

            _level += 1;
            _cost /= 2;

            if (_level > _maxLevel) _knownCollisionFree = true;
            return false;
        }

        bool doIsFullyChecked() const
        {
            return _knownCollisionFree || _knownInCollision;
        }

        QEdgeConstraintPtr doClone(const Q& from, const Q& to) const
        {
            return ownedPtr(
                new DiscreteLinear(
                    from, to, _metric, _resolution, _constraint));
        }

    private:
        // These are fixed.
        QMetricPtr _metric;
        double _resolution;
        QConstraintPtr _constraint;

        // These are updated as the path is being verified.
        int _level;
        int _maxLevel;
        int _maxPos;
        double _cost;
        Q _dir;
        bool _knownInCollision;
        bool _knownCollisionFree;
        int _collisionChecks;
    };
}

QEdgeConstraintPtr QEdgeConstraint::make(
    QConstraintPtr constraint,
    QMetricPtr metric,
    double resolution)
{
    return ownedPtr(
        new DiscreteLinear(Q(), Q(), metric, resolution, constraint));
}

QEdgeConstraintPtr QEdgeConstraint::makeDefault(
    QConstraintPtr constraint,
    DevicePtr device)
{
    // Run the tests for ceil_log2() once.
    static const int ok_ceil_log2 = test_ceil_log2();

    // We can be much more clever here, but this is what we are currently using:
    QMetricPtr metric = PlannerUtil::normalizingInfinityMetric(
        device->getBounds());
    const double resolution = 0.01;

    return QEdgeConstraint::make(constraint, metric, resolution);
}
