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


#include "QEdgeConstraintIncremental.hpp"
#include "PlannerUtil.hpp"
#include <rw/math/Math.hpp>
#include <rw/models/Device.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::pathplanning;

QEdgeConstraintIncremental::QEdgeConstraintIncremental(const Q& start,
                                 const Q& end):
    _start(start),
    _end(end)
{}

QEdgeConstraintIncremental::~QEdgeConstraintIncremental() {}

//----------------------------------------------------------------------
// Forwarding to the subclass methods.

double QEdgeConstraintIncremental::inCollisionCost() const
{
    return doInCollisionCost();
}

QEdgeConstraintIncremental::Ptr QEdgeConstraintIncremental::instance(const Q& start, const Q& end) const
{
    return doClone(start, end);
}

bool QEdgeConstraintIncremental::inCollision(const rw::math::Q& start, const rw::math::Q& end) const
{
    return doInCollision(start, end);
}

bool QEdgeConstraintIncremental::inCollision()
{
    return doInCollision();
}

bool QEdgeConstraintIncremental::inCollisionPartialCheck()
{
    return doInCollisionPartialCheck();
}

bool QEdgeConstraintIncremental::isFullyChecked() const
{
    return doIsFullyChecked();
}

void QEdgeConstraintIncremental::reset(const rw::math::Q& start, const rw::math::Q& end)
{
    _start = start;
    _end = end;
    doReset();
}

//----------------------------------------------------------------------
// Default implementations of the subclass methods.

bool QEdgeConstraintIncremental::doInCollision(const rw::math::Q& start, const rw::math::Q& end) const
{
    return instance(start, end)->inCollision();
}

bool QEdgeConstraintIncremental::doInCollision()
{
    while (!isFullyChecked())
        if (inCollisionPartialCheck())
            return true;
    return inCollisionPartialCheck();
}

bool QEdgeConstraintIncremental::doInCollisionPartialCheck()
{
    return inCollision();
}

//----------------------------------------------------------------------
// Here comes the specific implementations of the edge constraint.

namespace
{
    class DiscreteLinear : public QEdgeConstraintIncremental
    {
    public:
        DiscreteLinear(
            const Q& start,
            const Q& end,
			QMetric::Ptr metric,
            double resolution,
			QConstraint::Ptr constraint)
            :
            QEdgeConstraintIncremental(start, end),
            _metric(metric),
            _resolution(resolution),
            _constraint(constraint)
        {
            if (resolution <= 0)
                RW_THROW("Unable to create constraint with resolution<=0");

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
            _maxLevel = Math::ceilLog2(_maxPos + 1);

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

		QEdgeConstraintIncremental::Ptr doClone(const Q& from, const Q& to) const
        {
            return ownedPtr(
                new DiscreteLinear(
                    from, to, _metric, _resolution, _constraint));
        }

    private:
        // These are fixed.
		QMetric::Ptr _metric;
        double _resolution;
		QConstraint::Ptr _constraint;

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

    class FixedConstraint : public QEdgeConstraintIncremental
    {
    public:
        FixedConstraint(bool value) :
            QEdgeConstraintIncremental(Q(), Q()),
            _value(value)
        {}

    private:
        bool doInCollision(const Q& start,
                           const Q& end) const
        { return _value; }

        bool doInCollision() { return _value; }

        double doInCollisionCost() const { return 0; }

        bool doInCollisionPartialCheck() { return _value; }

        bool doIsFullyChecked() const { return true; }

		QEdgeConstraintIncremental::Ptr doClone(
            const Q&, const Q&) const
        { return ownedPtr(new FixedConstraint(_value)); }

        void doReset() {}

    private:
        bool _value;
    };
}

QEdgeConstraintIncremental::Ptr QEdgeConstraintIncremental::make(QConstraint::Ptr constraint,
	QMetric::Ptr metric,
    double resolution)
{
    return ownedPtr(new DiscreteLinear(Q(), Q(), metric, resolution, constraint));
}

QEdgeConstraintIncremental::Ptr QEdgeConstraintIncremental::makeDefault(QConstraint::Ptr constraint,
												  Device::Ptr device)
{
    // We can be much more clever here, but this is what we are currently using:
	QMetric::Ptr metric = PlannerUtil::normalizingInfinityMetric(device->getBounds());
    const double resolution = 0.01;

    return QEdgeConstraintIncremental::make(constraint, metric, resolution);
}

QEdgeConstraintIncremental::Ptr QEdgeConstraintIncremental::makeFixed(bool value)
{
    return ownedPtr(new FixedConstraint(value));
}
