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


#include "QEdgeConstraint.hpp"
#include "PlannerUtil.hpp"
#include <rw/math/Math.hpp>
#include <rw/models/Device.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::pathplanning;



QEdgeConstraint::~QEdgeConstraint() {}


//----------------------------------------------------------------------
// Here comes the specific implementations of the edge constraint.

namespace
{
    class ExpandedBinary: public QEdgeConstraint
    {
    public:
        ExpandedBinary(rw::common::Ptr<QConstraint> constraint,
				QMetric::CPtr metric,
				double resolution)
            :
            _metric(metric),
            _resolution(resolution),
            _constraint(constraint)
            
        {
            if (resolution <= 0)
                RW_THROW("Unable to create constraint with resolution<=0");

        }

    private:
		bool doInCollision(const Q&	start, const Q& end) const
        {

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

            const double len1 = _metric->distance(start, end);

            int maxPos = (int)floor(len1 / _resolution);
			int maxLevel = Math::ceilLog2(maxPos + 1);
            int level = 1;
            
			Q dir = (end - start) / len1;
			while (level <= maxLevel) {
				int pos = 1 << (maxLevel - level);
				const int step = 2 * pos;

				while (pos <= maxPos) {
					const Q q = start + (pos * _resolution) * dir;

					if (_constraint->inCollision(q)) {
						return true;
					} else {
						pos += step;
					}
				}
				level += 1;			
			} 
			return false;

        }
    private:
        // These are fixed.
		QMetric::CPtr _metric;
        double _resolution;
        rw::common::Ptr<QConstraint> _constraint;
    };

	class MergedEdgeConstraint: public QEdgeConstraint {
	public:
		MergedEdgeConstraint(const std::vector<QEdgeConstraint::Ptr>& constraints):
		  _constraints(constraints)
		{}
	private:
		bool doInCollision(const Q& qstart, const Q& qend) const {
			BOOST_FOREACH(QEdgeConstraint::Ptr constraint, _constraints) {
				if (constraint->inCollision(qstart, qend))
					return true;
			}
			return false;
		}

		std::vector<QEdgeConstraint::Ptr> _constraints;
	};

}

QEdgeConstraint::Ptr QEdgeConstraint::make(rw::common::Ptr<QConstraint> constraint,
	QMetric::CPtr metric,
    double resolution)
{
    return ownedPtr(new ExpandedBinary(constraint, metric, resolution));
}

QEdgeConstraint::Ptr QEdgeConstraint::makeDefault(rw::common::Ptr<QConstraint> constraint,
												  Device::CPtr device)
{
    // We can be much more clever here, but this is what we are currently using:
	QMetric::Ptr metric = PlannerUtil::normalizingInfinityMetric(device->getBounds());
    const double resolution = 0.01;

    return QEdgeConstraint::make(constraint, metric, resolution);
}

QEdgeConstraint::Ptr QEdgeConstraint::makeMerged(const std::vector<QEdgeConstraint::Ptr>& constraints) {
	return ownedPtr(new MergedEdgeConstraint(constraints));
}

QEdgeConstraint::Ptr QEdgeConstraint::makeMerged(QEdgeConstraint::Ptr constraint1, QEdgeConstraint::Ptr constraint2) {
	std::vector<QEdgeConstraint::Ptr> constraints;
	constraints.push_back(constraint1);
	constraints.push_back(constraint2);
	return makeMerged(constraints);
}
