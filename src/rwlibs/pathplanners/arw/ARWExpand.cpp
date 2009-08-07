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


#include "ARWExpand.hpp"
#include <rw/common/macros.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/models/Models.hpp>
#include <boost/foreach.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/math/Math.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;

//----------------------------------------------------------------------
// Methods on ARWExpand

bool ARWExpand::expand()
{
    return doExpand();
}

ARWExpandPtr ARWExpand::duplicate(const Q& start) const
{
    return doDuplicate(start);
}

//----------------------------------------------------------------------
// Concrete implementations of ARWExpand

namespace
{
    // The recommended variances to use for 'bounds'.
    Q makeMinVariances(const Device::QBox& bounds)
    {
        const Q diff = bounds.second - bounds.first;
        const Q deviation = (1.0 / 6) * diff;
        return Math::sqr(deviation);
    }

    // The sample variances for each dimension of the latest 'historySize'
    // samples.
    Q variance(const QPath& path, int historySize)
    {
        RW_ASSERT(path.size() >= 2);

        Q sum_squares = Q::zero(path.front().size());
        Q sum_elements = sum_squares;
        int cnt = 0;
        BOOST_FOREACH(const Q& q, std::make_pair(path.rbegin(), path.rend())) {
            if (cnt < historySize) {

                sum_squares += Math::sqr(q);
                sum_elements += q;

                ++cnt;
            } else {
                break;
            }
        }

        return
            (1 / (cnt - 1.0)) *
            (sum_squares - (1.0 / cnt) * Math::sqr(sum_elements));
    }

    // A random vector distributed according to 'variances'.
    Q gaussianStep(const Q& variances)
    {
        const int dim = (int)variances.size();
        Q result(dim);
        for (int i = 0; i < dim; i++)
            result[i] = Math::ranNormalDist(0, sqrt(variances[i]));
        return result;
    }

    class VarianceExpand : public ARWExpand
    {
    public:
        VarianceExpand(
            const Device::QBox& bounds,
            const PlannerConstraint& constraint,
            const Q& minVariances,
            int historySize,
            const Q& start)
            :
            _bounds(bounds),
            _constraint(constraint),
            _minVariances(minVariances),
            _historySize(historySize)
        {
            if (!start.empty())
                _path.push_back(start);
        }

    private:
        bool doExpand()
        {
            RW_ASSERT(!_path.empty());

            const Q xn = _path.back() + gaussianStep(updateVariances());

            if (
                Models::inBounds(xn, _bounds) &&
                !PlannerUtil::inCollision(
                    _constraint, _path.back(), xn, false, true))
            {
                _path.push_back(xn);
                return true;
            }
            else
                return false;
        }

        ARWExpandPtr doDuplicate(const rw::math::Q& start) const
        {
            return ownedPtr(
                new VarianceExpand(
                    _bounds,
                    _constraint,
                    _minVariances,
                    _historySize,
                    start));
        }

        Q updateVariances()
        {
            if (_path.size() == 1) return _minVariances;
            else {
                return Math::max(
                    variance(_path, _historySize),
                    _minVariances);
            }
        }

        Device::QBox _bounds;
        PlannerConstraint _constraint;
        Q _minVariances;
        int _historySize;
    };
}

ARWExpandPtr ARWExpand::make(
    const rw::models::Device::QBox& bounds,
    const rw::pathplanning::PlannerConstraint& constraint,
    const rw::math::Q& minVariances,
    int historySize)
{
    if (historySize < 0) historySize = 20;

    return ownedPtr(
        new VarianceExpand(
            bounds,
            constraint,
            minVariances.empty() ? makeMinVariances(bounds) : minVariances,
            historySize,
            Q()));
}
