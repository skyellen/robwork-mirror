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

#include "Metric.hpp"
#include "MetricUtil.hpp"

#include "EuclideanMetric.hpp"
#include "WeightedEuclideanMetric.hpp"
#include "InfinityMetric.hpp"

using namespace rw::math;

namespace
{
    class WeightedInfinityMetric: public Metric<>
    {
    private:
        Q _weights;

    public:
        WeightedInfinityMetric(const Q& weights) :
            _weights(weights)
        {}

        typedef double T;

        T distance(const rw::math::Q& q) const
        {
            return MetricUtil::normInfWeighted(q, _weights);
        }

        T distance(const rw::math::Q& a, const rw::math::Q& b) const
        {
            return MetricUtil::distInfWeighted(a, b, _weights);
        }

        T distance(const boost::numeric::ublas::vector<T>& q) const
        {
            T max = 0;
            for (size_t i = 0; i < q.size(); i++) {
                const T val = fabs(_weights(i) * q[i]);
                if (max < val) max = val;
            }
            return max;
        }

        T distance(
            const boost::numeric::ublas::vector<T>& a,
            const boost::numeric::ublas::vector<T>& b) const
        {
            T max = 0;
            for (size_t i = 0; i < a.size(); i++) {
                const T val = fabs(_weights(i) * (a[i] - b[i]));
                if (max < val) max = val;
            }
            return max;
        }

        virtual int size() const { return _weights.size(); }
    };
}

std::auto_ptr<Metric<> > Metric<>::makeEuclidean()
{
    typedef std::auto_ptr<Metric<> > T;
    return T(new EuclideanMetric<>());
}

std::auto_ptr<Metric<> > makeWeightedEuclidean(const Q& weights)
{
    typedef std::auto_ptr<Metric<> > T;
    return T(new WeightedEuclideanMetric<>(weights.m()));
}

std::auto_ptr<Metric<> > Metric<>::makeInfinity()
{
    typedef std::auto_ptr<Metric<> > T;
    return T(new InfinityMetric<>());
}

std::auto_ptr<Metric<> > Metric<>::makeWeightedInfinity(const Q& weights)
{
    typedef std::auto_ptr<Metric<> > T;
    return T(new WeightedInfinityMetric(weights));
}
