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

/*
#include "EuclideanMetric.hpp"
#include "WeightedEuclideanMetric.hpp"
#include "InfinityMetric.hpp"
*/
using namespace rw::math;

namespace
{
    //TODO: Make into a separate file to match how it is done elsewhere.
  /*  template <class T>
    class WeightedInfinityMetric: public Metric<T>
    {
    private:
        Q _weights;

    public:
        WeightedInfinityMetric(const Q& weights) :
            _weights(weights)
        {}


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
    };*/
}

template <class T>
std::auto_ptr<Metric<T> > Metric<T>::makeEuclidean()
{
    typedef std::auto_ptr<Metric<T> > Type;
    return Type(new EuclideanMetric<T>());
}

template <class T>
std::auto_ptr<Metric<T> > Metric<T>::makeWeightedEuclidean(const Q& weights)
{
    typedef std::auto_ptr<Metric<T> > Type;
    return Type(new WeightedEuclideanMetric<T>(weights.m()));
}

template <class T>
std::auto_ptr<Metric<T> > Metric<T>::makeInfinity()
{
    typedef std::auto_ptr<Metric<T> > Type;
    return Type(new InfinityMetric<T>());
}

template <class T>
std::auto_ptr<Metric<T> > Metric<T>::makeWeightedInfinity(const Q& weights)
{
    typedef std::auto_ptr<Metric<T> > Type;
    return Type(new WeightedInfinityMetric<T>(weights));
}

template <class T>
std::auto_ptr<Metric<T> > Metric<T>::makeMahalanobisMetric(const boost::numeric::ublas::matrix<T>& omega) {
    typedef std::auto_ptr<Metric<T> > Type;
    return Type(new MahalanobisMetric<T>(omega));
}

template <class T>
std::auto_ptr<Metric<T> > Metric<T>::makeManhattenMetric() {
    typedef std::auto_ptr<Metric<T> > Type;
    return Type(new ManhattenMetric<T>());
}


template class Metric<double>;
template class Metric<float>;
template class Metric<int>;
