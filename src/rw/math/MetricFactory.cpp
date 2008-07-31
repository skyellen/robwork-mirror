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

#include "MetricFactory.hpp"

#include "Vector2D.hpp"
#include "Vector3D.hpp"

#include <vector>
#include <boost/numeric/ublas/vector.hpp>

using namespace rw::math;
using namespace rw::common;

namespace
{
    // This should compile:
    void f()
    {
        Ptr<Metric<Q> > q =
            MetricFactory::makeWeightedManhattan(Q());

        Ptr<Metric<Vector2D<> > > v2 =
            MetricFactory::makeWeightedManhattan(Vector2D<>());

        Ptr<Metric<Vector3D<> > > v3 =
            MetricFactory::makeWeightedManhattan(Vector3D<>());

        Ptr<Metric<std::vector<double> > > sv =
            MetricFactory::makeWeightedManhattan(
                std::vector<double>());

        Ptr<Metric<boost::numeric::ublas::vector<double> > > bv =
            MetricFactory::makeWeightedManhattan(
                boost::numeric::ublas::vector<double>());
    }
}

// These should all compile:

// --

template class ManhattanMetric<Q>;
template class WeightedManhattanMetric<Q>;

template class EuclideanMetric<Q>;
template class WeightedEuclideanMetric<Q>;

template class InfinityMetric<Q>;
template class WeightedInfinityMetric<Q>;

// --
#include "Vector2D.hpp"

template class ManhattanMetric<Vector2D<> >;
template class WeightedManhattanMetric<Vector2D<> >;

template class EuclideanMetric<Vector2D<> >;
template class WeightedEuclideanMetric<Vector2D<> >;

template class InfinityMetric<Vector2D<> >;
template class WeightedInfinityMetric<Vector2D<> >;

template class ManhattanMetric<Vector2D<float> >;
template class WeightedManhattanMetric<Vector2D<float> >;

template class EuclideanMetric<Vector2D<float> >;
template class WeightedEuclideanMetric<Vector2D<float> >;

template class InfinityMetric<Vector2D<float> >;
template class WeightedInfinityMetric<Vector2D<float> >;

// --
#include "Vector3D.hpp"

template class ManhattanMetric<Vector3D<> >;
template class WeightedManhattanMetric<Vector3D<> >;

template class EuclideanMetric<Vector3D<> >;
template class WeightedEuclideanMetric<Vector3D<> >;

template class InfinityMetric<Vector3D<> >;
template class WeightedInfinityMetric<Vector3D<> >;

template class ManhattanMetric<Vector3D<float> >;
template class WeightedManhattanMetric<Vector3D<float> >;

template class EuclideanMetric<Vector3D<float> >;
template class WeightedEuclideanMetric<Vector3D<float> >;

template class InfinityMetric<Vector3D<float> >;
template class WeightedInfinityMetric<Vector3D<float> >;

// --
#include <boost/numeric/ublas/vector.hpp>

template class ManhattanMetric<boost::numeric::ublas::vector<double> >;
template class WeightedManhattanMetric<boost::numeric::ublas::vector<double> >;

template class EuclideanMetric<boost::numeric::ublas::vector<double> >;
template class WeightedEuclideanMetric<boost::numeric::ublas::vector<double> >;

template class InfinityMetric<boost::numeric::ublas::vector<double> >;
template class WeightedInfinityMetric<boost::numeric::ublas::vector<double> >;

template class ManhattanMetric<boost::numeric::ublas::vector<float> >;
template class WeightedManhattanMetric<boost::numeric::ublas::vector<float> >;

template class EuclideanMetric<boost::numeric::ublas::vector<float> >;
template class WeightedEuclideanMetric<boost::numeric::ublas::vector<float> >;

template class InfinityMetric<boost::numeric::ublas::vector<float> >;
template class WeightedInfinityMetric<boost::numeric::ublas::vector<float> >;

// --

#include <vector>

template class ManhattanMetric<std::vector<double> >;
template class WeightedManhattanMetric<std::vector<double> >;

template class EuclideanMetric<std::vector<double> >;
template class WeightedEuclideanMetric<std::vector<double> >;

template class InfinityMetric<std::vector<double> >;
template class WeightedInfinityMetric<std::vector<double> >;

template class ManhattanMetric<std::vector<float> >;
template class WeightedManhattanMetric<std::vector<float> >;

template class EuclideanMetric<std::vector<float> >;
template class WeightedEuclideanMetric<std::vector<float> >;

template class InfinityMetric<std::vector<float> >;
template class WeightedInfinityMetric<std::vector<float> >;

// --

// You can remove this math -> kinematics dependency if you want. Right now we
// leave it here just to check that states can be treated as vectors.

#include <rw/kinematics/State.hpp>
using namespace rw::kinematics;

template class ManhattanMetric<State>;
template class WeightedManhattanMetric<State>;

template class EuclideanMetric<State>;
template class WeightedEuclideanMetric<State>;

template class InfinityMetric<State>;
template class WeightedInfinityMetric<State>;
