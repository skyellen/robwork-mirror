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
		Metric<Q>::Ptr q =
            MetricFactory::makeWeightedManhattan(Q());

		Metric<Vector2D<> >::Ptr v2 =
            MetricFactory::makeWeightedManhattan(Vector2D<>());

		Metric<Vector3D<> >::Ptr v3 =
            MetricFactory::makeWeightedManhattan(Vector3D<>());

		Metric<std::vector<double> >::Ptr sv =
            MetricFactory::makeWeightedManhattan(
                std::vector<double>());

		Metric<boost::numeric::ublas::vector<double> >::Ptr bv =
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
