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

template class rw::math::ManhattanMetric<Q>;
template class rw::math::WeightedManhattanMetric<Q>;

template class rw::math::EuclideanMetric<Q>;
template class rw::math::WeightedEuclideanMetric<Q>;

template class rw::math::InfinityMetric<Q>;
template class rw::math::WeightedInfinityMetric<Q>;

// --
#include "Vector2D.hpp"

template class rw::math::ManhattanMetric<Vector2D<> >;
template class rw::math::WeightedManhattanMetric<Vector2D<> >;

template class rw::math::EuclideanMetric<Vector2D<> >;
template class rw::math::WeightedEuclideanMetric<Vector2D<> >;

template class rw::math::InfinityMetric<Vector2D<> >;
template class rw::math::WeightedInfinityMetric<Vector2D<> >;

template class rw::math::ManhattanMetric<Vector2D<float> >;
template class rw::math::WeightedManhattanMetric<Vector2D<float> >;

template class rw::math::EuclideanMetric<Vector2D<float> >;
template class rw::math::WeightedEuclideanMetric<Vector2D<float> >;

template class rw::math::InfinityMetric<Vector2D<float> >;
template class rw::math::WeightedInfinityMetric<Vector2D<float> >;

// --
#include "Vector3D.hpp"

template class rw::math::ManhattanMetric<Vector3D<> >;
template class rw::math::WeightedManhattanMetric<Vector3D<> >;

template class rw::math::EuclideanMetric<Vector3D<> >;
template class rw::math::WeightedEuclideanMetric<Vector3D<> >;

template class rw::math::InfinityMetric<Vector3D<> >;
template class rw::math::WeightedInfinityMetric<Vector3D<> >;

template class rw::math::ManhattanMetric<Vector3D<float> >;
template class rw::math::WeightedManhattanMetric<Vector3D<float> >;

template class rw::math::EuclideanMetric<Vector3D<float> >;
template class rw::math::WeightedEuclideanMetric<Vector3D<float> >;

template class rw::math::InfinityMetric<Vector3D<float> >;
template class rw::math::WeightedInfinityMetric<Vector3D<float> >;

// --
#include <boost/numeric/ublas/vector.hpp>

template class rw::math::ManhattanMetric<boost::numeric::ublas::vector<double> >;
template class rw::math::WeightedManhattanMetric<boost::numeric::ublas::vector<double> >;

template class rw::math::EuclideanMetric<boost::numeric::ublas::vector<double> >;
template class rw::math::WeightedEuclideanMetric<boost::numeric::ublas::vector<double> >;

template class rw::math::InfinityMetric<boost::numeric::ublas::vector<double> >;
template class rw::math::WeightedInfinityMetric<boost::numeric::ublas::vector<double> >;

template class rw::math::ManhattanMetric<boost::numeric::ublas::vector<float> >;
template class rw::math::WeightedManhattanMetric<boost::numeric::ublas::vector<float> >;

template class rw::math::EuclideanMetric<boost::numeric::ublas::vector<float> >;
template class rw::math::WeightedEuclideanMetric<boost::numeric::ublas::vector<float> >;

template class rw::math::InfinityMetric<boost::numeric::ublas::vector<float> >;
template class rw::math::WeightedInfinityMetric<boost::numeric::ublas::vector<float> >;

// --

#include <vector>

template class rw::math::ManhattanMetric<std::vector<double> >;
template class rw::math::WeightedManhattanMetric<std::vector<double> >;

template class rw::math::EuclideanMetric<std::vector<double> >;
template class rw::math::WeightedEuclideanMetric<std::vector<double> >;

template class rw::math::InfinityMetric<std::vector<double> >;
template class rw::math::WeightedInfinityMetric<std::vector<double> >;

template class rw::math::ManhattanMetric<std::vector<float> >;
template class rw::math::WeightedManhattanMetric<std::vector<float> >;

template class rw::math::EuclideanMetric<std::vector<float> >;
template class rw::math::WeightedEuclideanMetric<std::vector<float> >;

template class rw::math::InfinityMetric<std::vector<float> >;
template class rw::math::WeightedInfinityMetric<std::vector<float> >;

// --

// You can remove this math -> kinematics dependency if you want. Right now we
// leave it here just to check that states can be treated as vectors.

#include <rw/kinematics/State.hpp>
using namespace rw::kinematics;

template class rw::math::ManhattanMetric<State>;
template class rw::math::WeightedManhattanMetric<State>;

template class rw::math::EuclideanMetric<State>;
template class rw::math::WeightedEuclideanMetric<State>;

template class rw::math::InfinityMetric<State>;
template class rw::math::WeightedInfinityMetric<State>;
