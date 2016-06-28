/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_MATH_FWD_HPP_
#define RW_MATH_FWD_HPP_

namespace rw {
namespace math {
	template<class T> class CameraMatrix;
	//class Constants;
	template<class T> struct EigenDecomposition;
	template<class T> class EAA;
	template<class RES_T, class ARG_T> class Function;
	template<class T> class InertiaMatrix;
	class Jacobian;
	class Line2D;
	class Line2DPolar;
	class LinearAlgebra;
	//class Math;
	class Line2DPolar;
	template<class T> class Metric;
	template<class T> class ManhattanMetric;
	template<class T> class WeightedManhattanMetric;
	template<class T> class EuclideanMetric;
	template<class T> class WeightedEuclideanMetric;
	template<class T> class InfinityMetric;
	template<class T> class WeightedInfinityMetric;
	template<class T> class MahalanobisMetric;
	template<class T> class Rotation3DAngleMetric;
	template<class T> class Transform3DAngleMetric;
	class MetricFactory;
	class MetricUtil;
	template<class T> class PerspectiveTransform2D;
	template<class T> class Polynomial;
	template<class T> class Pose2D;
	template<class T> class Pose6D;
	class ProjectionMatrix;
	class Q;
	template<class T> class Quaternion;
	class Random;
	template<class T> class Rotation2D;
	template<class T> class Rotation3D;
	template<class T> class Rotation3DVector;
	template<class T> class RPY;
	template<class T> class Statistics;
	template<class T> class Transform2D;
	template<class T> class Transform3D;
	template<class T> class Vector;
	template<class T> class Vector2D;
	template<class T> class Vector3D;
	template<size_t N, class T> class VectorND;
	template<class T> class VelocityScrew6D;
	template<class T> class Wrench6D;
}
}

#endif /* RW_MATH_FWD_HPP_ */
