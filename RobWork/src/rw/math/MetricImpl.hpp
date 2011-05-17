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

#ifndef RW_MATH_METRIC_INTERNAL_HPP
#define RW_MATH_METRIC_INTERNAL_HPP

#include "Math.hpp"
#include <vector>

namespace rw {
    namespace math {
        namespace internal {

            template<class Operator, class VectorType>
            inline typename VectorType::value_type accumulateNorm(const VectorType& q, Operator op)
            {
                typename VectorType::value_type result = 0;
                const size_t len = q.size();
                for (size_t i = 0; i < len; i++) {
                    op(result, q[i]);
                }
                return result;
            }

            template <class VectorType,class Operator>
            inline typename VectorType::value_type accumulateNormWeighted(const VectorType& q, Operator op)
            {
                typename VectorType::value_type result = 0;
                const size_t len = q.size();
                for (size_t i = 0; i < len; i++) {
                    op(result, q[i], i);
                }
                return result;
            }

            template <class VectorType, class Operator>
            inline typename VectorType::value_type accumulateDist(const VectorType& a,
                                                                  const VectorType& b,
                                                                  Operator op)
            {
                typename VectorType::value_type result = 0;
                const size_t len = a.size();
                for (size_t i = 0; i < len; i++) {
                    op(result, a[i] - b[i]);
                }
                return result;
            }

            template <class VectorType,class Operator>
            inline typename VectorType::value_type accumulateDistWeighted(const VectorType& a,
                                                                          const VectorType& b,
                                                                          Operator op)
            {
                typename VectorType::value_type result = 0;
                const size_t len = a.size();
                for (size_t i = 0; i < len; i++) {
                    op(result, a[i] - b[i], i);
                }
                return result;
            }

            template <class VectorType, template <typename> class Norm>
            inline
            typename VectorType::value_type
            accumulateNormHelper(const VectorType& q)
            {
                typename Norm<VectorType>::operator_type op;
                return internal::accumulateNorm(q, op);
            }

            template <class VectorType, template <typename> class Norm>
            inline
            typename VectorType::value_type
            accumulateDistHelper(const VectorType& a, const VectorType& b)
            {
                typename Norm<VectorType>::operator_type op;
                return internal::accumulateDist(a, b, op);
            }

            template <class VectorType, template <typename> class Norm>
            inline
            typename VectorType::value_type
            accumulateNormWeightedHelper(const VectorType& q, const VectorType& scale)
            {
                typename Norm<Q>::operator_type op(&scale);
                return internal::accumulateNormWeighted(q, op);
            }

            template <class VectorType, template <typename> class Norm>
            inline
            typename VectorType::value_type
            accumulateDistWeightedHelper(const VectorType& a, const VectorType& b, const VectorType& scale)
            {
                typename Norm<VectorType>::operator_type op(&scale);
                return internal::accumulateDistWeighted(a, b, op);
            }

            // --

            template <class VectorType, template <typename> class Norm>
            inline
            typename VectorType::value_type
            normHelper(const VectorType& q)
            {
                typename Norm<VectorType>::operator_type op;
                return op.done(internal::accumulateNorm(q, op));
            }

            template <class VectorType, template <typename> class Norm>
            inline
            typename VectorType::value_type
            distHelper(const VectorType& a, const VectorType& b)
            {
                typename Norm<VectorType>::operator_type op;
                return op.done(internal::accumulateDist(a, b, op));
            }

            template <class VectorType, template <typename> class Norm>
            inline
            typename VectorType::value_type
            normWeightedHelper(const VectorType& q, const VectorType& scale)
            {
                typename Norm<Q>::operator_type op(&scale);
                return op.done(internal::accumulateNormWeighted(q, op));
            }

            template <class VectorType, template <typename> class Norm>
            inline
            typename VectorType::value_type
            distWeightedHelper(const VectorType& a, const VectorType& b, const VectorType& scale)
            {
                typename Norm<VectorType>::operator_type op(&scale);
                return op.done(internal::accumulateDistWeighted(a, b, op));
            }

            template <class ScalarType>
            struct ManhattanOperator
            {
                inline void operator()(ScalarType& result, ScalarType val) const
                {
                    result += std::fabs(val);
                }

                inline ScalarType done(ScalarType result) const
                {
                    return result;
                }
            };

            template <class ScalarType>
            struct EuclideanOperator
            {
                inline void operator()(ScalarType& result, ScalarType val) const
                {
                    result += Math::sqr(val);
                }

                inline ScalarType done(ScalarType result) const
                {
                    return sqrt(result);
                }
            };

            template <class ScalarType>
            struct InfinityOperator
            {
                inline void operator()(ScalarType& result, ScalarType val) const
                {
                    const ScalarType abs_val = std::fabs(val);
                    if (abs_val> result) result = abs_val;
                }

                inline ScalarType done(ScalarType result) const
                {
                    return result;
                }
            };

            template <
            class VectorType,
            template <typename> class StandardOperator
>            struct WeightedOperator
            {
                typedef typename VectorType::value_type scalar_type;

                WeightedOperator(const VectorType* scale) :
                _scale(scale)
                {}

                inline void operator()(scalar_type& result, scalar_type val, size_t i) const
                {
                    const scalar_type& x = (*_scale)[i];
                    _op(result, x * val);
                }

                inline scalar_type done(scalar_type result) const {return _op.done(result);}

            private:
                const VectorType* _scale;
                StandardOperator<scalar_type> _op;
            };

            template <class VectorType, template <typename> class StandardOperator>
            class StandardMetric : public Metric<VectorType>
            {
            public:
                typedef typename Metric<VectorType>::scalar_type scalar_type;
                typedef typename Metric<VectorType>::value_type value_type;
                typedef StandardOperator<scalar_type> operator_type;

                StandardMetric() {}

            protected:
                scalar_type doDistance(const value_type& q) const
                {
                    return _op.done(internal::accumulateNorm(q, _op));
                }

                scalar_type doDistance(const value_type& a, const value_type& b) const
                {
                    return _op.done(internal::accumulateDist(a, b, _op));
                }

                operator_type _op;
            };

            template <class VectorType, template <typename> class StandardOperator>
            class WeightedMetric : public Metric<VectorType>
            {
            public:
                typedef typename Metric<VectorType>::scalar_type scalar_type;
                typedef typename Metric<VectorType>::value_type value_type;

                typedef internal::WeightedOperator<
                value_type, StandardOperator> operator_type;

                WeightedMetric(const value_type& weights) :
                _weights(weights),
                _op(&_weights)
                {}

            protected:
                scalar_type doDistance(const value_type& q) const
                {
                    return _op.done(internal::accumulateNormWeighted(q, _op));
                }

                scalar_type doDistance(const value_type& a, const value_type& b) const
                {
                    return _op.done(internal::accumulateDistWeighted(a, b, _op));
                }

                int doSize() const {return (int)_weights.size();}

                value_type _weights;
                operator_type _op;
            };

        } //end internal namespace

		template <class T>
		class Rotation3DAngleMetric: public Metric<rw::math::Rotation3D<T> > {
		protected:
				T doDistance(const rw::math::Rotation3D<T>& r) const
                {
					EAA<T> eaa(r);
					return eaa.angle();
                }

                T doDistance(const rw::math::Rotation3D<T>& a, const rw::math::Rotation3D<T>& b) const
                {
					return doDistance(a*inverse(b));
                }
		};


		
		template <class T>
		class Transform3DAngleMetric: public Metric<rw::math::Transform3D<T> > {
		public:
			Transform3DAngleMetric(T posWeight, T angWeight):
			  _posWeight(posWeight),
			  _angWeight(angWeight)
			{}

		protected:
				T doDistance(const rw::math::Transform3D<T>& t) const
                {
					EAA<T> eaa(t.R());
					const T ang = eaa.angle();
					const T pos = t.P().norm2();
					return pos*_posWeight + ang*_angWeight;

                }

                T doDistance(const rw::math::Transform3D<T>& a, const rw::math::Transform3D<T>& b) const
                {					
					return doDistance(a*inverse(b));
                }
		private:
			T _posWeight;
			T _angWeight;
		};

}} // end namespaces

#endif // end include guard

