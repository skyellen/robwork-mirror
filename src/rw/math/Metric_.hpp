// -*- c++ -*-

#ifndef RW_MATH_METRIC_INTERNAL_HPP
#define RW_MATH_METRIC_INTERNAL_HPP

#include "Math.hpp"
#include <vector>

namespace rw { namespace math { namespace internal { 

    template <
        class Operator,
        class VectorType
        >
    inline
    typename VectorType::value_type accumulateNorm(const VectorType& q, Operator op)
    {
        typename VectorType::value_type result = 0;
        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            result = op(result, q[i]);
        }
        return result;
    }

    template <
        class VectorType,
        class Operator
        >
    inline
    typename VectorType::value_type accumulateNormWeighted(const VectorType& q, Operator op)
    {
        typename VectorType::value_type result = 0;
        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            result = op(result, q[i], i);
        }
        return result;
    }

    template <
        class VectorType,
        class Operator
        >
    inline
    typename VectorType::value_type accumulateDist(
        const VectorType& a,
        const VectorType& b,
        Operator op)
    {
        typename VectorType::value_type result = 0;
        const int len = (int)a.size();
        for (int i = 0; i < len; i++) {
            result = op(result, a[i] - b[i]);
        }
        return result;
    }

    template <
        class VectorType,
        class Operator
        >
    inline
    typename VectorType::value_type accumulateDistWeighted(
        const VectorType& a,
        const VectorType& b,
        Operator op)
    {
        typename VectorType::value_type result = 0;
        const int len = (int)a.size();
        for (int i = 0; i < len; i++) {
            result = op(result, a[i] - b[i], i);
        }
        return result;
    }

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
        ScalarType operator()(ScalarType result, ScalarType val) const
        {
            return result + fabs(val);
        }

        ScalarType done(ScalarType result) const
        {
            return result;
        }
    };

    template <class ScalarType>
    struct EuclideanOperator
    {
        ScalarType operator()(ScalarType result, ScalarType val)
        {
            return result + Math::sqr(val);
        }

        ScalarType done(ScalarType result) const
        {
            return sqrt(result);
        }
    };

    template <class ScalarType>
    struct InfinityOperator
    {
        ScalarType operator()(ScalarType result, ScalarType val)
        {
            return std::max(result, fabs(val));
        }

        ScalarType done(ScalarType result) const
        {
            return result;
        }
    };

    template <
        class VectorType,
        template <typename> class StandardOperator
    >
    struct WeightedOperator
    {
        typedef typename VectorType::value_type scalar_type;

        WeightedOperator(const VectorType* scale) :
            _scale(scale)
        {}

        scalar_type operator()(scalar_type result, scalar_type val, int i)
        {
            const scalar_type& x = (*_scale)[i];
            return _op(result, x * val);
        }

        scalar_type done(scalar_type result) const { return _op.done(result); }

    private:
        const VectorType* _scale;
        StandardOperator<scalar_type> _op;
    };

    template <class VectorType, template <typename> class StandardOperator>
    class StandardMetric : public Metric<VectorType>
    {
    public:
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
        typedef internal::WeightedOperator<value_type, StandardOperator> operator_type;

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

        int doSize() const { return _weights.size(); }

        value_type _weights;
        operator_type _op;
    };

}}} // end namespaces

#endif // end include guard
