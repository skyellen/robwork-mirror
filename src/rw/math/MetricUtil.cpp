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

#include "MetricUtil.hpp"

#include <rw/common/macros.hpp>



using namespace rw::math;


typedef Vector3D<> Vector;

namespace
{
    /**
       @brief The value <code>x * x</code>.
    */
    inline double sqr(double x)
    {
        return x * x;
    }
}

namespace
{
    template <class Operator>
    double accumulateNorm(const Q& q, Operator op)
    {
        double result = 0;
        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            result = op(result, q[i]);
        }
        return result;
    }

    template <class Operator>
    double accumulateNormScaled(const Q& q, Operator op)
    {
        double result = 0;
        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            result = op(result, q[i], i);
        }
        return result;
    }

    template <class Operator>
    double accumulateDist(
        const Q& a,
        const Q& b,
        Operator op)
    {
        RW_ASSERT(a.size() == b.size());

        double result = 0;
        const int len = (int)a.size();
        for (int i = 0; i < len; i++) {
            result = op(result, a[i] - b[i]);
        }
        return result;
    }

    template <class Operator>
    double accumulateDistScaled(
        const Q& a,
        const Q& b,
        Operator op)
    {
        RW_ASSERT(a.size() == b.size());

        double result = 0;
        const int len = (int)a.size();
        for (int i = 0; i < len; i++) {
            result = op(result, a[i] - b[i], i);
        }
        return result;
    }

    struct ManhattanOperator
    {
        double operator()(double result, double val)
        {
            return result + fabs(val);
        }
    };

    struct EuclideanOperator
    {
        double operator()(double result, double val)
        {
            return result + sqr(val);
        }
    };

    struct MaxOperator
    {
        double operator()(double result, double val)
        {
            return std::max(result, fabs(val));
        }
    };

    struct MaxOperatorScaled
    {
        MaxOperatorScaled(const Q* scale) :
            _scale(scale)
        {}

        double operator()(double result, double val, int i)
        {
            const double x = (*_scale)[i];
            return std::max(result, fabs(x * val));
        }

    private:
        const Q* _scale;
    };

    struct MaxOperatorDivided
    {
        MaxOperatorDivided(const Q* scale) :
            _scale(scale)
        {}

        double operator()(double result, double val, int i)
        {
            const double x = (*_scale)[i];
            return std::max(result, fabs(val / x));
        }

    private:
        const Q* _scale;
    };
}

//----------------------------------------------------------------------
// 1-norm.

double MetricUtil::norm1(const Q& q)
{
    return accumulateNorm(q, ManhattanOperator());
}

double MetricUtil::dist1(const Q& a, const Q& b)
{
    return accumulateDist(a, b, ManhattanOperator());
}

//----------------------------------------------------------------------
// 2-norm.

double MetricUtil::norm2(const Q& q)
{
    return sqrt(norm2Sqr(q));
}

double MetricUtil::norm2Sqr(const Q& q)
{
    return accumulateNorm(q, EuclideanOperator());
}

double MetricUtil::dist2(const Q& a, const Q& b)
{
    return sqrt(dist2Sqr(a, b));
}

double MetricUtil::dist2Sqr(const Q& a, const Q& b)
{
    return accumulateDist(a, b, EuclideanOperator());
}

//----------------------------------------------------------------------
// infinity norm.

double MetricUtil::normInf(const Q& q)
{
    return accumulateNorm(q, MaxOperator());
}

double MetricUtil::distInf(const Q& a, const Q& b)
{
    return accumulateDist(a, b, MaxOperator());
}

double MetricUtil::normInfScaled(const Q& q, const Q& scale)
{
    return accumulateNormScaled(q, MaxOperatorScaled(&scale));
}

double MetricUtil::distInfScaled(
    const Q& a, const Q& b, const Q& scale)
{
    return accumulateDistScaled(a, b, MaxOperatorScaled(&scale));
}
