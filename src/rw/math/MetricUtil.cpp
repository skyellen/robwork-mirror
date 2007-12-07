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
    inline double square(double x)
    {
        return x * x;
    }
}

// Vectors

// 2-norm.

double MetricUtil::EuclideanLength(const Vector& vec)
{
    return sqrt(EuclideanLengthSquared(vec));
}

double MetricUtil::EuclideanLengthSquared(const Vector& vec)
{
    return
        square(vec[0]) +
        square(vec[1]) +
        square(vec[2]);
}

double MetricUtil::EuclideanDistance(const Vector& a, const Vector& b)
{
    return sqrt(EuclideanDistanceSquared(a, b));
}

double MetricUtil::EuclideanDistanceSquared(const Vector& a, const Vector& b)
{
    return
        square(a[0] - b[0]) +
        square(a[1] - b[1]) +
        square(a[2] - b[2]);
}

// Qs

namespace
{
    template <class Operator>
    double accumulateLength(const Q& q, Operator op)
    {
        double result = 0;
        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            result = op(result, q[i]);
        }
        return result;
    }

    template <class Operator>
    double accumulateLengthScaled(const Q& q, Operator op)
    {
        double result = 0;
        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            result = op(result, q[i], i);
        }
        return result;
    }

    template <class Operator>
    double accumulateDistance(
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
    double accumulateDistanceScaled(
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
            return result + square(val);
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

double MetricUtil::ManhattanLength(const Q& q)
{
    return accumulateLength(q, ManhattanOperator());
}

double MetricUtil::ManhattanDistance(const Q& a, const Q& b)
{
    return accumulateDistance(a, b, ManhattanOperator());
}

//----------------------------------------------------------------------
// 2-norm.

double MetricUtil::EuclideanLength(const Q& q)
{
    return sqrt(EuclideanLengthSquared(q));
}

double MetricUtil::EuclideanLengthSquared(const Q& q)
{
    return accumulateLength(q, EuclideanOperator());
}

double MetricUtil::EuclideanDistance(const Q& a, const Q& b)
{
    return sqrt(EuclideanDistanceSquared(a, b));
}

double MetricUtil::EuclideanDistanceSquared(const Q& a, const Q& b)
{
    return accumulateDistance(a, b, EuclideanOperator());
}

//----------------------------------------------------------------------
// infinity norm.

double MetricUtil::MaxLength(const Q& q)
{
    return accumulateLength(q, MaxOperator());
}

double MetricUtil::MaxDistance(const Q& a, const Q& b)
{
    return accumulateDistance(a, b, MaxOperator());
}

double MetricUtil::MaxLengthScaled(const Q& q, const Q& scale)
{
    return accumulateLengthScaled(q, MaxOperatorScaled(&scale));
}

double MetricUtil::MaxDistanceScaled(
    const Q& a, const Q& b, const Q& scale)
{
    return accumulateDistanceScaled(a, b, MaxOperatorScaled(&scale));
}

