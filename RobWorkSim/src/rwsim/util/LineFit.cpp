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

#include "LineFit.hpp"
#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <boost/foreach.hpp>
#include <cmath>
using namespace robwork;

using namespace rw::math;

namespace
{
    typedef LineFit::const_iterator I;
    typedef std::pair<I, I> R;

    P2D sqr(const P2D& v)
    {
        return P2D(v[0] * v[0], v[1] * v[1]);
    }

    P2D average(R pnts)
    {
        int cnt = 0;
        P2D sum(0, 0);
        BOOST_FOREACH(const P2D& pnt, pnts) {
            sum += pnt;
            ++cnt;
        }

        RW_ASSERT(cnt > 0);
        return sum / cnt;
    }

    P2D sumPP(R pnts, const P2D& avg)
    {
        int cnt = 0;
        P2D sum(0, 0);
        BOOST_FOREACH(const P2D& pnt, pnts) {
            sum += sqr(pnt - avg);
        }
        return sum;
    }

    double sumXY(R pnts, const P2D& avg)
    {
        double sum = 0;
        BOOST_FOREACH(const P2D& pnt, pnts) {
            const P2D diff = pnt - avg;
            sum += diff[0] * diff[1];
        }
        return sum;
    }

    // The function to minimize.
    double f(double S_xx, double S_yy, double S_xy, double theta)
    {
        const double ct = cos(theta);
        const double st = sin(theta);
        return (ct * ct) * S_xx + (st * st) * S_yy + 2 * ct * st * S_xy;
    }

    // The derivative of f() wrt. theta.
    double df(double S_xx, double S_yy, double S_xy, double theta)
    {
        return
            2 * S_xy * cos(2 * theta) +
            (S_yy - S_xx) * sin(2 * theta);
    }
}

LinePolar LineFit::fit(R pnts)
{
    const P2D avg = average(pnts);
    const P2D S_xx_yy = sumPP(pnts, avg);

    const double S_xx = S_xx_yy[0];
    const double S_yy = S_xx_yy[1];
    const double S_xy = sumXY(pnts, avg);

    const double a = 2 * S_xy;
    const double b = S_yy - S_xx;

    // There are two solutions here:
    const double theta1 = 0.5 * atan2(a, -b);
    const double theta2 = 0.5 * atan2(-a, b);

    // Insert in the derivative of f() to check that those are indeed solutions
    // to df(theta) == 0.
    RW_ASSERT(fabs(df(S_xx, S_yy, S_xy, theta1)) < 1e-8);
    RW_ASSERT(fabs(df(S_xx, S_yy, S_xy, theta2)) < 1e-8);

    // Insert theta1 and theta2 in the function to minimize ...
    const double val1 = f(S_xx, S_yy, S_xy, theta1);
    const double val2 = f(S_xx, S_yy, S_xy, theta1);

    // ... and select the theta that gave the smallest value of f():
    const double theta = val1 < val2 ? theta1 : theta2;

    // Construct a polar point from a point on the line and the angle.
    return LinePolar::make(avg, theta);
}

LinePolar LineFit::fit(I a, I b)
{
    return fit(std::make_pair(a, b));
}

LinePolar LineFit::fit(const std::vector<P2D>& pnts)
{
    return fit(std::make_pair(pnts.begin(), pnts.end()));
}
