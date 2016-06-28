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

#include "Line2DPolar.hpp"
#include <rw/common/macros.hpp>
#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::math;

Line2DPolar::Line2DPolar(
    double rho,
    double theta)
    :
    _rho(rho),
    _theta(theta)
{}


Line2DPolar::Line2DPolar(const Vector2D<>& pnt, double theta):_theta(theta)
{
    const Vector2D<> normal(cos(theta), sin(theta));
	_rho = dot(pnt, normal);
    if (_rho < 0) {
        // Normalize rho and theta.
        const Vector2D<> inv_normal = -normal;
        _rho = -_rho;
        _theta = atan2(inv_normal[1], inv_normal[0]);
    }
}

Line2DPolar::Line2DPolar(const Vector2D<>& start, const Vector2D<>& end)
{
    const Vector2D<> diff = end - start;

    RW_ASSERT(diff.norm2() > 1e-14);

    const Vector2D<> step = diff / diff.norm2();

    const Vector2D<> normal(-step[1], step[0]);
    _theta = atan2(normal[1], normal[0]);
    _rho = dot(start, normal);
    if (_rho < 0) {
        // Normalize rho and theta.
        const Vector2D<> inv_normal = -normal;
        _rho = -_rho;
        _theta = atan2(inv_normal[1], inv_normal[0]);
    }
}

Line2DPolar::Line2DPolar(const Line2D& line)
{
    const Vector2D<> diff = line.p2() - line.p1();

    RW_ASSERT(diff.norm2() > 1e-14);

    const Vector2D<> step = diff / diff.norm2();

    const Vector2D<> normal(-step[1], step[0]);
    _theta = atan2(normal[1], normal[0]);
    _rho = dot(line.p1(), normal);
    if (_rho < 0) {
        // Normalize rho and theta.
        const Vector2D<> inv_normal = -normal;
        _rho = -_rho;
        _theta = atan2(inv_normal[1], inv_normal[0]);
    }
}



double Line2DPolar::dist2(const Vector2D<>& pnt)
{
	return fabs(dot(pnt, calcNormal()) - _rho);
}

Vector2D<> Line2DPolar::calcNormal() const{
	return Vector2D<>(cos(_theta), sin(_theta));
}

Vector2D<> Line2DPolar::linePoint(const Line2DPolar& line)
{
    return line.getRho() * line.calcNormal();
}

Vector2D<> Line2DPolar::normalProjectionVector(const Line2DPolar& line, const Vector2D<>& pnt)
{
    const Vector2D<> p0 = linePoint(line);
    const Vector2D<> normal = line.calcNormal();
    return dot(pnt - p0, normal) * normal;
}

Vector2D<> Line2DPolar::projectionPoint(const Line2DPolar& line, const Vector2D<>& pnt)
{
    return pnt - normalProjectionVector(line, pnt);
}

Line2DPolar Line2DPolar::lineToLocal(
    const Pose2D<>& pose,
    const Line2DPolar& line)
{
    const double ami = line.getTheta();
    const double rho = line.getRho();
    const double x = pose.x();
    const double y = pose.y();
    const double r = sqrt(x*x + y*y);
    const double beta = atan2(y,x);

    const double rho_local = rho - r * cos(ami - beta);
    const double angle_local = ami - pose.theta();
    return Line2DPolar(rho_local, angle_local);
}

template<>
void rw::common::serialization::write(const Line2DPolar& tmp, rw::common::OutputArchive& oar, const std::string& id)
{
    oar.writeEnterScope(id);
    oar.write( tmp.getRho(), "rho" );
    oar.write( tmp.getTheta() , "theta" );
    oar.writeLeaveScope(id);
}

template<>
void rw::common::serialization::read(Line2DPolar& tmp, rw::common::InputArchive& iar, const std::string& id){
    double rho,theta;
    iar.readEnterScope(id);
    iar.read(rho, "rho");
    iar.read(theta, "theta");
    iar.readLeaveScope(id);
    tmp = Line2DPolar(rho, theta);
}
