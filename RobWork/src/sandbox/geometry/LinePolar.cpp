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

#include "LinePolar.hpp"
#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>
#include <boost/foreach.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <cstdio>
#include <iostream>



using namespace rw::geometry;
using namespace rw::math;
using namespace boost::numeric::ublas;


LinePolar::LinePolar(
    double rho,
    double theta)
    :
    _rho(rho),
    _theta(theta),
    _normal(cos(theta), sin(theta))
{}


LinePolar LinePolar::make(const Vector2D<>& pnt, double theta)
{
    const rw::math::Vector2D<> normal(cos(theta), sin(theta));
    const double rho = dot(pnt, normal);

    if (rho >= 0) {
        return LinePolar(rho, theta);
    } else {
        // Normalize rho and theta.
        const Vector2D<> inv_normal = -normal;
        return LinePolar(
            -rho,
            atan2(inv_normal[1], inv_normal[0]));
    }
}

LinePolar LinePolar::make(const Vector2D<>& start, const Vector2D<>& end)
{
    const Vector2D<> diff = end - start;
	if(diff.norm2() < 1e-14)
		RW_THROW("To close points");

    const Vector2D<> step = diff / diff.norm2();

    const Vector2D<> normal(-step[1], step[0]);
    const double theta = atan2(normal[1], normal[0]);
    return LinePolar::make(start, theta);
}

LinePolar LinePolar::make(const std::vector< rw::math::Vector2D<> >& points){
    if(points.size()<2)
        RW_THROW("Not enough points!");

    return make(points[0],points[1]);
}

double LinePolar::dist2(const Vector2D<>& pnt)
{
	return fabs(dot(pnt, getNormal()) - _rho);
}

Vector2D<> LinePolar::linePoint(const LinePolar& line)
{
    return line.getRho() * line.getNormal();
}

Vector2D<> LinePolar::normalProjectionVector(const LinePolar& line, const Vector2D<>& pnt)
{
    const Vector2D<> p0 = linePoint(line);

    return dot(pnt - p0, line.getNormal()) * line.getNormal();
}

Vector2D<> LinePolar::projectionPoint(const LinePolar& line, const Vector2D<>& pnt)
{
    return pnt - normalProjectionVector(line, pnt);
}

void LinePolar::print(const LinePolar& line)
{
    printf(
        "(%.2f m, %.2f Deg)",
        line.getRho(),
        line.getTheta() * rw::math::Rad2Deg);
}

typedef boost::numeric::ublas::vector<double> Vec;
Vec LinePolar::toUblas(const LinePolar& line)
{
    Vec vec(2);
    vec(0) = line.getRho();
    vec(1) = line.getTheta();
    return vec;
}

LinePolar LinePolar::lineToLocal(
    const Pose2D& pose,
    const LinePolar& line)
{
    const double ami = line.getTheta();
    const double rho = line.getRho();
    const double x = pose.x();
    const double y = pose.y();
    const double r = sqrt(x*x + y*y);
    const double beta = atan2(y,x);

    const double rho_local = rho - r * cos(ami - beta);
    const double angle_local = ami - pose.theta();
    return LinePolar(rho_local, angle_local);
}

namespace
{
    typedef rw::geometry::LinePolar::const_iterator I;
    typedef std::pair<I, I> R;

    Vector2D<> sqr(const Vector2D<>& v)
    {
        return Vector2D<>(v[0] * v[0], v[1] * v[1]);
    }

    Vector2D<> average(R pnts)
    {
        int cnt = 0;
        Vector2D<> sum(0, 0);
        BOOST_FOREACH(const Vector2D<>& pnt, pnts) {
            sum += pnt;
            ++cnt;
        }

        RW_ASSERT(cnt > 0);
        return sum / cnt;
    }

    Vector2D<> sumPP(R pnts, const Vector2D<>& avg)
    {
        Vector2D<> sum(0, 0);
        BOOST_FOREACH(const Vector2D<>& pnt, pnts) {
            sum += sqr(pnt - avg);
        }
        return sum;
    }

    double sumXY(R pnts, const Vector2D<>& avg)
    {
        double sum = 0;
        BOOST_FOREACH(const Vector2D<>& pnt, pnts) {
            const Vector2D<> diff = pnt - avg;
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

//TODO
void makeA(R pnts,int degress, matrix<double> &A)
{
	std::vector<int> tmp;
	BOOST_FOREACH(const Vector2D<>& pnt, pnts) {
		tmp.push_back(pnt(0));
	}
	for (unsigned int i = 0; i < A.size1 (); ++ i){
		for (unsigned int j = 0; j < A.size2 (); ++ j){

			if(j == 0)
				A (i, j) = 1;
			else if(j <= (unsigned int)degress) 
				A (i, j) = pow((double)tmp[i],(int)j);
			else
				A (i, j) = 0.0;
		}
	}
}

void makeY(R pnts,matrix<double> &y)
{
	std::vector<int> tmp;
	BOOST_FOREACH(const Vector2D<>& pnt, pnts) {
		tmp.push_back(pnt(1));
	}
	for (unsigned int i = 0; i < y.size1 (); ++ i){
		y(i ,0) = tmp[i];
	}
}

bool solveLS(matrix<double> &A,matrix<double> &y,matrix<double> &x)
{
	x = prod(LinearAlgebra::pseudoInverse(A),y);
	if(fabs(x(0,0)) < 10E-7)
		x(0,0) = 0;
	if(fabs(x(1,0)) < 10E-7)
		x(1,0) = 0;
	return true;
}
LinePolar LinePolar::fitSVD(R pnts){
	
	int cnt = -1;
    BOOST_FOREACH(const Vector2D<>& pnt, pnts) {
        cnt++;
    }

	matrix<double> A (cnt, 2);
	makeA(pnts,1,A);
	
	matrix<double> y(cnt,1);
	makeY(pnts,y); 

	matrix<double> x;
	
	solveLS(A,y,x);
	
//	for(int i = 0;i < x.size1(); i++)
//		for(int j = 0;j < x.size2(); j++)
//			std::cout << "matrix "<<i << ","<<j<<" -> "<<x(i,j) << std::endl; 
	if(x.size1() == 2){
		Vector2D<> p1(100.0,x(1,0)*100.0+ x(0,0));
		Vector2D<> p2(500.0,x(1,0)*500.0+ x(0,0));
		return LinePolar::make(p1,p2);
	}
	else{
		std::cout <<"Error in fitSVD " << std::endl;
		return LinePolar::make(Vector2D<>(0,0), 0);
	}
}


LinePolar LinePolar::fit(R pnts)
{
	

    const Vector2D<> avg = average(pnts);
    const Vector2D<> S_xx_yy = sumPP(pnts, avg);

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
    //RW_ASSERT(fabs(df(S_xx, S_yy, S_xy, theta1)) < 1e-8);
    //RW_ASSERT(fabs(df(S_xx, S_yy, S_xy, theta2)) < 1e-8);
	if( (fabs(df(S_xx, S_yy, S_xy, theta1)) > 1e-6) && (fabs(df(S_xx, S_yy, S_xy, theta2)) > 1e-6) ){
		RW_THROW("Something is wrong");
	} else if( fabs(df(S_xx, S_yy, S_xy, theta1)) > 1e-6 ) {
		return LinePolar::make(avg, theta2);
	} else if( fabs(df(S_xx, S_yy, S_xy, theta2)) > 1e-6 ) {
		return LinePolar::make(avg, theta1);
	}


    // Insert theta1 and theta2 in the function to minimize ...
    //const double val1 = f(S_xx, S_yy, S_xy, theta1);
    //const double val2 = f(S_xx, S_yy, S_xy, theta1);

    // ... and select the theta that gave the smallest value of f():
    //const double theta = val1 < val2 ? theta1 : theta2;

    // Construct a polar point from a point on the line and the angle.
	//std::cout << "return fitSVD " << std::endl;
	return fitSVD(pnts);
//	std::cout << "LinePolar::fitSVD " << LinePolar::linePoint(ppp) << " theta " << ppp.getTheta() << std::endl;
//	std::cout << "LinePolar::fit " << avg << " theta " << theta << std::endl;
//    return LinePolar::make(avg, theta);
}

LinePolar LinePolar::fit(I a, I b)
{
    return fit(std::make_pair(a, b));
}

LinePolar LinePolar::fit(const std::vector<Vector2D<> >& pnts)
{
    return fit(std::make_pair(pnts.begin(), pnts.end()));
}

rw::math::Line2D LinePolar::toLine2D(){
    Vector2D<> p1 = _rho*_normal;
    Vector2D<> p2 = p1 + Rotation2D<>(90*Deg2Rad)*_normal*1000;
    return rw::math::Line2D(p1,p2);
}

