/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "QuadraticUtil.hpp"
#include "QuadraticCurve.hpp"
#include "QuadraticSurface.hpp"

#include <rw/math/Polynomial.hpp>
#include <rw/math/PolynomialND.hpp>
#include <rw/math/PolynomialSolver.hpp>

#include <cmath>

using namespace rw::geometry;
using namespace rw::math;

bool QuadraticUtil::closestPointsApproximation(const QuadraticSurface& s1, const QuadraticSurface& s2, PointPair& result, double epsIn, double epsOut) {
	const QuadraticSurface::Ptr s1normalized = s1.normalize();
	const QuadraticSurface::Ptr s2normalized = s2.normalize();
	const QuadraticSurface& sA = *s1normalized;
	const QuadraticSurface& sB = *s2normalized;
	const Eigen::Matrix3d adjA = adjugate(sA.A());
	const Eigen::Matrix3d adjB = adjugate(sB.A());
	const Eigen::Matrix3d AB = sA.A()*sB.A();

	Polynomial<> detC(3);
	detC[3] = sA.determinantA();
	detC[2] = (sB.A()*adjA).trace();
	detC[1] = (sA.A()*adjB).trace();
	detC[0] = sB.determinantA();

	PolynomialND<Eigen::Matrix3d> adjC(2);
	adjC[2] = adjA;
	adjC[1] = (sB.A().trace()*Eigen::Matrix3d::Identity()-sB.A())*(sA.A().trace()*Eigen::Matrix3d::Identity()-sA.A());
	adjC[1] -= (AB).trace()*Eigen::Matrix3d::Identity()-AB;
	adjC[0] = adjB;

	PolynomialND<Eigen::Vector3d> c(1);
	PolynomialND<Eigen::RowVector3d> cT(1);
	c[1] = sA.a();
	c[0] = sB.a();
	cT[1] = sA.a().transpose();
	cT[0] = sB.a().transpose();

	const Polynomial<> phiF = cT*adjC*sA.A()*adjC*c - 2*detC*cT[1]*adjC*c + detC*detC*sA.u();
	const Polynomial<> phiG = cT*adjC*sB.A()*adjC*c - 2*detC*cT[0]*adjC*c + detC*detC*sB.u();

	const Polynomial<> optFct = phiF*phiF+phiG*phiG;
	const Polynomial<> der = optFct.derivative();

	PolynomialSolver solver(der);
	const std::vector<double> sols = solver.getRealSolutions();
	if (sols.size() > 0) {
		double min = optFct.evaluate(sols[0]);
		std::size_t minI = 0;
		for (std::size_t i = 1; i < sols.size(); i++) {
			const double eval = optFct.evaluate(sols[i]);
			if (eval < min) {
				min = eval;
				minI = i;
			}
		}

		const Vector3D<> z(-adjC.evaluate(sols[minI])*c.evaluate(sols[minI])/detC.evaluate(sols[minI]));
		const double dfz = sA(z)/sA.gradient(z).norm2();
		const double dgz = sB(z)/sB.gradient(z).norm2();
		const double distance = dfz+dgz;
		if (distance >= -epsIn && distance <= epsOut) {
			result.first = z-dfz*normalize(sA.gradient(z));
			result.second = z-dgz*normalize(sB.gradient(z));
			if (s1.insideTrimmingRegion(result.first) && s2.insideTrimmingRegion(result.second))
				return true;
		} else {
			return false;
		}
	}

	return false;
}

bool QuadraticUtil::closestPointsApproximation(const QuadraticSurface& s, const QuadraticCurve& c, PointPair& result, double epsIn, double epsOut) {
	const Eigen::Matrix3d& A = s.A();
	const Eigen::Vector3d& a = s.a();
	const Eigen::Vector3d u = c.u().e();
	const Eigen::Vector3d v = c.v().e();
	const Eigen::Vector3d cc = c.c().e();

	Polynomial<> pol(0);
	Eigen::Vector3d K1;
	Eigen::Vector3d K2;
	switch(c.type()) {
	case QuadraticCurve::Elliptic:
		pol = Polynomial<>(4);
		K1 = A*(cc-v)+a;
		K2 = A*(cc+v)+a;
		pol[4] = -u.transpose()*K1;
		pol[3] = (-v.transpose()*K1-u.transpose()*A*u)[0]*2;
		pol[2] = (-u.transpose()*(A*v))[0]*6;
		pol[1] = (v.transpose()*K2+u.transpose()*(A*u))[0]*2;
		pol[0] = u.transpose()*K2;
		break;
	case QuadraticCurve::Hyperbola:
		pol = Polynomial<>(4);
		K1 = A*(v-cc)-a;
		K2 = A*(v+cc)+a;
		pol[4] = u.transpose()*K1;
		pol[3] = (v.transpose()*K1+u.transpose()*(A*u))[0]*2;
		pol[2] = 6.*u.transpose()*(A*v);
		pol[1] = (v.transpose()*K2+u.transpose()*(A*u))[0]*2;
		pol[0] = u.transpose()*K2;
		break;
	case QuadraticCurve::Line:
		pol = Polynomial<>(1);
		pol[1] = u.transpose()*(A*u);
		pol[0] = (A*cc+a).transpose()*u;
		break;
	case QuadraticCurve::Parabola:
		pol = Polynomial<>(3);
		K1 = A*cc+a;
		pol[3] = 2.*v.transpose()*(A*v);
		pol[2] = 3.*u.transpose()*(A*v);
		pol[1] = 2.*v.transpose()*K1;
		pol[1] += u.transpose()*(A*u);
		pol[0] = u.transpose()*K1;
	}
	PolynomialSolver solver(pol);
	std::vector<double> sols = solver.getRealSolutions();
	std::vector<double> minSols;
	if (sols.size() > 0) {
		for (std::size_t i = 0; i < sols.size(); i++) {
			if (pol.evaluateDerivatives(sols[i],1)[1] > 0) {
				minSols.push_back(sols[i]);
			}
		}
	}
	if (c.type() == QuadraticCurve::Elliptic) {
		for (std::size_t i = 0; i < minSols.size(); i++)
			minSols[i] = std::atan(minSols[i])*2;
	} else if (c.type() == QuadraticCurve::Hyperbola) {
		for (std::size_t i = 0; i < minSols.size(); i++)
			minSols[i] = std::atanh(minSols[i])*2;
	}


	if (minSols.size() > 0) {
		double min = pol.evaluate(minSols[0]);
		std::size_t minI = 0;
		for (std::size_t i = 1; i < minSols.size(); i++) {
			const double eval = pol.evaluate(minSols[i]);
			if (eval < min) {
				min = eval;
				minI = i;
			}
		}

		const Vector3D<> z = c(minSols[minI]);
		const double dfz = s(z)/s.gradient(z).norm2();
		if (dfz >= -epsIn && dfz <= epsOut) {
			result.first = z-dfz*normalize(s.gradient(z));
			result.second = z;
			if (s.insideTrimmingRegion(result.first) && c.inLimits(min))
				return true;
		} else {
			return false;
		}
	}

	return false;
}

std::vector<double> QuadraticUtil::closestTimesApproximation(const QuadraticSurface& sa, const QuadraticSurface& sb, const QuadraticCurve& c) {
	const Eigen::Matrix3d& Aa = sa.A();
	const Eigen::Vector3d& aa = sa.a();
	const double ua = sa.u();
	const Eigen::Matrix3d& Ab = sb.A();
	const Eigen::Vector3d& ab = sb.a();
	const double ub = sb.u();
	const Eigen::Vector3d u = c.u().e();
	const Eigen::Vector3d v = c.v().e();
	const Eigen::Vector3d cc = c.c().e();

	const double A1 = u.transpose()*Aa*u;
	const double A2 = v.transpose()*Aa*v;
	const double A3 = 2.*u.transpose()*Aa*v;
	const double A4 = 2.*(cc.transpose()*Aa+aa.transpose())*u;
	const double A5 = 2.*(cc.transpose()*Aa+aa.transpose())*v;
	const double A6 = (cc.transpose()*Aa+2.*aa.transpose())*cc+ua;

	const double B1 = u.transpose()*Ab*u;
	const double B2 = v.transpose()*Ab*v;
	const double B3 = 2.*u.transpose()*Ab*v;
	const double B4 = 2.*(cc.transpose()*Ab+ab.transpose())*u;
	const double B5 = 2.*(cc.transpose()*Ab+ab.transpose())*v;
	const double B6 = (cc.transpose()*Ab+2.*ab.transpose())*cc+ub;

	// Constants that are used in all cases:
	const double m11 = A1*A1 + B1*B1;
	const double m14 = A1*A4 + B1*B4;
	const double m16 = A1*A6 + B1*B6;
	const double m44 = A4*A4 + B4*B4;
	const double m46 = A4*A6 + B4*B6;

	Polynomial<> pol(0);
	if (c.type() == QuadraticCurve::Elliptic || c.type() == QuadraticCurve::Hyperbola) {
		pol = Polynomial<>(7);

		const double m22 = A2*A2 + B2*B2;
		const double m33 = A3*A3 + B3*B3;
		const double m55 = A5*A5 + B5*B5;
		const double m66 = A6*A6 + B6*B6;

		const double m12 = A1*A2 + B1*B2;
		const double m13 = A1*A3 + B1*B3;
		const double m15 = A1*A5 + B1*B5;
		const double m23 = A2*A3 + B2*B3;
		const double m24 = A2*A4 + B2*B4;
		const double m25 = A2*A5 + B2*B5;
		const double m26 = A2*A6 + B2*B6;
		const double m34 = A3*A4 + B3*B4;
		const double m35 = A3*A5 + B3*B5;
		const double m36 = A3*A6 + B3*B6;
		const double m45 = A4*A5 + B4*B5;
		const double m46 = A4*A6 + B4*B6;
		const double m56 = A5*A6 + B5*B6;

		// Basic polynomial with common terms for ellipse and hyperbola
		const double mA = m23 + m45 + m36;
		const double mB = m24 + m35 + m46;
		const double mCp = m22 + m66;
		const double mCm = m22 - m66;
		const double mE = m25 - m56;
		const double mF = m34 + m15;
		const double mG = m12 + m16;
		const double mH = m24 + m35;
		const double mI = m45 + m36;

		pol[7] = (mCp + m55)*8 + (- m25 + m26 - m56)*16;
		pol[6] = 0;
		pol[5] = (mF-mG)*48 + (m33)*24;
		pol[4] = (m23 + m46)*60 - (mH + mI)*20;
		pol[3] = m11*64 + (mCp)*24 + (-m55)*8 + (-m26)*16;
		pol[2] = (m13 + m14)*48;
		pol[1] = (mG+mF)*16 + (m33 + m44)*8;
		pol[0] = (mA+mB)*4;

		// Specific terms
		const double K6 = (mA - mB)*28;
		const double K5 = (mCm - m44 - mE)*24;
		const double K4 = (m13 - m14)*80;
		const double K3 = (m12 - m16)*64 + (m33 - m44)*32;
		const double K2 = (m23 - m46)*36 + (mH-mI)*12;
		const double K1 = (mCm + mE)*8;

		if (c.type() == QuadraticCurve::Elliptic) {
			pol[6] -= K6;
			pol[5] -= K5;
			pol[4] -= K4;
			pol[3] -= K3;
			pol[2] -= K2;
			pol[1] -= K1;
		} else if (c.type() == QuadraticCurve::Hyperbola) {
			pol[6] += K6;
			pol[5] += K5;
			pol[4] += K4;
			pol[3] += K3;
			pol[2] += K2;
			pol[1] += K1;
		}
	} else if (c.type() == QuadraticCurve::Line) {
		pol = Polynomial<>(3);
		pol[3] = m11*2;
		pol[2] = m14*3;
		pol[1] = m44 + m16*2;
		pol[0] = m46;
	} else if (c.type() == QuadraticCurve::Parabola) {
		pol = Polynomial<>(7);
		pol[7] = (A2*A2 + B2*B2)*4;
		pol[6] = (A2*A3 + B2*B3)*7;
		pol[5] = (A1*A2 + B1*B2 + A2*A5 + B2*B5)*6 + (A3*A3 + B3*B3)*3;
		pol[4] = (A1*A3 + B1*B3 + A2*A4 + B2*B4 + A3*A5 + B3*B5)*5;
		pol[3] = (m11 + A5*A5 + B5*B5)*2 + (A3*A4 + B3*B4 + A1*A5 + B1*B5 + A2*A6 + B2*B6)*4;
		pol[2] = (m14 + A4*A5 + B4*B5 + A3*A6 + B3*B6)*3;
		pol[1] = m44 + (m16 + A5*A6 + B5*B6)*2;
		pol[0] = m46;
	}
	PolynomialSolver solver(pol);
	std::vector<double> sols = solver.getRealSolutions();
	std::vector<double> minSols;
	if (sols.size() > 0) {
		for (std::size_t i = 0; i < sols.size(); i++) {
			if (pol.evaluateDerivatives(sols[i],1)[1] > 0) {
				minSols.push_back(sols[i]);
			} else {
			}
		}
	}
	if (c.type() == QuadraticCurve::Elliptic) {
		for (std::size_t i = 0; i < minSols.size(); i++)
			minSols[i] = std::atan(minSols[i])*2;
	} else if (c.type() == QuadraticCurve::Hyperbola) {
		for (std::size_t i = 0; i < minSols.size(); i++)
			minSols[i] = std::atanh(minSols[i])*2;
	}
	return minSols;
}

std::vector<QuadraticUtil::PointPair> QuadraticUtil::closestPointsApproximation(const QuadraticSurface& sa, const QuadraticSurface& sb, const QuadraticCurve& c, double epsIn, double epsOut) {
	const std::vector<double> times = closestTimesApproximation(sa, sb, c);

	std::vector<PointPair> result;
	for (std::size_t i = 0; i < times.size(); i++) {
		if (!c.inLimits(times[i]))
			continue;
		const Vector3D<> z = c(times[i]);
		const double dfz = sa(z)/sa.gradient(z).norm2();
		const double dgz = sb(z)/sb.gradient(z).norm2();
		const double distance = dfz-dgz;
		if (distance >= -epsIn && distance <= epsOut) {
			PointPair points;
			points.first = z-dfz*normalize(sa.gradient(z));
			points.second = z;
			result.push_back(points);
		}
	}
	return result;
}

std::vector<QuadraticUtil::PointPair> QuadraticUtil::closestPointsApproximation(const QuadraticSurface& sa1, const QuadraticSurface& sa2, const QuadraticCurve& ca, const QuadraticCurve& cb, double epsIn, double epsOut) {
	std::vector<QuadraticUtil::PointPair> closest = closestPointsApproximation(sa1,sa2,cb,epsIn,epsOut);
	std::vector<QuadraticUtil::PointPair> result;
	for (std::size_t i = 0; i < closest.size(); i++) {
		const std::vector<Vector3D<> > points = ca.closestPoints(closest[i].second);
		for (std::size_t k = 0; k < points.size(); k++) {
			result.push_back(std::make_pair(points[k],closest[i].second));
		}
	}
	return result;
}

std::vector<std::pair<double,double> > QuadraticUtil::closestTimesApproximation(const QuadraticSurface& sa1, const QuadraticSurface& sa2, const QuadraticCurve& ca, const QuadraticCurve& cb) {
	std::vector<double> closest = closestTimesApproximation(sa1,sa2,cb);
	std::vector<std::pair<double,double> > result;
	for (std::size_t i = 0; i < closest.size(); i++) {
		if (!cb.inLimits(closest[i]))
			continue;
		const std::vector<double> times = ca.closestTimes(cb(closest[i]));
		for (std::size_t k = 0; k < times.size(); k++) {
			if (!ca.inLimits(times[k]))
				continue;
			result.push_back(std::make_pair(times[k],closest[i]));
		}
	}
	return result;
}

Eigen::Matrix3d QuadraticUtil::adjugate(const Eigen::Matrix3d& A) {
	// Notice: A is symmetric, so we only need to calculate triangular cofactor matrix. Also there is no need to do the transpose.
	Eigen::Matrix3d res;
	res(0,0) = A.block(1,1,2,2).determinant();
	res(0,1) = -A(1,0)*A(2,2)+A(1,2)*A(2,0);
	res(0,2) = A.block(0,1,2,2).determinant();
	res(1,1) = A(0,0)*A(2,2)-A(0,2)*A(2,0);
	res(1,2) = -A(0,0)*A(2,1)+A(0,1)*A(2,0);
	res(2,2) = A.block(0,0,2,2).determinant();
	// Fill in lower part
	res(1,0) = res(0,1);
	res(2,0) = res(0,2);
	res(2,1) = res(1,2);
	return res;
}
