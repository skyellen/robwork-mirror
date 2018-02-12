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

#include "QuadraticCurve.hpp"

#include <rw/math/Constants.hpp>
#include <rw/math/Polynomial.hpp>
#include <rw/math/PolynomialSolver.hpp>

#include <limits>

using rw::common::ownedPtr;
using namespace rw::math;
using namespace rw::geometry;

QuadraticCurve::QuadraticCurve(const Vector3D<>& c, const Vector3D<>& u, const Vector3D<>& v, const Type& type):
	_c(c), _u(u), _v(v), _type(type),
	_hasLimits(false),
	_limits(std::make_pair(0,0))
{
}

QuadraticCurve::~QuadraticCurve() {
}

QuadraticCurve::Ptr QuadraticCurve::transform(const Transform3D<>& T) const {
	const QuadraticCurve::Ptr curve = ownedPtr(new QuadraticCurve(T*_c,T.R()*_u,T.R()*_v,_type));
	if (_hasLimits)
		curve->setLimits(_limits);
	return curve;
}

QuadraticCurve::Ptr QuadraticCurve::transform(const Vector3D<>& P) const {
	const QuadraticCurve::Ptr curve = ownedPtr(new QuadraticCurve(_c+P,_u,_v,_type));
	if (_hasLimits)
		curve->setLimits(_limits);
	return curve;
}

QuadraticCurve::Ptr QuadraticCurve::scale(double factor) const {
	const QuadraticCurve::Ptr curve = ownedPtr(new QuadraticCurve(_c*factor,_u*factor,_v*factor,_type));
	if (_hasLimits)
		curve->setLimits(_limits);
	return curve;
}

QuadraticCurve::Ptr QuadraticCurve::reverse() const {
	const QuadraticCurve::Ptr curve = ownedPtr(new QuadraticCurve(_c,-_u,_v,_type));
	if (_hasLimits)
		curve->setLimits(std::make_pair(-_limits.second,-_limits.first));
	return curve;
}

QuadraticCurve::Ptr QuadraticCurve::clone() const {
	const QuadraticCurve::Ptr curve = ownedPtr(new QuadraticCurve(_c,_u,_v,_type));
	if (_hasLimits)
		curve->setLimits(std::make_pair(_limits.first,_limits.second));
	return curve;
}

std::pair<double,double> QuadraticCurve::extremums(const Vector3D<>& dir) const {
	std::pair<double,double> result;
	double t;
	double t2;
	double tLim;
	double tMax;
	double tMin;
	const double eu = dot(dir,_u);
	const double ev = dot(dir,_v);
	switch(_type) {
	case Line:
		if (_hasLimits) {
			result.first = std::min(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
			result.second = std::max(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
		} else {
			result.first = -std::numeric_limits<double>::max();
			result.second = std::numeric_limits<double>::max();
		}
		break;
	case Elliptic:
		/*
		if (std::abs(eu) == 0) {
			t = 0;
			t2 = Pi;
		} else {
			t = -ev/eu+std::sqrt(ev*ev/eu/eu+1);
			t2 = -ev/eu-std::sqrt(ev*ev/eu/eu+1);
			t = std::atan(t)*2;
			t2 = std::atan(t2)*2;
		}*/
		if (std::abs(ev) == 0) {
			t = Pi/2;
			t2 = -Pi/2;
		} else {
			t = std::atan(eu/ev);
			t2 = (t >= 0)? t-Pi : t+Pi;
		}
		//std::cout << "Elliptic " << _c << " " << _u << " " << _v << " " << _limits.first << " " << _limits.second << " " << dir << " " << t << " " << t2 << std::endl;
		tMax = 100;
		tMin = -100;
		if ((t >= _limits.first && t <= _limits.second) || (t-2*Pi >= _limits.first && t-2*Pi <= _limits.second) || (t+2*Pi >= _limits.first && t+2*Pi <= _limits.second)) {
			if (-eu*std::sin(t)-ev*std::cos(t) < 0) {
				tMax = t;
			} else {
				tMin = t;
			}
		}
		if ((t2 >= _limits.first && t2 <= _limits.second) || (t2-2*Pi >= _limits.first && t2-2*Pi <= _limits.second) || (t2+2*Pi >= _limits.first && t2+2*Pi <= _limits.second)) {
			if (-eu*std::sin(t2)-ev*std::cos(t2) < 0) {
				tMax = t2;
			} else {
				tMin = t2;
			}
		}
		//std::cout << " tmin tmax: " << tMin << " " << tMax << std::endl;
		if (tMax > 5) {
			if (_hasLimits)
				result.second = std::max(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
			else
				result.second = std::numeric_limits<double>::max();
		} else {
			result.second = dot(dir,(*this)(tMax));
		}
		if (tMin < -5) {
			if (_hasLimits)
				result.first = std::min(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
			else
				result.first = -std::numeric_limits<double>::max();
		} else {
			result.first = dot(dir,(*this)(tMin));
		}
		break;
	case Hyperbola:
		//std::cout << "Hyperbola " << _c << " " << _u << " " << _v << " " << _limits.first << " " << _limits.second << " " << dir << std::endl;
		if (ev != 0 && std::abs(ev) >= std::abs(eu)) {
			t = std::atanh(-eu/ev);

			tMax = std::numeric_limits<double>::max();
			tMin = -std::numeric_limits<double>::max();
			if (t >= _limits.first && t <= _limits.second) {
				if (eu*std::sinh(t)+ev*std::cosh(t) < 0) {
					tMax = t;
				} else {
					tMin = t;
				}
			}
			//std::cout << " tmin tmax " << tMin << " " << tMax << std::endl;
			if (tMax == std::numeric_limits<double>::max()) {
				if (_hasLimits)
					result.first = std::min(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
				else
					result.first = -std::numeric_limits<double>::max();
			} else {
				result.second = dot(dir,(*this)(tMax));
			}
			if (tMin == -std::numeric_limits<double>::max()) {
				if (_hasLimits)
					result.second = std::max(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
				else
					result.second = std::numeric_limits<double>::max();
			} else {
				result.first = dot(dir,(*this)(tMin));
			}
		} else {
			if (_hasLimits) {
				result.second = std::max(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
				result.first = std::min(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
			} else {
				result.first = -std::numeric_limits<double>::max();
				result.second = std::numeric_limits<double>::max();
			}
		}
		break;
	case Parabola:
		t = -eu/ev/2;
		if (_hasLimits && t < _limits.first) {
			tLim = dot(dir,(*this)(_limits.first));
		} else if (_hasLimits && t > _limits.second) {
			tLim = dot(dir,(*this)(_limits.second));
		} else {
			tLim = dot(dir,(*this)(t));
		}
		if (ev < 0) {
			result.second = tLim;
			if (_hasLimits)
				result.first = std::min(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
			else
				result.first = -std::numeric_limits<double>::max();
		} else if (ev > 0) {
			result.first = tLim;
			if (_hasLimits)
				result.second = std::max(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
			else
				result.second = std::numeric_limits<double>::max();
		} else { // ev = 0 (reduction to Line)
			if (_hasLimits) {
				result.first = std::min(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
				result.second = std::max(dot(dir,(*this)(_limits.first)),dot(dir,(*this)(_limits.second)));
			} else {
				result.first = -std::numeric_limits<double>::max();
				result.second = std::numeric_limits<double>::max();
			}
		}
		break;
	default:
		RW_THROW("extremums encountered non-enumerated value.");
	}
	return result;
}

std::list<Vector3D<> > QuadraticCurve::discretizeAdaptive(double stepsPerRevolution) const {
	std::list<Vector3D<> > points;
	std::list<Vector3D<> > segment;
	const Vector3D<> p1 = (*this)(_limits.first);
	const Vector3D<> p2 = (*this)(_limits.second);
	double dt = 0;
	double K;
	if (_type == Line) {
		points.push_back(p1);
		points.push_back(p2);
	} else if (_type == Elliptic) {
		// Assumption: first limit from -2*Pi to Pi, second limit from -Pi to 2*Pi.
		if (_v.norm2() > _u.norm2()) {
			const std::list<double> times = discretizeEllipse(stepsPerRevolution);
			for (std::list<double>::const_iterator tit = times.begin(); tit != times.end(); tit++)
				points.push_back((*this)(*tit));
		} else {
			// maximum curvature at t=Pi/2 and t=-Pi/2
			QuadraticCurve rot(_c,_v,-_u,Elliptic);
			rot.setLimits(std::make_pair<>(_limits.first-Pi/2,_limits.second-Pi/2));
			const std::list<double> times = rot.discretizeEllipse(stepsPerRevolution);
			for (std::list<double>::const_iterator tit = times.begin(); tit != times.end(); tit++)
				points.push_back((*this)(*tit+Pi/2));
		}
	} else if (_type == Parabola || _type == Hyperbola) {
		if (_limits.first < 0) {
			for (double t = (_limits.second < 0)?_limits.second:0; t > _limits.first; t -= dt) {
				K = curvature(t);
				dt = Pi*2/K/stepsPerRevolution;
				points.push_front((*this)(t));
			}
			points.push_front(p1);
		} else if (_limits.first == 0) {
			points.push_back(p1);
		}
		if (_limits.second > 0) {
			K = curvature(0);
			dt = Pi*2/K/stepsPerRevolution;
			for (double t = (_limits.first > 0)?_limits.first:dt; t < _limits.second; t += dt) {
				K = curvature(t);
				dt = Pi*2/K/stepsPerRevolution;
				points.push_back((*this)(t));
			}
			points.push_back(p2);
		}
	}
	return points;
}

OBB<> QuadraticCurve::obr() const {
	if (!hasLimits())
		RW_THROW("QuadraticCurve (obr): limits are needed to create the bounding rectangle for curve.");
	if (_type == Line) {
		const double uLen = _u.norm2()*std::fabs(_limits.second-_limits.first)/2;
		const Vector3D<> c = _c+(_limits.second+_limits.first)/2*_u;
		return OBB<>(Transform3D<>(c,Rotation3D<>(normalize(_u),normalize(_v),normalize(cross(_u,_v)))),Vector3D<>(uLen,0,0));
	}
	// First evaluate end points and use those for the first axis
	const Vector3D<> p1 = (*this)(_limits.first);
	const Vector3D<> p2 = (*this)(_limits.second);
	const Vector3D<> pDiff = p2-p1;
	const Vector3D<> dir1 = normalize(pDiff);
	const double len1 = pDiff.norm2()/2;

	// Find second axis
	const Vector3D<> n = cross(_u,_v);
	const Vector3D<> dir2 = normalize(cross(n,pDiff));

	// Find min and max
	double t = 0;
	switch(_type) {
	case Elliptic:
		t = std::atan2(dot(dir2,_u),dot(dir2,_v));
		break;
	case Hyperbola:
		t = std::atanh(-dot(dir2,_u)/dot(dir2,_v));
		break;
	case Parabola:
		t = -dot(dir2,_u)/dot(dir2,_v)/2;
		break;
	default:
		RW_THROW("OBR encountered non-enumerated value.");
	}
	const double len2 = dot((*this)(t)-p2,dir2)/2;
	const Vector3D<> c = (p1+p2)/2+len2*dir2;
	return OBB<>(Transform3D<>(c,Rotation3D<>(normalize(dir1),normalize(dir2),cross(dir1,dir2))),Vector3D<>(len1,std::fabs(len2),0));
}

std::vector<Vector3D<> > QuadraticCurve::closestPoints(const Vector3D<>& p) const {
	const std::vector<double> times = closestTimes(p);
	std::vector<rw::math::Vector3D<>> res(times.size());
	for (std::size_t i = 0; i < times.size(); i++) {
		res[i] = (*this)(times[i]);
	}
	return res;
}

Vector3D<> QuadraticCurve::x(double t) const {
	return _c+r(t)*_u+s(t)*_v;
}

Vector3D<> QuadraticCurve::dx(double t) const {
	switch(_type) {
	case Elliptic:
		return std::cos(t)*_u-std::sin(t)*_v;
	case Hyperbola:
		return std::cosh(t)*_u+std::sinh(t)*_v;
	case Line:
		return _u;
	case Parabola:
		return _u+t*_v*2;
	}
	return Vector3D<>::zero();
}

Vector3D<> QuadraticCurve::ddx(double t) const {
	switch(_type) {
	case Elliptic:
		return -std::sin(t)*_u-std::cos(t)*_v;
	case Hyperbola:
		return std::sinh(t)*_u+std::cosh(t)*_v;
	case Line:
		return Vector3D<>::zero();
	case Parabola:
		return _v*2;
	}
	return Vector3D<>::zero();
}

Vector3D<> QuadraticCurve::operator()(double t) const {
	return _c+r(t)*_u+s(t)*_v;
}

bool QuadraticCurve::inLimits(double t) const {
	if (t >= _limits.first && t <= _limits.second)
		return true;
	if (_type == Elliptic) {
		while(t < 0) {
			t += Pi*2;
		}
		while(t > Pi*2) {
			t -= Pi*2;
		}
		if ((t-Pi*2 >= _limits.first && t-Pi*2 <= _limits.second) || (t >= _limits.first && t <= _limits.second)) {
			return true;
		}
	}
	return false;
}

void QuadraticCurve::setLimits(const std::pair<double,double>& limits) {
	_limits = limits;
	if (_type == Elliptic) {
		if (_limits.second-_limits.first < 0 || _limits.second-limits.first > Pi*2) {
			_limits.first = 0;
			_limits.second = Pi*2;
		} else {
			while(_limits.first < -Pi*2) {
				_limits.first += Pi*2;
				_limits.second += Pi*2;
			}
			while(_limits.second > Pi*2) {
				_limits.first -= Pi*2;
				_limits.second -= Pi*2;
			}
		}
	}
	_hasLimits = true;
}

double QuadraticCurve::curvature(double t) const {
	const double ku = _u.norm2();
	const double kv = _v.norm2();
	switch(_type) {
	case Elliptic:
		return std::abs(ku*kv/std::pow(ku*ku*std::pow(std::cos(t),2.)+kv*kv*std::pow(std::sin(t),2.),1.5));
	case Hyperbola:
		return std::abs(ku*kv/std::pow(ku*ku*std::pow(std::cosh(t),2.)+kv*kv*std::pow(std::sinh(t),2.),1.5));
	case Line:
		return 0;
	case Parabola:
		return 2.*ku*kv/std::pow(ku*ku+4.*kv*kv*t*t,1.5);
	}
	return 0;
}

std::vector<double> QuadraticCurve::closestTimes(const Vector3D<>& p) const {
	const Eigen::Vector3d u = _u.e();
	const Eigen::Vector3d v = _v.e();
	const Eigen::Vector3d cc = _c.e();
	const Eigen::Vector3d dp = cc-p.e();

	Polynomial<> pol(0);
	double K1;
	double K2;
	switch(_type) {
	case QuadraticCurve::Elliptic:
		pol = Polynomial<>(4);
		K1 = dp.transpose()*u;
		K2 = u.transpose()*u;
		K2 -= v.transpose()*v;
		pol[4] = dp.transpose()*v;
		pol[3] = 2.*(K1-K2);
		pol[2] = 0;
		pol[1] = 2.*(K1+K2);
		pol[0] = -pol[4];
		break;
	case QuadraticCurve::Hyperbola:
		pol = Polynomial<>(4);
		K1 = dp.transpose()*v;
		K2 = u.transpose()*u;
		K2 += v.transpose()*v;
		pol[4] = -dp.transpose()*u;
		pol[3] = 2.*(K2-K1);
		pol[2] = 0;
		pol[1] = 2.*(K2+K1);
		pol[0] = -pol[4];
		break;
	case QuadraticCurve::Line:
		pol = Polynomial<>(1);
		pol[1] = u.transpose()*u;
		pol[0] = dp.transpose()*u;
		break;
	case QuadraticCurve::Parabola:
		pol = Polynomial<>(3);
		pol[3] = 2.*v.transpose()*v;
		pol[2] = 0;
		pol[1] = 2.*dp.transpose()*v;
		pol[1] += u.transpose()*u;
		pol[0] = dp.transpose()*u;
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
	if (_type == QuadraticCurve::Elliptic) {
		for (std::size_t i = 0; i < minSols.size(); i++)
			minSols[i] = std::atan(minSols[i])*2;
	} else if (_type == QuadraticCurve::Hyperbola) {
		for (std::size_t i = 0; i < minSols.size(); i++) {
			minSols[i] = std::atanh(minSols[i])*2;
		}
	}

	if (!_hasLimits)
		return minSols;

	std::vector<double> limitedSols;
	bool minLim = false;
	bool maxLim = false;
	for (std::size_t i = 0; i < minSols.size(); i++) {
		if (minSols[i] >= _limits.second) {
			if (!maxLim) {
				limitedSols.push_back(_limits.second);
				maxLim = true;
			}
		} else if (minSols[i] <= _limits.first) {
			if (!minLim) {
				limitedSols.push_back(_limits.first);
				minLim = true;
			}
		} else {
			limitedSols.push_back(minSols[i]);
		}
	}

	return limitedSols;
}

double QuadraticCurve::closestTime(const Vector3D<>& p) const {
	const std::vector<double> times = closestTimes(p);
	if (times.size() == 0)
		RW_THROW("No closest times found!");
	double res = times[0];
	for (std::size_t i = 1; i < times.size(); i++) {
		if (times[i] < res)
			res = times[i];
	}
	return res;
}

double QuadraticCurve::r(double t) const {
	switch(_type) {
	case Elliptic:
		return std::sin(t);
	case Hyperbola:
		return std::sinh(t);
	case Line:
		return t;
	case Parabola:
		return t;
	}
	return 0;
}

double QuadraticCurve::s(double t) const {
	switch(_type) {
	case Elliptic:
		return std::cos(t);
	case Hyperbola:
		return std::cosh(t);
	case Line:
		return 0;
	case Parabola:
		return t*t;
	}
	return 0;
}

std::list<double> QuadraticCurve::discretizeEllipse(double stepsPerRevolution) const {
	double dt = 0;
	double K = 0;
	std::list<double> times;
	std::list<double> segment;
	// maximum curvature at t=0 and t=Pi
	if (_limits.first < -Pi) {
		// treat t=-2Pi to t=-3Pi/2
		if (_limits.first < -Pi*3/2) {
			for (double t = _limits.first; std::cos(t) > 0; t += dt) {
				K = curvature(t);
				dt = Pi*2/K/stepsPerRevolution;
				times.push_back(t);
			}
		}
		if (_limits.first <= -Pi*3/2)
			times.push_back(-Pi*3/2);
		// treat t=-3Pi/2 to t=-Pi
		for (double t = -Pi; std::cos(t) < 0 && t > _limits.first; t -= dt) {
			K = curvature(t);
			dt = Pi*2/K/stepsPerRevolution;
			if (t != -Pi) {
				segment.push_front(t);
			}
		}
		if (_limits.first > -Pi*3/2)
			segment.push_front(_limits.first);
		times.insert(times.end(),segment.begin(),segment.end());
		segment.clear();
	}
	// treat t=-Pi to t=-Pi/2
	if (_limits.first < -Pi/2) {
		for (double t = (_limits.first > -Pi)?_limits.first:-Pi; std::cos(t) < 0 && t >= _limits.first && t < _limits.second; t += dt) {
			K = curvature(t);
			dt = Pi*2/K/stepsPerRevolution;
			times.push_back(t);
		}
		if (_limits.second < -Pi/2)
			times.push_back(_limits.second);
		else
			times.push_back(-Pi/2);
	}
	// treat t=-Pi/2 to t=0
	if (_limits.first < 0 && _limits.second > -Pi/2) {
		for (double t = 0; std::cos(t) > 0 && t > _limits.first && t <= _limits.second; t -= dt) {
			K = curvature(t);
			dt = Pi*2/K/stepsPerRevolution;
			if (t != 0) {
				segment.push_front(t);
			}
		}
		if (_limits.second == 0) {
			segment.push_back(0);
		}
		times.insert(times.end(),segment.begin(),segment.end());
		segment.clear();
	}
	// treat t=0 to t=Pi/2
	if (_limits.first < Pi/2 && _limits.second > 0) {
		for (double t = (_limits.first <= 0)?0:_limits.first; std::cos(t) > 0 && t >= _limits.first && t < _limits.second; t += dt) {
			K = curvature(t);
			dt = Pi*2/K/stepsPerRevolution;
			times.push_back(t);
		}
		if (_limits.second >= Pi/2)
			times.push_back(Pi/2);
	}
	// treat t=Pi to t=Pi/2
	if (_limits.second > Pi/2) {
		for (double t = (_limits.second < Pi)?_limits.second:Pi; std::cos(t) < 0 && t > _limits.first && t <= _limits.second; t -= dt) {
			K = curvature(t);
			dt = Pi*2/K/stepsPerRevolution;
			if (t != Pi) {
				segment.push_front(t);
			}
		}
		times.insert(times.end(),segment.begin(),segment.end());
		segment.clear();
	}
	if (_limits.second == Pi)
		times.push_back(Pi);
	if (_limits.second > Pi) {
		// treat t=Pi to t=Pi*3/2
		for (double t = Pi; std::cos(t) < 0 && t < _limits.second; t += dt) {
			K = curvature(t);
			dt = Pi*2/K/stepsPerRevolution;
			times.push_back(t);
		}
		if (_limits.second < Pi*3/2)
			times.push_back(_limits.second);
		else
			times.push_back(Pi*3/2);
		// treat t=Pi*3/2 to t=2Pi
		if (_limits.second > Pi*3/2) {
			for (double t = _limits.second; std::cos(t) > 0 && t <= _limits.second; t -= dt) {
				K = curvature(t);
				dt = Pi*2/K/stepsPerRevolution;
				segment.push_front(t);
			}
			times.insert(times.end(),segment.begin(),segment.end());
			segment.clear();
		}
	}
	return times;
}
