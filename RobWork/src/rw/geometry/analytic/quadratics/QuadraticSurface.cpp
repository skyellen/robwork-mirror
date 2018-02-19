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

#include "QuadraticSurface.hpp"

#include <rw/geometry/Polygon.hpp>
#include <rw/geometry/PolygonUtil.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/Delaunay.hpp>
#include <rw/math/Vector2D.hpp>

#include <Eigen/Eigenvalues>

#include <list>

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::math;

QuadraticSurface::QuadraticSurface(const Eigen::Diagonal<Eigen::Matrix3d>& A, const Eigen::Vector3d& a, double u, const std::vector<TrimmingRegion>& conditions):
	_A(Eigen::DiagonalMatrix<double,3,3>(A)),
	_a(a),
	_u(u),
	_determinantA(A.determinant()),
	_diagonal(true),
	_conditions(conditions),
	_stepsPerRevolution(10)
{
}

QuadraticSurface::QuadraticSurface(const Eigen::DiagonalMatrix<double,3,3>& A, const Eigen::Vector3d& a, double u, const std::vector<TrimmingRegion>& conditions):
	_A(A),
	_a(a),
	_u(u),
	_determinantA(A.diagonal().determinant()),
	_diagonal(true),
	_conditions(conditions),
	_stepsPerRevolution(10)
{
}

QuadraticSurface::QuadraticSurface(const Eigen::SelfAdjointView<const Eigen::Matrix3d, Eigen::Upper>& A, const Eigen::Vector3d& a, double u, const std::vector<TrimmingRegion>& conditions):
	_A(A),
	_a(a),
	_u(u),
	_determinantA(_A.determinant()),
	_diagonal(false),
	_conditions(conditions),
	_stepsPerRevolution(10)
{
}

QuadraticSurface::QuadraticSurface(const Eigen::SelfAdjointView<Eigen::Matrix3d, Eigen::Upper>& A, const Eigen::Vector3d& a, double u, const std::vector<TrimmingRegion>& conditions):
	_A(A),
	_a(a),
	_u(u),
	_determinantA(_A.determinant()),
	_diagonal(false),
	_conditions(conditions),
	_stepsPerRevolution(10)
{
}

QuadraticSurface::QuadraticSurface(const Eigen::SelfAdjointView<const Eigen::Matrix3d, Eigen::Lower>& A, const Eigen::Vector3d& a, double u, const std::vector<TrimmingRegion>& conditions):
	_A(A),
	_a(a),
	_u(u),
	_determinantA(_A.determinant()),
	_diagonal(false),
	_conditions(conditions),
	_stepsPerRevolution(10)
{
}

QuadraticSurface::QuadraticSurface(const Eigen::SelfAdjointView<Eigen::Matrix3d, Eigen::Lower>& A, const Eigen::Vector3d& a, double u, const std::vector<TrimmingRegion>& conditions):
	_A(A),
	_a(a),
	_u(u),
	_determinantA(_A.determinant()),
	_diagonal(false),
	_conditions(conditions),
	_stepsPerRevolution(10)
{
}

QuadraticSurface::QuadraticSurface(const Eigen::Matrix3d& A, bool diagonal, double determinantA, const Eigen::Vector3d& a, double u, const std::vector<TrimmingRegion>& conditions, double stepsPerRevolution):
	_A(A),
	_a(a),
	_u(u),
	_determinantA(determinantA),
	_diagonal(diagonal),
	_conditions(conditions),
	_stepsPerRevolution(stepsPerRevolution)
{
}

QuadraticSurface::~QuadraticSurface() {
}

QuadraticSurface::Ptr QuadraticSurface::transform(const Transform3D<>& T) const {
	const Eigen::Matrix3d R = T.R().e();
	const Eigen::Matrix3d Rinv = inverse(T.R()).e();
	const Eigen::Vector3d p = T.P().e();
	std::vector<TrimmingRegion> conditions;
	for (std::size_t i = 0; i < _conditions.size(); i++)
		conditions.push_back(_conditions[i]->transform(T));
	const Eigen::Matrix3d Anew = R*_A*Rinv;
	return ownedPtr(new QuadraticSurface(Anew, false, _determinantA, R*_a-Anew*p, (Anew*p-2*(R*_a)).transpose()*p+_u, conditions,_stepsPerRevolution));
}

QuadraticSurface::Ptr QuadraticSurface::transform(const Vector3D<>& P) const {
	const Eigen::Vector3d p = P.e();
	std::vector<TrimmingRegion> conditions;
	for (std::size_t i = 0; i < _conditions.size(); i++)
		conditions.push_back(_conditions[i]->transform(Transform3D<>(P)));
	return ownedPtr(new QuadraticSurface(_A, _diagonal, _determinantA, _a-_A*p, (_A*p-2*_a).transpose()*p+_u, conditions,_stepsPerRevolution));
}

QuadraticSurface::Ptr QuadraticSurface::scale(double factor) const {
	std::vector<TrimmingRegion> conditions;
	for (std::size_t i = 0; i < _conditions.size(); i++) {
		conditions.push_back(_conditions[i]->scale(factor));
	}
	return ownedPtr(new QuadraticSurface(_A, _diagonal, _determinantA, _a*factor, _u*factor*factor, conditions,_stepsPerRevolution));
}

QuadraticSurface::Ptr QuadraticSurface::clone() const {
	std::vector<TrimmingRegion> conditions;
	for (std::size_t i = 0; i < _conditions.size(); i++) {
		conditions.push_back(_conditions[i]->clone());
	}
	return ownedPtr(new QuadraticSurface(_A, _diagonal, _determinantA, _a, _u, conditions, _stepsPerRevolution));
}

std::pair<double,double> QuadraticSurface::extremums(const Vector3D<>& dir) const {
	if (_diagonal)
		return extremumsDiagonal(dir);

	std::pair<QuadraticSurface,Rotation3D<> > diagonalized = diagonalize();
	const Rotation3D<>& R = diagonalized.second;
	QuadraticSurface& s = diagonalized.first;

	return s.extremumsDiagonal(R*dir);
}

TriMesh::Ptr QuadraticSurface::getTriMesh(const std::vector<Vector3D<> >& border) const {
	if (_diagonal)
		return getTriMeshDiagonal(border);

	std::pair<QuadraticSurface,Rotation3D<> > diagonalized = diagonalize();
	const Rotation3D<>& R = diagonalized.second;
	QuadraticSurface& s = diagonalized.first;

	std::vector<Vector3D<> > borderRot(border.size());
	for (std::size_t i = 0; i < border.size(); i++)
		borderRot[i] = R*border[i];
	return s.getTriMeshDiagonal(borderRot, inverse(R));
}

double QuadraticSurface::operator()(const Vector3D<>& p) const {
	return p.e().transpose()*_A*p.e()+_a.dot(p.e())*2+_u;
}

bool QuadraticSurface::insideTrimmingRegion(const Vector3D<>& P) const {
	for (std::size_t i = 0; i < _conditions.size(); i++) {
		if ((*_conditions[i])(P) > 0)
			return false;
	}
	return true;
}

Vector3D<> QuadraticSurface::normal(const Vector3D<>& point) const {
	return Vector3D<>(_A*point.e()+_a)*2;
}

Vector3D<> QuadraticSurface::gradient(const Vector3D<>& point) const {
	return Vector3D<>(_A*point.e()+_a)*2;
}

QuadraticSurface::Ptr QuadraticSurface::normalize() const {
	std::vector<TrimmingRegion> conditions;
	for (std::size_t i = 0; i < _conditions.size(); i++) {
		conditions.push_back(_conditions[i]->clone());
	}
	const double normFactor = std::max(_A.cwiseAbs().maxCoeff(), std::max(_a.cwiseAbs().maxCoeff(),std::abs(_u)));
	return ownedPtr(new QuadraticSurface(_A/normFactor, _diagonal, _determinantA/std::pow(normFactor,3), _a/normFactor, _u/normFactor, conditions,_stepsPerRevolution));
}

std::pair<QuadraticSurface,Rotation3D<> > QuadraticSurface::diagonalize() const {
	if (_diagonal)
		return std::make_pair(*this,Rotation3D<>::identity());
	const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(_A);
	Rotation3D<> R(eigen.eigenvectors().transpose());
	if (!R.isProperRotation()) {
		R(2,0) *= -1;
		R(2,1) *= -1;
		R(2,2) *= -1;
	}
	if (!R.isProperRotation())
		RW_THROW("Could not create proper rotation from eigenvalues!");
	std::vector<TrimmingRegion> conditions;
	for (std::size_t i = 0; i < _conditions.size(); i++)
		conditions.push_back(_conditions[i]->transform(Transform3D<>(R)));
	return std::make_pair(QuadraticSurface(Eigen::DiagonalMatrix<double,3,3>(eigen.eigenvalues()), _diagonal, _determinantA,_a.transpose()*inverse(R).e(),_u, conditions,_stepsPerRevolution),R);
}

namespace {
static const double EPS = std::numeric_limits<double>::epsilon()*5;
}

TriMesh::Ptr QuadraticSurface::getTriMeshDiagonal(const std::vector<Vector3D<> >& border, const Rotation3D<>& R) const {
	const PlainTriMeshN1D::Ptr mesh = ownedPtr(new PlainTriMeshN1D());

	// Choose indices
	std::size_t u = 3;
	std::size_t v = 3;
	std::size_t e = 3;
	if (std::fabs(_A(2,2)) < EPS && std::fabs(_a[2]) > EPS) {
		u = 0;
		v = 1;
		e = 2;
	} else if (std::fabs(_A(1,1)) < EPS && std::fabs(_a[1]) > EPS) {
		u = 2;
		v = 0;
		e = 1;
	} else if (std::fabs(_A(0,0)) < EPS && std::fabs(_a[0]) > EPS) {
		u = 1;
		v = 2;
		e = 0;
	} else if (std::fabs(_A(2,2)) >= EPS) {
		u = 0;
		v = 1;
		e = 2;
	} else if (std::fabs(_A(1,1)) >= EPS) {
		u = 2;
		v = 0;
		e = 1;
	} else if (std::fabs(_A(0,0)) >= EPS) {
		u = 1;
		v = 2;
		e = 0;
	}
	if (u > 2 || v > 2 || e > 2)
		RW_THROW("Error happened. Could not choose explicit variable!");

	// Split border polygon in front and back (when seen in uv plane)
	if (std::fabs(_A(e,e)) >= EPS) {
		const double eSplit = -_a[e]/_A(e,e);

		std::list<std::vector<std::size_t> > borderFront;
		std::list<std::vector<std::size_t> > borderBack;
		// Front
		{
			Vector3D<> last = border.front();
			std::vector<std::size_t> cur;
			if (last[e] >= -EPS)
				cur.push_back(0);
			for (std::size_t i = 1; i < border.size(); i++) {
				if (last[e] >= -EPS && border[i][e] >= -EPS) {
					cur.push_back(i);
				} else if (last[e] < -EPS) {
					if (cur.size() > 0) {
						borderFront.push_back(cur);
						cur.clear();
					}
					if (border[i][e] >= -EPS)
						cur.push_back(i);
				}
				last = border[i];
			}
			if (last[e] >= -EPS) {
				if (border.front()[e] >= -EPS && borderFront.size() > 0) {
					borderFront.front().insert(borderFront.front().begin(),cur.begin(),cur.end());
					cur.clear();
				}
			}
			if (cur.size() > 0) {
				borderFront.push_back(cur);
			}
		}
		// Back
		{
			Vector3D<> last = border.front();
			std::vector<std::size_t> cur;
			if (last[e] <= EPS)
				cur.push_back(0);
			for (std::size_t i = 1; i < border.size(); i++) {
				if (last[e] <= EPS && border[i][e] <= EPS) {
					cur.push_back(i);
				} else if (last[e] > EPS) {
					if (cur.size() > 0) {
						borderBack.push_back(cur);
						cur.clear();
					}
					if (border[i][e] <= EPS)
						cur.push_back(i);
				}
				last = border[i];
			}
			if (last[e] <= EPS) {
				if (border.front()[e] <= EPS && borderBack.size() > 0) {
					borderBack.front().insert(borderBack.front().begin(),cur.begin(),cur.end());
					cur.clear();
				}
			}
			if (cur.size() > 0) {
				borderBack.push_back(cur);
			}
		}

		// If there are front border segments lying completely inside the +/- EPS region, remove them if they are part of a larger back segment.
		{
			std::list<std::vector<std::size_t> >& listA = borderFront;
			const std::list<std::vector<std::size_t> >& listB = borderBack;

			for (std::list<std::vector<std::size_t> >::const_iterator it = listA.begin(); it != listA.end();) {
				bool within = true;
				for (std::size_t i = 0; i < (*it).size() && within; i++) {
					if (std::abs(border[(*it)[i]][e]) > EPS) {
						within = false;
					}
				}
				if (within) {
					bool match = false;
					for (std::list<std::vector<std::size_t> >::const_iterator itB = listB.begin(); itB != listB.end() && !match; itB++) {
						bool outsideB = false;
						for (std::size_t i = 0; i < (*itB).size() && !outsideB; i++) {
							if (std::abs(border[(*itB)[i]][e]) > EPS) {
								outsideB = true;
							}
						}
						if (outsideB) {
							bool firstMatched = false;
							std::size_t i = 0;
							for (std::size_t j = 0; j <= (*itB).size() && !match; j++) {
								if (j == (*itB).size()) {
									if (firstMatched)
										j = 0;
									else
										break;
								}
								if (!firstMatched) {
									if ((*itB)[j] == (*it)[i]) {
										firstMatched = true;
										i++;
									}
								} else {
									if (i >= (*it).size()) {
										match = true;
									} else if ((*itB)[j] != (*it)[i]) {
										match = false;
										break;
									} else if ((*itB)[j] == (*it)[i]) {
										i++;
									}
								}
							}
						}
					}
					if (match) {
						it = listA.erase(it);
					} else {
						it++;
					}
				} else {
					it++;
				}
			}
		}
		// If there are back border segments lying completely inside the +/- EPS region, remove them if they are part of a larger front segment.
		{
			std::list<std::vector<std::size_t> >& listA = borderBack;
			const std::list<std::vector<std::size_t> >& listB = borderFront;

			for (std::list<std::vector<std::size_t> >::const_iterator it = listA.begin(); it != listA.end();) {
				bool within = true;
				for (std::size_t i = 0; i < (*it).size() && within; i++) {
					if (std::abs(border[(*it)[i]][e]) > EPS) {
						within = false;
					}
				}
				if (within) {
					bool match = false;
					for (std::list<std::vector<std::size_t> >::const_iterator itB = listB.begin(); itB != listB.end() && !match; itB++) {
						bool outsideB = false;
						for (std::size_t i = 0; i < (*itB).size() && !outsideB; i++) {
							if (std::abs(border[(*itB)[i]][e]) > EPS) {
								outsideB = true;
							}
						}
						if (outsideB) {
							bool firstMatched = false;
							std::size_t i = 0;
							for (std::size_t j = 0; j <= (*itB).size() && !match; j++) {
								if (j == (*itB).size()) {
									if (firstMatched)
										j = 0;
									else
										break;
								}
								if (!firstMatched) {
									if ((*itB)[j] == (*it)[i]) {
										firstMatched = true;
										i++;
									}
								} else {
									if (i >= (*it).size()) {
										match = true;
									} else if ((*itB)[j] != (*it)[i]) {
										match = false;
										break;
									} else if ((*itB)[j] == (*it)[i]) {
										i++;
									}
								}
							}
						}
					}
					if (match) {
						it = listA.erase(it);
					} else {
						it++;
					}
				} else {
					it++;
				}
			}
		}

		// Find silhouette of surface
		std::vector<QuadraticCurve> silhouette = findSilhouette(u, v, e, eSplit);

		// Try to connect polygons if open
		const std::list<std::vector<Vector3D<> > > fullPolygonFront = combinePolygons(border, borderFront, silhouette);
		const std::list<std::vector<Vector3D<> > > fullPolygonBack = combinePolygons(border, borderBack, silhouette);

		for (std::list<std::vector<Vector3D<> > >::const_iterator it = fullPolygonFront.begin(); it != fullPolygonFront.end(); it++) {
			makeSurface(*it, u, v, e, eSplit, FRONT, R, mesh);
		}
		for (std::list<std::vector<Vector3D<> > >::const_iterator it = fullPolygonBack.begin(); it != fullPolygonBack.end(); it++) {
			makeSurface(*it, u, v, e, eSplit, BACK, R, mesh);
		}
	} else {
		makeSurface(border, u, v, e, 0, BOTH, R, mesh);
	}

	return mesh;
}

std::vector<QuadraticCurve> QuadraticSurface::findSilhouette(std::size_t u, std::size_t v, std::size_t e, double eSplit) const {
	// Find silhouette of surface
	std::vector<QuadraticCurve> silhouette;
	if (std::fabs(_A(u,u)) < EPS && std::fabs(_a[u]) < EPS) {
		const double s = _a[v]*_a[v]-(_u+_a[e]*_a[e]/_A(e,e))*_A(v,v);
		if (s >= 0) {
			const double vVal1 = (_a[v]+std::sqrt(s))/_A(v,v);
			const double vVal2 = (_a[v]-std::sqrt(s))/_A(v,v);
			const Vector3D<> c1((u==0)?0:(v==0)?vVal1:eSplit,(u==1)?0:(v==1)?vVal1:eSplit,(u==2)?0:(v==2)?vVal1:eSplit);
			const Vector3D<> c2((u==0)?0:(v==0)?vVal2:eSplit,(u==1)?0:(v==1)?vVal2:eSplit,(u==2)?0:(v==2)?vVal2:eSplit);
			const Vector3D<> uDir((u==0)?1:0,(u==1)?1:0,(u==2)?1:0);
			const Vector3D<> vDir((v==0)?1:0,(v==1)?1:0,(v==2)?1:0);
			silhouette.push_back(QuadraticCurve(c1, uDir, vDir, QuadraticCurve::Line));
			if (s != 0)
				silhouette.push_back(QuadraticCurve(c2, uDir, vDir, QuadraticCurve::Line));
		}
	} else if (std::fabs(_A(v,v)) < EPS && std::fabs(_a[v]) < EPS) {
		const double s = _a[u]*_a[u]-(_u+_a[e]*_a[e]/_A(e,e))*_A(u,u);
		if (s >= 0) {
			const double uVal1 = (_a[u]+std::sqrt(s))/_A(u,u);
			const double uVal2 = (_a[u]-std::sqrt(s))/_A(u,u);
			const Vector3D<> c1((u==0)?uVal1:(v==0)?0:eSplit,(u==1)?uVal1:(v==1)?0:eSplit,(u==2)?uVal1:(v==2)?0:eSplit);
			const Vector3D<> c2((u==0)?uVal2:(v==0)?0:eSplit,(u==1)?uVal2:(v==1)?0:eSplit,(u==2)?uVal2:(v==2)?0:eSplit);
			const Vector3D<> uDir((u==0)?1:0,(u==1)?1:0,(u==2)?1:0);
			const Vector3D<> vDir((v==0)?1:0,(v==1)?1:0,(v==2)?1:0);
			silhouette.push_back(QuadraticCurve(c1, vDir, uDir, QuadraticCurve::Line));
			if (s != 0)
				silhouette.push_back(QuadraticCurve(c2, vDir, uDir, QuadraticCurve::Line));
		}
	} else if (std::fabs(_A(u,u)) < EPS) {
		RW_THROW("Polynomial not yet implemented!");
	} else if (std::fabs(_A(v,v)) < EPS) {
		RW_THROW("Polynomial not yet implemented!");
	} else if (_A(u,u) > 0 && _A(v,v) > 0) {
		// Ellipse
		const std::size_t ru = (_A(u,u)>0)?u:v;
		const std::size_t rv = (_A(u,u)>0)?v:u;
		const double ruValC = -_a[ru]/_A(ru,ru);
		const double rvValC = -_a[rv]/_A(rv,rv);
		const double Ke = _A(e,e)*eSplit*eSplit+2.*_a[e]*eSplit+_u;
		const double b = std::sqrt((_a[ru]*_a[ru]/_A(ru,ru)*2+_a[rv]*_a[rv]/_A(rv,rv)-Ke)/_A(rv,rv));
		const double a = std::sqrt(b*b*_A(rv,rv)/_A(ru,ru));
		Vector3D<> c;
		Vector3D<> uDir;
		Vector3D<> vDir;
		if (_A(u,u) > 0) {
			c = Vector3D<>((u==0)?ruValC:(v==0)?rvValC:eSplit,(u==1)?ruValC:(v==1)?rvValC:eSplit,(u==2)?ruValC:(v==2)?rvValC:eSplit);
			uDir = Vector3D<>((v==0)?1:0,(v==1)?1:0,(v==2)?1:0)*a;
			vDir = Vector3D<>((u==0)?1:0,(u==1)?1:0,(u==2)?1:0)*b;
		} else {
			c = Vector3D<>((u==0)?rvValC:(v==0)?ruValC:eSplit,(u==1)?rvValC:(v==1)?ruValC:eSplit,(u==2)?rvValC:(v==2)?ruValC:eSplit);
			uDir = Vector3D<>((u==0)?1:0,(u==1)?1:0,(u==2)?1:0)*b;
			vDir = Vector3D<>((v==0)?1:0,(v==1)?1:0,(v==2)?1:0)*a;
		}
		silhouette.push_back(QuadraticCurve(c, uDir, vDir, QuadraticCurve::Elliptic));
		silhouette.push_back(QuadraticCurve(c, uDir, -vDir, QuadraticCurve::Elliptic));
	} else {
		// Hyperbola
		const std::size_t ru = (_A(u,u)>0)?u:v;
		const std::size_t rv = (_A(u,u)>0)?v:u;
		const double ruValC = -_a[ru]/_A(ru,ru);
		const double rvValC = -_a[rv]/_A(rv,rv);
		const double Ke = _A(e,e)*eSplit*eSplit+2.*_a[e]*eSplit+_u;
		const double b = std::sqrt((_a[ru]-_a[rv]*_a[rv]/_A(rv,rv)+Ke)/_A(rv,rv));
		const double a = std::sqrt(-b*b*_A(rv,rv)/_A(ru,ru));
		Vector3D<> c;
		Vector3D<> uDir;
		Vector3D<> vDir;
		if (_A(u,u) > 0) {
			c = Vector3D<>((u==0)?ruValC:(v==0)?rvValC:eSplit,(u==1)?ruValC:(v==1)?rvValC:eSplit,(u==2)?ruValC:(v==2)?rvValC:eSplit);
			uDir = Vector3D<>((v==0)?1:0,(v==1)?1:0,(v==2)?1:0)*a;
			vDir = Vector3D<>((u==0)?1:0,(u==1)?1:0,(u==2)?1:0)*b;
		} else {
			c = Vector3D<>((u==0)?rvValC:(v==0)?ruValC:eSplit,(u==1)?rvValC:(v==1)?ruValC:eSplit,(u==2)?rvValC:(v==2)?ruValC:eSplit);
			uDir = Vector3D<>((u==0)?1:0,(u==1)?1:0,(u==2)?1:0)*b;
			vDir = Vector3D<>((v==0)?1:0,(v==1)?1:0,(v==2)?1:0)*a;
		}
		silhouette.push_back(QuadraticCurve(c, uDir, vDir, QuadraticCurve::Hyperbola));
		silhouette.push_back(QuadraticCurve(c, uDir, -vDir, QuadraticCurve::Hyperbola));
	}
	return silhouette;
}

std::list<std::vector<Vector3D<> > > QuadraticSurface::combinePolygons(const std::vector<Vector3D<> >& border, const std::list<std::vector<std::size_t> >& subborder, const std::vector<QuadraticCurve>& silhouette) const {
	std::list<std::vector<std::size_t> >::const_iterator curPol;
	// Try to connect polygons if open
	std::list<std::vector<Vector3D<> > > fullPolygons;
	for (curPol = subborder.begin(); curPol != subborder.end(); curPol++) {
		fullPolygons.resize(fullPolygons.size()+1);
		std::vector<Vector3D<> >& fullPolygon = fullPolygons.back();
		const QuadraticCurve* closest = &silhouette[0];
		const Vector3D<>& P1 = border[curPol->front()];
		const Vector3D<>& P2 = border[curPol->back()];
		double time1 = silhouette[0].closestTime(P1);
		double time2 = silhouette[0].closestTime(P2);
		double dist = std::min((silhouette[0](time1)-P1).norm2(),((silhouette[0](time2)-P2).norm2()));
		for (std::size_t i = 1; i < silhouette.size(); i++) {
			time1 = silhouette[i].closestTime(P1);
			time2 = silhouette[i].closestTime(P2);
			const double d = std::min((silhouette[i](time1)-P1).norm2(),((silhouette[i](time2)-P2).norm2()));
			if (d < dist) {
				closest = &silhouette[i];
				dist = d;
			}
		}
		time1 = closest->closestTime(P1);
		time2 = closest->closestTime(P2);
		QuadraticCurve cp(*closest);
		cp.setLimits(std::make_pair((time1<time2)?time1:time2,(time1<time2)?time2:time1));
		for (std::size_t k = 0; k < curPol->size(); k++) {
			fullPolygon.push_back(border[(*curPol)[k]]);
		}

		const std::list<Vector3D<> > seg = cp.discretizeAdaptive(_stepsPerRevolution);
		if ((seg.front()-P1).norm2() < (seg.front()-P2).norm2()) {
			fullPolygon.insert(fullPolygon.end(),++seg.rbegin(),--seg.rend());
		} else {
			fullPolygon.insert(fullPolygon.end(),++seg.begin(),--seg.end());
		}
	}
	return fullPolygons;
}

void QuadraticSurface::makeSurface(const std::vector<Vector3D<> > fullPolygon, std::size_t u, std::size_t v, std::size_t e, double eSplit, Place place, const Rotation3D<>& R, PlainTriMeshN1D::Ptr mesh) const {
	// Project border
	Polygon<Vector2D<> > polygon;
	for (std::size_t i = 0; i < fullPolygon.size(); i++) {
		polygon.addVertex(Vector2D<>(fullPolygon[i][u],fullPolygon[i][v]));
	}

	// Do convex decomposition
	if (polygon.size() < 3)
		RW_THROW("Can not decompose polygon that has " << polygon.size() << " vertices.");
	const std::vector<std::vector<std::size_t> > convexPolygonsIndexed = PolygonUtil::convexDecompositionIndexed(polygon);

	std::vector<Polygon<Vector2D<> > > convexPolygons(convexPolygonsIndexed.size());
	{
		// Construct plain polygon
		std::size_t i = 0;
		for (std::vector<std::vector<std::size_t> >::const_iterator it = convexPolygonsIndexed.begin(); it != convexPolygonsIndexed.end(); it++) {
			const std::vector<std::size_t>& subPoly = *it;
			for (std::size_t k = 0; k < subPoly.size(); k++) {
				convexPolygons[i].addVertex(polygon[subPoly[k]]);
				// Check if edge is shared with other polygon
				const std::size_t idxNext = subPoly[(k+1)%subPoly.size()];
				bool inOther = false;
				for (std::vector<std::vector<std::size_t> >::const_iterator itB = convexPolygonsIndexed.begin(); itB != convexPolygonsIndexed.end() && inOther == false; itB++) {
					if (itB == it)
						continue;
					const std::vector<std::size_t>& subPolyB = *itB;
					for (std::size_t kB = 0; kB < subPolyB.size() && inOther == false; kB++) {
						const std::size_t idxNextB = subPolyB[(kB+1)%subPolyB.size()];
						if (subPolyB[kB] == subPoly[k] && idxNextB == idxNext)
							inOther = true;
						else if (subPolyB[kB] == idxNext && idxNextB == subPoly[k])
							inOther = true;
					}
				}
				if (inOther) {
					// Sample inner border
					const Vector2D<> dp = polygon[idxNext]-polygon[subPoly[k]];
					std::size_t samples = 20;
					if (_A(e,e) < EPS && _A(u,u) < EPS && _A(v,v) < EPS)
						samples = 0; // no samples if surface is a plane!
					for (std::size_t sI = 1; sI < samples; sI++) {
						convexPolygons[i].addVertex(polygon[subPoly[k]]+dp*((double)sI/samples));
					}
				}
			}
			i++;
		}
	}

	// Sample surface points
	double minU = convexPolygons[0][0][0];
	double maxU = convexPolygons[0][0][0];
	double minV = convexPolygons[0][0][1];
	double maxV = convexPolygons[0][0][1];
	for (std::size_t i = 0; i < convexPolygons.size(); i++) {
		const Polygon<Vector2D<> >& convexPoly = convexPolygons[i];
		for (std::size_t k = 0; k < convexPoly.size(); k++) {
			if (convexPoly[k][0] < minU)
				minU = convexPoly[k][0];
			if (convexPoly[k][0] > maxU)
				maxU = convexPoly[k][0];
			if (convexPoly[k][1] < minV)
				minV = convexPoly[k][1];
			if (convexPoly[k][1] > maxV)
				maxV = convexPoly[k][1];
		}
	}

	static const std::size_t nrPoints = 20;
	std::size_t nrPointsU = 1;
	std::size_t nrPointsV = 1;
	if (std::abs(_A(u,u)) >= EPS || std::abs(_A(v,v)) >= EPS) {
		nrPointsU = 2;
		nrPointsV = 2;
	}
	if (std::abs(_A(u,u)) >= EPS)
		nrPointsU = nrPoints;
	if (std::abs(_A(v,v)) >= EPS)
		nrPointsV = nrPoints;
	const double dU = (maxU-minU)/nrPointsU;
	const double dV = (maxV-minV)/nrPointsV;
	std::vector<Vector2D<> > surfacePoints((nrPointsU-1)*(nrPointsV-1));
	for (std::size_t ui = 1; ui < nrPointsU; ui++) {
		const double u = minU+dU*ui;
		for (std::size_t vi = 1; vi < nrPointsV; vi++) {
			const double v = minV+dV*vi;
			surfacePoints[(ui-1)*(nrPointsV-1)+vi-1][0] = u;
			surfacePoints[(ui-1)*(nrPointsV-1)+vi-1][1] = v;
		}
	}

	for (std::size_t i = 0; i < convexPolygons.size(); i++) {
		const Polygon<Vector2D<> >& convexPoly = convexPolygons[i];

		std::vector<Vector2D<> > points2d(convexPoly.size());
		for (std::size_t pi = 0; pi < convexPoly.size(); pi++) {
			points2d[pi] = convexPoly[pi];
		}

		// Take the surface points in the convex region
		for (std::size_t k = 0; k < surfacePoints.size(); k++) {
			if (PolygonUtil::isInsideConvex(surfacePoints[k],convexPoly,1e-1)) {
				points2d.push_back(surfacePoints[k]);
			}
		}

		// Triangulate with Delaunay
		const IndexedTriMesh<>::Ptr meshProj = Delaunay::triangulate(points2d);

		if (std::fabs(_A(2,2)) < EPS && std::fabs(_A(1,1)) < EPS && std::fabs(_A(0,0)) < EPS) {
			// Unproject
			for (std::size_t triI = 0; triI < meshProj->size(); triI++) {
				Triangle<double> tri = meshProj->getTriangle(triI);
				bool br = false;
				for (std::size_t pi = 0; pi < 3; pi++) {
					const double uVal = tri[pi][0];
					const double vVal = tri[pi][1];
					double eVal = -_a[u]*2*uVal-_a[v]*2*vVal-_u;
					eVal /= _a[e]*2;
					if ((place == FRONT && eVal < eSplit) || (place == BACK && eVal > eSplit)) {
						br = true;
						break;
					}
					tri[pi] = R*Vector3D<>((u==0)?uVal:(v==0)?vVal:eVal,(u==1)?uVal:(v==1)?vVal:eVal,(u==2)?uVal:(v==2)?vVal:eVal);
				}
				if (br)
					continue;
				const double nu = _a[u];
				const double nv = _a[v];
				const double ne = _a[e];
				const Vector3D<> normal = R*Vector3D<>((u==0)?nu:(v==0)?nv:ne,(u==1)?nu:(v==1)?nv:ne,(u==2)?nu:(v==2)?nv:ne);
				if (dot(tri.calcFaceNormal(),normal) < 0) {
					mesh->add(TriangleN1<>(tri[0],tri[2],tri[1],normal));
				} else {
					mesh->add(TriangleN1<>(tri[0],tri[1],tri[2],normal));
				}
			}
		} else if (std::fabs(_A(e,e)) < EPS && std::fabs(_a[e]) > EPS) {
			// Unproject
			for (std::size_t triI = 0; triI < meshProj->size(); triI++) {
				Triangle<double> tri = meshProj->getTriangle(triI);
				Vector2D<> avg(0,0);
				bool br = false;
				for (std::size_t pi = 0; pi < 3; pi++) {
					const double uVal = tri[pi][0];
					const double vVal = tri[pi][1];
					double eVal = -_A(u,u)*uVal*uVal-_A(v,v)*vVal*vVal-_a[u]*2*uVal-_a[v]*2*vVal-_u;
					eVal /= _a[e]*2;
					if ((place == FRONT && eVal < eSplit) || (place == BACK && eVal > eSplit)) {
						br = true;
						break;
					}
					tri[pi] = R*Vector3D<>((u==0)?uVal:(v==0)?vVal:eVal,(u==1)?uVal:(v==1)?vVal:eVal,(u==2)?uVal:(v==2)?vVal:eVal);
					avg += Vector2D<>(uVal,vVal);
				}
				if (br)
					continue;
				avg /= 3;
				const double nu = _A(u,u)*avg[0]+_a[u];
				const double nv = _A(v,v)*avg[1]+_a[v];
				const double ne = _a[e];
				const Vector3D<> normal = R*Vector3D<>((u==0)?nu:(v==0)?nv:ne,(u==1)?nu:(v==1)?nv:ne,(u==2)?nu:(v==2)?nv:ne);
				if (dot(tri.calcFaceNormal(),normal) < 0) {
					mesh->add(TriangleN1<>(tri[0],tri[2],tri[1],normal));
				} else {
					mesh->add(TriangleN1<>(tri[0],tri[1],tri[2],normal));
				}
			}
		} else if (std::fabs(_A(e,e)) > EPS) {
			// Unproject
			for (std::size_t triI = 0; triI < meshProj->size(); triI++) {
				Triangle<double> tri = meshProj->getTriangle(triI);
				Vector3D<> avg(0,0,0);
				bool br = false;
				bool zero = false;
				for (std::size_t pi = 0; pi < 3; pi++) {
					const double uVal = tri[pi][0];
					const double vVal = tri[pi][1];
					double eVal = _a[e]*_a[e]/_A(e,e)/_A(e,e)-(_A(u,u)*uVal*uVal+_A(v,v)*vVal*vVal+_a[u]*uVal*2+_a[v]*vVal*2+_u)/_A(e,e);
					if (eVal >= 0) {
						if (place == FRONT)
							eVal = -_a[e]/_A(e,e)+std::sqrt(eVal);
						else if (place == BACK)
							eVal = -_a[e]/_A(e,e)-std::sqrt(eVal);
					} else if (eVal >= -EPS) {
						eVal = -_a[e]/_A(e,e);
					} else {
						//RW_WARN("For triangle " << tri << ", the explicit value under square root was negative (" << eVal << ") assuming split value!");
						zero = true;
						//br = true;
						eVal = eSplit;
					}
					if ((place == FRONT && eVal < eSplit) || (place == BACK && eVal > eSplit)) {
						//RW_WARN("For triangle " << tri << ", the explicit value was not on right side front/back (" << eVal << ")!");
						eVal = eSplit;
						br = true;
						break;
					}
					tri[pi] = R*Vector3D<>((u==0)?uVal:(v==0)?vVal:eVal,(u==1)?uVal:(v==1)?vVal:eVal,(u==2)?uVal:(v==2)?vVal:eVal);
					avg += Vector3D<>(uVal,vVal,eVal);
				}
				if (br)
					continue;
				avg /= 3;
				const double nu = _A(u,u)*avg[0]+_a[u];
				const double nv = _A(v,v)*avg[1]+_a[v];
				const double ne = _A(e,e)*avg[2]+_a[e];
				Vector3D<> normal;
				if (!zero)
					normal = R*Vector3D<>((u==0)?nu:(v==0)?nv:ne,(u==1)?nu:(v==1)?nv:ne,(u==2)?nu:(v==2)?nv:ne);
				else if (place == FRONT)
					normal = R*Vector3D<>((e==0)?1:0,(e==1)?1:0,(e==2)?1:0);
				else if (place == BACK)
					normal = R*Vector3D<>((e==0)?-1:0,(e==1)?-1:0,(e==2)?-1:0);
				if (dot(tri.calcFaceNormal(),normal) < 0) {
					mesh->add(TriangleN1<>(tri[0],tri[2],tri[1],normal));
				} else {
					mesh->add(TriangleN1<>(tri[0],tri[1],tri[2],normal));
				}
			}
		}
	}
}

std::pair<double,double> QuadraticSurface::extremumsDiagonal(const Vector3D<>& dir) const {
	std::size_t u = 3;
	std::size_t v = 3;
	std::size_t e = 3;
	if (std::fabs(_A(2,2)) < EPS && std::fabs(_a[2]) > EPS) {
		u = 0;
		v = 1;
		e = 2;
	} else if (std::fabs(_A(1,1)) < EPS && std::fabs(_a[1]) > EPS) {
		u = 2;
		v = 0;
		e = 1;
	} else if (std::fabs(_A(0,0)) < EPS && std::fabs(_a[0]) > EPS) {
		u = 1;
		v = 2;
		e = 0;
	} else if (std::fabs(_A(2,2)) >= EPS) {
		u = 0;
		v = 1;
		e = 2;
	} else if (std::fabs(_A(1,1)) >= EPS) {
		u = 2;
		v = 0;
		e = 1;
	} else if (std::fabs(_A(0,0)) >= EPS) {
		u = 1;
		v = 2;
		e = 0;
	}
	if (u > 2 || v > 2 || e > 2)
		RW_THROW("Error happened. Could not choose explicit variable!");

	std::pair<double,double> extremums;
	extremums.first = -std::numeric_limits<double>::max();
	extremums.second = std::numeric_limits<double>::max();
	if (std::fabs(_A(e,e)) < EPS && std::fabs(_a[e]) > EPS) {
		if (std::fabs(dir[e]) < EPS) {
			// Skipping due to e zero element.
			return extremums;
		}
		if (std::fabs(_A(u,u)) < EPS || std::fabs(_A(v,v)) < EPS) {
			// - No extremum
		} else {
			if (_A(u,u)*_A(v,v) < 0) {
				// sadle - not extremum
			} else {
				const double xu = (dir[u]*_a[e]-dir[e]*_a[u])/(dir[e]*_A(u,u));
				const double xv = (dir[v]*_a[e]-dir[e]*_a[v])/(dir[e]*_A(v,v));
				const double xe = -(_A(u,u)*xu*xu+_A(v,v)*xv*xv+_a[u]*xu*2+_a[v]*xv*2+_u)/(_a[e]*2);
				const Vector3D<> P = Vector3D<>((u==0)?xu:(v==0)?xv:xe,(u==1)?xu:(v==1)?xv:xe,(u==2)?xu:(v==2)?xv:xe);
				if (insideTrimmingRegion(P)) {
					const double extremum = dir[u]*xu+dir[v]*xv+dir[e]*xe;
					if (dir[e]*_A(u,u)/_a(e) > 0) {
						extremums.second = extremum;
					} else {
						extremums.first = extremum;
					}
				}
			}
		}
	} else if (std::fabs(_A(e,e)) >= EPS) {
		std::vector<Vector3D<> > x;
		if (((_A(u,u) >= EPS) || (_A(u,u) < EPS && _a[u] < EPS)) && ((_A(v,v) >= EPS) || (_A(v,v) < EPS && _a[v] < EPS))) {
			if (std::fabs(dir[u]) < EPS && std::fabs(dir[v]) < EPS) {
				const double xu = (_A(u,u) < EPS) ? 0 : -_a[u]/_A(u,u);
				const double xv = (_A(v,v) < EPS) ? 0 : -_a[v]/_A(v,v);
				double s = _a[e]/_A(e,e)*_a[e]/_A(e,e)-(_A(u,u)*xu*xu+_A(v,v)*xv*xv+_a[u]*xu*2+_a[v]*xv*2+_u)/_A(e,e);
				if (s < 0)
					RW_THROW("What should happen here?");
				const double xe1 = -_a[e]/_A(e,e)+std::sqrt(s);
				const double xe2 = -_a[e]/_A(e,e)-std::sqrt(s);
				x.push_back(Vector3D<>(xu,xv,xe1));
				x.push_back(Vector3D<>(xu,xv,xe2));
			} else {
				const double Auv = _A(u,u)*_A(v,v);
				const double K1 = _a[e]*_a[e]*Auv + _A(e,e)*(_A(u,u)*_a[v]*_a[v] + _a[u]*_a[u]*_A(v,v) - Auv*_u);
				const double K2 = Auv*dir[e]*dir[e] + _A(e,e)*_A(v,v)*dir[u]*dir[u] + _A(e,e)*_A(u,u)*dir[v]*dir[v];
				const double su = (_A(u,u) < EPS) ? 0 : dir[u]*dir[u]*K1/(_A(u,u)*_A(u,u)*K2);
				const double sv = (_A(v,v) < EPS) ? 0 : dir[v]*dir[v]*K1/(_A(v,v)*_A(v,v)*K2);
				if (su >= 0 && sv >= 0) {
					const double xu1 = (_A(u,u) < EPS) ? 0 : -_a[u]/_A(u,u)+std::sqrt(su);
					const double xu2 = (_A(u,u) < EPS) ? 0 : -_a[u]/_A(u,u)-std::sqrt(su);
					const double xv1 = (_A(v,v) < EPS) ? 0 : -_a[v]/_A(v,v)+std::sqrt(sv);
					const double xv2 = (_A(v,v) < EPS) ? 0 : -_a[v]/_A(v,v)-std::sqrt(sv);
					const double s1 = _a[e]/_A(e,e)*_a[e]/_A(e,e)-(_A(u,u)*xu1*xu1+_A(v,v)*xv1*xv1+_a[u]*xu1*2+_a[v]*xv1*2+_u)/_A(e,e);
					const double s2 = _a[e]/_A(e,e)*_a[e]/_A(e,e)-(_A(u,u)*xu2*xu2+_A(v,v)*xv2*xv2+_a[u]*xu2*2+_a[v]*xv2*2+_u)/_A(e,e);
					if (s1 > 0) {
						const double xe1a = -_a[e]/_A(e,e)+std::sqrt(s1);
						const double xe1b = -_a[e]/_A(e,e)-std::sqrt(s1);
						x.push_back(Vector3D<>(xu1,xv1,xe1a));
						x.push_back(Vector3D<>(xu1,xv1,xe1b));
					}
					if (s2 > 0) {
						const double xe2a = -_a[e]/_A(e,e)+std::sqrt(s2);
						const double xe2b = -_a[e]/_A(e,e)-std::sqrt(s2);
						x.push_back(Vector3D<>(xu2,xv2,xe2a));
						x.push_back(Vector3D<>(xu2,xv2,xe2b));
					}
				}
			}
		}
		for (std::size_t i = 0; i < x.size(); i++) {
			const double xu = x[i][0];
			const double xv = x[i][1];
			const double xe = x[i][2];
			const Vector3D<> P = Vector3D<>((u==0)?xu:(v==0)?xv:xe,(u==1)?xu:(v==1)?xv:xe,(u==2)?xu:(v==2)?xv:xe);
			if (insideTrimmingRegion(P)) {
				const double extremum = dir[u]*xu+dir[v]*xv+dir[e]*xe;
				if (dir[e]*_A(u,u)/_a(e) > 0) {
					extremums.second = extremum;
				} else {
					extremums.first = extremum;
				}
			}
		}
	}
	return extremums;
}

QuadraticSurface::Ptr QuadraticSurface::makeEllipsoid(const double a, const double b, const double c) {
	const Eigen::DiagonalMatrix<double,3> A(b*b*c*c, a*a*c*c, a*a*b*b);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	const double u = -a*a*b*b*c*c;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeSpheroid(const double a, const double b) {
	const Eigen::DiagonalMatrix<double,3> A(b*b, b*b, a*a);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	const double u = -a*a*b*b;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeSphere(const double radius) {
	static const Eigen::DiagonalMatrix<double,3> A(1, 1, 1);
	static const Eigen::Vector3d a = Eigen::Vector3d::Zero();
	const double u = -radius*radius;
	return ownedPtr(new QuadraticSurface(A, a, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeEllipticParaboloid(const double a, const double b) {
	const Eigen::DiagonalMatrix<double,3> A(b*b, a*a, 0);
	const Eigen::Vector3d avec(0,0,-a*a*b*b);
	static const double u = 0;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeCircularParaboloid(const double a) {
	static const Eigen::DiagonalMatrix<double,3> A(1, 1, 0);
	const Eigen::Vector3d avec(0,0,-a*a);
	static const double u = 0;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeHyperbolicParaboloid(const double a, const double b) {
	const Eigen::DiagonalMatrix<double,3> A(b*b, -a*a, 0);
	const Eigen::Vector3d avec(0,0,-a*a*b*b);
	static const double u = 0;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeEllipticHyperboloidOneSheet(const double a, const double b, const double c) {
	const Eigen::DiagonalMatrix<double,3> A(b*b*c*c, a*a*c*c, -a*a*b*b);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	const double u = -a*a*b*b*c*c;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeCircularHyperboloidOneSheet(const double a, const double b) {
	const Eigen::DiagonalMatrix<double,3> A(b*b, a*a, -a*a);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	static const double u = -a*a*b*b;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeEllipticHyperboloidTwoSheets(const double a, const double b, const double c) {
	const Eigen::DiagonalMatrix<double,3> A(b*b*c*c, a*a*c*c, -a*a*b*b);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	static const double u = a*a*b*b*c*c;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeCircularHyperboloidTwoSheets(const double a, const double b) {
	const Eigen::DiagonalMatrix<double,3> A(b*b, a*a, -a*a);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	static const double u = a*a*b*b;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeEllipticCone(const double a, const double b, const double c) {
	const Eigen::DiagonalMatrix<double,3> A(b*b*c*c, a*a*c*c, -a*a*b*b);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	static const double u = 0;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeCircularCone(const double a, const double b) {
	const Eigen::DiagonalMatrix<double,3> A(b*b, b*b, -a*a);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	static const double u = 0;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeEllipticCylinder(const double a, const double b) {
	const Eigen::DiagonalMatrix<double,3> A(b*b, a*a, 0);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	static const double u = -a*a*b*b;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeCircularCylinder(const double radius) {
	static const Eigen::DiagonalMatrix<double,3> A(1, 1, 0);
	static const Eigen::Vector3d a = Eigen::Vector3d::Zero();
	const double u = -radius*radius;
	return ownedPtr(new QuadraticSurface(A, a, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeHyperbolicCylinder(const double a, const double b) {
	const Eigen::DiagonalMatrix<double,3> A(b*b, -a*a, 0);
	static const Eigen::Vector3d avec = Eigen::Vector3d::Zero();
	static const double u = -a*a*b*b;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makeParabolicCylinder(const double a) {
	static const Eigen::DiagonalMatrix<double,3> A(1, 0, 0);
	const Eigen::Vector3d avec(0, a*2, 0);
	static const double u = 0;
	return ownedPtr(new QuadraticSurface(A, avec, u));
}

QuadraticSurface::Ptr QuadraticSurface::makePlane(const Vector3D<>& n, double d) {
	static const Eigen::DiagonalMatrix<double,3> A(0, 0, 0);
	return ownedPtr(new QuadraticSurface(A, Eigen::Vector3d(n[0], n[1], n[2]), d*2));
}
