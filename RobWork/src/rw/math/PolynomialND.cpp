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

#include "PolynomialND.hpp"

using rw::math::PolynomialND;

PolynomialND<Eigen::Vector3d> rw::math::operator*(const PolynomialND<Eigen::Matrix3d>& A, const PolynomialND<Eigen::Vector3d>& b) {
	return A.multiply<Eigen::Vector3d,Eigen::Vector3d>(b);
}

PolynomialND<Eigen::Matrix<double,1,3> > rw::math::operator*(const PolynomialND<Eigen::Matrix<double,1,3> >& a, const PolynomialND<Eigen::Matrix3d>& A) {
	return a.multiply<Eigen::Matrix<double,1,3>,Eigen::Matrix3d>(A);
}

PolynomialND<Eigen::Vector3d> rw::math::operator*(const PolynomialND<Eigen::Matrix3d>& A, const Eigen::Vector3d& b) {
	return A.multiply<Eigen::Vector3d,Eigen::Vector3d>(b);
}

PolynomialND<Eigen::Matrix<double,1,3> > rw::math::operator*(const PolynomialND<Eigen::Matrix<double,1,3> >& a, const Eigen::Matrix3d& A) {
	return a.multiply<Eigen::Matrix<double,1,3>,Eigen::Matrix3d>(A);
}

PolynomialND<Eigen::Vector3f,float> rw::math::operator*(const PolynomialND<Eigen::Matrix3f,float>& A, const PolynomialND<Eigen::Vector3f,float>& b) {
	return A.multiply<Eigen::Vector3f,Eigen::Vector3f>(b);
}

PolynomialND<Eigen::Matrix<float,1,3>,float> rw::math::operator*(const PolynomialND<Eigen::Matrix<float,1,3>,float>& a, const PolynomialND<Eigen::Matrix3f,float>& A) {
	return a.multiply<Eigen::Matrix<float,1,3>,Eigen::Matrix3f>(A);
}

PolynomialND<Eigen::Vector3f,float> rw::math::operator*(const PolynomialND<Eigen::Matrix3f,float>& A, const Eigen::Vector3f& b) {
	return A.multiply<Eigen::Vector3f,Eigen::Vector3f>(b);
}

PolynomialND<Eigen::Matrix<float,1,3>,float> rw::math::operator*(const PolynomialND<Eigen::Matrix<float,1,3>,float>& a, const Eigen::Matrix3f& A) {
	return a.multiply<Eigen::Matrix<float,1,3>,Eigen::Matrix3f>(A);
}
