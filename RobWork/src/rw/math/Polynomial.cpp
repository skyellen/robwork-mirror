/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "Polynomial.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include <rw/math/Math.hpp>

using namespace rw::common;
using namespace rw::math;

namespace rw{ namespace common { namespace serialization {

    template<class T>
    void writeImpl(const rw::math::Polynomial<T>& tmp, rw::common::OutputArchive& oar, const std::string& id){
        const std::vector<double> data = rw::math::Math::toStdVector(tmp, static_cast<int>(tmp.order()+1));
        oar.write(data, id);
    }

    template<class T>
    void readImpl(rw::math::Polynomial<T>& tmp, rw::common::InputArchive& iar, const std::string& id){
        std::vector<T> data;
        iar.read(data, id);
        tmp = Polynomial<T>(data);
    }

    // we need these to explicitly instantiate these functions
    template<> void write(const rw::math::Polynomial<double>& tmp, rw::common::OutputArchive& oar, const std::string& id){writeImpl(tmp,oar,id);}
    template<> void write(const rw::math::Polynomial<float>& tmp, rw::common::OutputArchive& oar, const std::string& id){writeImpl(tmp,oar,id);}
    template<> void read(rw::math::Polynomial<double>& tmp, rw::common::InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);}
    template<> void read(rw::math::Polynomial<float>& tmp, rw::common::InputArchive& iar, const std::string& id){readImpl(tmp,iar,id);}

}}}

Polynomial<> rw::math::operator*(const PolynomialND<Eigen::Matrix<double,1,3> >& a, const PolynomialND<Eigen::Matrix<double,3,1> >& b) {
	return a.template multiply<double,Eigen::Vector3d>(b);
}

PolynomialND<Eigen::Vector3d> rw::math::operator*(const PolynomialND<Eigen::Vector3d>& polynomial, const Polynomial<>& p) {
	return polynomial.template multiply<Eigen::Vector3d>(static_cast<PolynomialND<double> >(p));
}

PolynomialND<Eigen::Matrix<double,1,3> > rw::math::operator*(const PolynomialND<Eigen::Matrix<double,1,3> >& polynomial, const Polynomial<>& p) {
	return polynomial.template multiply<Eigen::Matrix<double,1,3> >(static_cast<PolynomialND<double> >(p));
}

PolynomialND<Eigen::Matrix3d> rw::math::operator*(const PolynomialND<Eigen::Matrix3d >& polynomial, const Polynomial<>& p) {
	return polynomial.template multiply<Eigen::Matrix3d>(static_cast<PolynomialND<double> >(p));
}

Polynomial<float> rw::math::operator*(const PolynomialND<Eigen::Matrix<float,1,3>,float>& a, const PolynomialND<Eigen::Matrix<float,3,1>,float>& b) {
	return a.template multiply<float,Eigen::Vector3f>(b);
}

PolynomialND<Eigen::Vector3f,float> rw::math::operator*(const PolynomialND<Eigen::Vector3f,float>& polynomial, const Polynomial<float>& p) {
	return polynomial.template multiply<Eigen::Vector3f>(static_cast<PolynomialND<float,float> >(p));
}

PolynomialND<Eigen::Matrix<float,1,3>,float> rw::math::operator*(const PolynomialND<Eigen::Matrix<float,1,3>,float>& polynomial, const Polynomial<float>& p) {
	return polynomial.template multiply<Eigen::Matrix<float,1,3> >(static_cast<PolynomialND<float,float> >(p));
}

PolynomialND<Eigen::Matrix3f,float> rw::math::operator*(const PolynomialND<Eigen::Matrix3f,float>& polynomial, const Polynomial<float>& p) {
	return polynomial.template multiply<Eigen::Matrix3f>(static_cast<PolynomialND<float,float> >(p));
}
