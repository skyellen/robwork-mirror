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
#include "PointPairsRegistration.hpp" 
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Quaternion.hpp>

#include <boost/foreach.hpp>

using namespace rwlibs::algorithms; 
using namespace rw::math;


rw::math::Transform3D<> PointPairsRegistration::pointPairRegistrationQuaternion(const std::vector<PointPair>& pointPairs )
{
    // 𝜇_𝑏𝑎𝑠𝑒=1/𝑁 ∑_(𝑖=1)^𝑁 𝑃_𝑖^𝑏𝑎𝑠𝑒   and 𝜇_𝑐𝑎𝑚=1/𝑁 ∑_(𝑖=1)^𝑁 𝑃_𝑖^𝑐𝑎𝑚 
    Vector3D<> my_a(0,0,0);
    Vector3D<> my_b(0,0,0);

    BOOST_FOREACH(const PointPair& pp, pointPairs) {
        my_a += pp.first;
        my_b += pp.second;
    }

    my_a /= (double)pointPairs.size();
    my_b /= (double)pointPairs.size();


    // Compute 𝐻=∑_(𝑖=1)^𝑁▒〖 (𝑃_𝑖^𝑏𝑎𝑠𝑒−〖𝜇" " 〗_𝑏𝑎𝑠𝑒)(〖𝑃_𝑖^𝑐𝑎𝑚−〖𝜇" " 〗_𝑐𝑎𝑚)〗^𝑇 〗
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,3);
    BOOST_FOREACH(const PointPair& pp, pointPairs) {
        Vector3D<> a = pp.first - my_a;
        Vector3D<> b = pp.second - my_b;

        for (size_t i = 0; i<3; i++) {
            for (size_t j = 0; j<3; j++) {
                H(i,j) += a(i)*b(j);
            }
        }
    }

    // Compute 𝐴_𝑖𝑗=(𝐻−𝐻^𝑇 ) and ∆=[𝐴_23,𝐴_31,𝐴_12]
    Eigen::Vector3d Delta;
    Delta(0) = H(1,2) - H(2,1);
    Delta(1) = H(2,0) - H(0,2);
    Delta(2) = H(0,1) - H(1,0);
    //𝑄=[(𝑡𝑟(𝐻)      ∆^𝑇 
    //     ∆        𝐻+𝐻^𝑇−𝑡𝑟(𝐻)𝐼)]
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4,4);
    Q(0,0) = H.trace();
    Q(0,1) = Delta(0);
    Q(0,2) = Delta(1);
    Q(0,3) = Delta(2);

    Q(1,0) = Delta(0);
    Q(2,0) = Delta(1);
    Q(3,0) = Delta(2);

    Eigen::MatrixXd HHTHtrace = H + H.transpose() - H.trace()*Eigen::MatrixXd::Identity(3,3);
    for (size_t i = 0; i<3; i++) {
        for (size_t j = 0; j<3; j++) {
            Q(i+1,j+1) = HHTHtrace(i,j);
        }
    }

    //The Eigen vector of Q corresponding to the largest Eigen value gives the quaternion, q, representing the optimal rotation.
    std::pair<typename Eigen::MatrixXcd, typename Eigen::VectorXcd> eigenDecomp = LinearAlgebra::eigenDecomposition(Q);

    Quaternion<> qua(eigenDecomp.first(1,0).real(), eigenDecomp.first(2,0).real(), eigenDecomp.first(3,0).real(), eigenDecomp.first(0,0).real());
    Rotation3D<> rot(qua.toRotation3D());
	rot = inverse(rot);
    //Translation is given by 𝑃=𝜇_𝑐𝑎𝑚−𝑅(𝑞)𝜇_𝑏𝑎𝑠𝑒
    Vector3D<> pos = my_b - rot * my_a;

    return Transform3D<>(pos, rot);

}

rw::math::Transform3D<> PointPairsRegistration::pointPairRegistrationSVD(const std::vector<PointPair>& pointPairs )
{

    ///〖𝜇" " 〗_𝑏𝑎𝑠𝑒=1/𝑁 ∑_(𝑖=1)^𝑁▒𝑃_𝑖^𝑏𝑎𝑠𝑒   and 〖𝜇" " 〗_𝑐𝑎𝑚=1/𝑁 ∑_(𝑖=1)^𝑁▒𝑃_𝑖^𝑐𝑎𝑚 

    Vector3D<> my_a(0,0,0);
    Vector3D<> my_b(0,0,0);

    BOOST_FOREACH(const PointPair& pp, pointPairs) {
        my_a += pp.first;
        my_b += pp.second;
    }

    my_a /= (double)pointPairs.size();
    my_b /= (double)pointPairs.size();

    //Compute 𝐻=∑_(𝑖=1)^𝑁▒〖 (𝑃_𝑖^𝑏𝑎𝑠𝑒−〖𝜇" " 〗_𝑏𝑎𝑠𝑒)(〖𝑃_𝑖^𝑐𝑎𝑚−〖𝜇" " 〗_𝑐𝑎𝑚)〗^𝑇 〗 
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,3);
    BOOST_FOREACH(const PointPair& pp, pointPairs) {
        Vector3D<> a = pp.first - my_a;
        Vector3D<> b = pp.second - my_b;

        for (size_t i = 0; i<3; i++) {
            for (size_t j = 0; j<3; j++) {
                H(i,j) += a(i)*b(j);
            }
        }
    }
    
    //Perform SVD on H: [𝑈,𝑆,𝑉]=𝑆𝑉𝐷(𝐻)
    Eigen::MatrixXd U(3,3);
    Eigen::MatrixXd V(3,3);
    Eigen::VectorXd S(3);

    LinearAlgebra::svd(H, U, S, V);
    
    //Rotation is given by 𝑅=𝑉𝑈^𝑇
    Eigen::Matrix3d rot = V*U.transpose();

	if (rot.determinant() < 0) 
	{
		rot(0, 2) *= -1;
		rot(1, 2) *= -1;
		rot(2, 2) *= -1;
	}
    
    //𝑃=−𝑅〖𝜇" " 〗_𝑏𝑎𝑠𝑒+ 〖𝜇" " 〗_𝑐𝑎𝑚
    Eigen::Vector3d pos = my_b.e() - rot* my_a.e();

    Vector3D<> p(pos);
    Rotation3D<> r(rot);
    return Transform3D<>(p, r);
    

    

}

