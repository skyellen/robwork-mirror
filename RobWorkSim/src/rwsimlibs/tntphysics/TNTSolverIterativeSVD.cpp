/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TNTSolverIterativeSVD.hpp"
#include "TNTSettings.hpp"

#include <boost/foreach.hpp>

#include <Eigen/SVD>

using namespace rw::common;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_SVD_PRECISION "TNTSolverSVDPrecision"
#define PROPERTY_ITERATIVESVD_PRECISION "TNTSolverIterativeSVDPrecision"
#define PROPERTY_ITERATIVESVD_ITERATIONS "TNTSolverIterativeSVDIterations"
#define PROPERTY_ITERATIVESVD_ALPHA "TNTSolverIterativeSVDAlpha"
#define PROPERTY_ENABLE_DEBUG "TNTSolverDebug"

TNTSolverIterativeSVD::TNTSolverIterativeSVD():
	TNTSolver(NULL,Vector3D<>::zero())
{
}

TNTSolverIterativeSVD::TNTSolverIterativeSVD(const TNTBodyConstraintManager* manager, const Vector3D<double> &gravity):
	TNTSolver(manager,gravity)
{
}

TNTSolverIterativeSVD::~TNTSolverIterativeSVD() {
}

const TNTSolver* TNTSolverIterativeSVD::createSolver(const TNTBodyConstraintManager* manager, const rw::math::Vector3D<double> &gravity) const {
	return new TNTSolverIterativeSVD(manager,gravity);
}

Eigen::VectorXd TNTSolverIterativeSVD::solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const PropertyMap& pmap) const {
	if (_manager == NULL)
		RW_THROW("TNTSolverSVD (solve): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");

	// First find the properties to use
	double SVD_PRECISION = pmap.get<double>(PROPERTY_SVD_PRECISION,-1.);
	double PRECISION = pmap.get<double>(PROPERTY_ITERATIVESVD_PRECISION,-1.);
	double ALPHA = pmap.get<double>(PROPERTY_ITERATIVESVD_ALPHA,0.01);
	int ITERATIONS = pmap.get<int>(PROPERTY_ITERATIVESVD_ITERATIONS,-1.);
	int DEBUG = pmap.get<int>(PROPERTY_ENABLE_DEBUG,-1.);
#if !TNT_DEBUG_ENABLE_SOLVER
	DEBUG = 0;
#endif
	if (SVD_PRECISION < 0 || PRECISION < 0 || ALPHA < 0 || ITERATIONS < 0 || (DEBUG != 0 && DEBUG != 1)) {
		PropertyMap tmpMap;
		addDefaultProperties(tmpMap);
		if (SVD_PRECISION < 0)
			SVD_PRECISION = tmpMap.get<double>(PROPERTY_SVD_PRECISION,-1.);
		RW_ASSERT(SVD_PRECISION > 0);
		if (PRECISION < 0)
			PRECISION = tmpMap.get<double>(PROPERTY_ITERATIVESVD_PRECISION,-1.);
		RW_ASSERT(PRECISION > 0);
		if (ALPHA < 0)
			ALPHA = tmpMap.get<double>(PROPERTY_ITERATIVESVD_ALPHA,-1.);
		RW_ASSERT(ALPHA >= 0);
		if (ITERATIONS < 0)
			ITERATIONS = tmpMap.get<int>(PROPERTY_ITERATIVESVD_ITERATIONS,-1.);
		RW_ASSERT(ITERATIONS > 0);
		if (DEBUG < 0)
			DEBUG = tmpMap.get<int>(PROPERTY_ENABLE_DEBUG,-1.);
		RW_ASSERT(DEBUG == 0 || DEBUG == 1);
	}

	// Solution
	Eigen::VectorXd solution;
	RW_ASSERT(A.rows() == b.rows());
	if (A.rows() > 0) {
		const Eigen::MatrixXd::Index N = A.rows();

		// Do a full SVD
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        const double tolerance = SVD_PRECISION * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs().maxCoeff();
        const Eigen::MatrixXd& U = svd.matrixU();
        const Eigen::ArrayXd Wdiag = (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array(), 0);
        //const Eigen::ArrayXd WinvDiag = (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0);
        const int K = (Wdiag != 0).count(); // the rank
        //const Eigen::DiagonalWrapper<const Eigen::VectorXd> W = Eigen::VectorXd(Wdiag).asDiagonal();
        const Eigen::MatrixXd& V = svd.matrixV();

        // Extract submatrices in range and null space
        const Eigen::Block<const Eigen::MatrixXd> Us = U.topLeftCorner(N,K);
        //const Eigen::Block<const Eigen::MatrixXd> Ws = W.topLeftCorner(N,N);
        const Eigen::ArrayXd WdiagHead = Wdiag.head(K);
        const Eigen::VectorXd WsDiag(WdiagHead);
        const Eigen::DiagonalWrapper<const Eigen::VectorXd> Ws = WsDiag.asDiagonal();
        const Eigen::MatrixXd VT = V.transpose();
        const Eigen::Block<const Eigen::MatrixXd> Vs = V.topLeftCorner(N,K);
        const Eigen::Block<const Eigen::MatrixXd> VsT = VT.topLeftCorner(K,N);
        const Eigen::Block<const Eigen::MatrixXd> VsP = V.bottomRightCorner(N,N-K);
        const Eigen::Block<const Eigen::MatrixXd> VsPT = VT.bottomRightCorner(N-K,N);

        // Constant values
        //const Eigen::DiagonalWrapper<const Eigen::VectorXd> B = Eigen::VectorXd(WdiagHead.array().square() + 1).asDiagonal();
        const Eigen::VectorXd BinvDiag( (WdiagHead.array().square() + 1).inverse() );
        const Eigen::DiagonalWrapper<const Eigen::VectorXd> Binv = BinvDiag.asDiagonal();
        const Eigen::MatrixXd Gs = Us*Ws;
        const Eigen::MatrixXd GsT = Ws*Us.transpose();
        const Eigen::VectorXd zeroVecN = Eigen::VectorXd::Zero(N);

        // Initialize
        Eigen::VectorXd phi = Eigen::VectorXd::Zero(K);
        Eigen::VectorXd phiP = Eigen::VectorXd::Zero(N-K);
        Eigen::VectorXd sB = -b;
        Eigen::VectorXd sX = Eigen::VectorXd::Zero(N);
        double err = 2*PRECISION;
        int k = 0;
        for (; k < ITERATIONS && err > PRECISION; k++) {
        	const Eigen::VectorXd bViolation = sB.cwiseMax(zeroVecN);
        	const Eigen::VectorXd xViolation = sX.cwiseMax(zeroVecN);
        	const Eigen::VectorXd dsB = -bViolation+ALPHA*(-sB).cwiseMax(zeroVecN);
        	const Eigen::VectorXd dsX = -xViolation+ALPHA*(-sX).cwiseMax(zeroVecN);
        	//const Eigen::VectorXd dsB = -sB;
        	//const Eigen::VectorXd dsX = -ALPHA*sX;
        	const Eigen::VectorXd dPhi = Binv*(GsT*dsB+VsT*dsX);
        	const Eigen::VectorXd dPhiP = VsPT*dsX;
        	phi += dPhi;
        	phiP += dPhiP;
        	sB += Gs*dPhi;
        	sX += Vs*dPhi+VsP*dPhiP;
        	err = dPhi.norm()+dPhiP.norm();
        	std::cout << "iteration " << k << ": " << err << std::endl;
        }
        solution = sX;
		if (DEBUG == 1) {
			const Eigen::VectorXd residual = A*solution-b;
			TNT_DEBUG_SOLVER("Forces & Torques found with error " << err << " after " << k << " of " << ITERATIONS << " iterations.");
			TNT_DEBUG_SOLVER("A dimensions: " << A.rows() << "x" << A.cols() << " - rank " << K << ".");
			TNT_DEBUG_SOLVER("Solution: " << solution.transpose() << ".");
			TNT_DEBUG_SOLVER("-B: " << (-b).transpose() << ".");
			TNT_DEBUG_SOLVER("Residual: " << residual.transpose() << ".");
		}
	} else
		solution = Eigen::VectorXd::Zero(0);
	return solution;
}

void TNTSolverIterativeSVD::addDefaultProperties(PropertyMap& map) const {
	TNTSolver::addDefaultProperties(map);
	map.add<double>(PROPERTY_SVD_PRECISION,"Precision of SVD - used for LinearAlgebra::pseudoInverse.",1e-6);
	map.add<double>(PROPERTY_ITERATIVESVD_PRECISION,"Precision of the method.",1e-6);
	map.add<int>(PROPERTY_ITERATIVESVD_ITERATIONS,"The maximum number of iterations of the method.",20);
	map.add<double>(PROPERTY_ITERATIVESVD_ALPHA,"The alpha parameter which adds the objective of small forces and torques.",0.01);
	map.add<int>(PROPERTY_ENABLE_DEBUG,"Enable or disable debugging (really slow).",0);
}
