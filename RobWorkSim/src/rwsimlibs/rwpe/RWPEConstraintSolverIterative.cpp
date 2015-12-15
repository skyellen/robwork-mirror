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

#include <boost/foreach.hpp>

//#include <Eigen/SVD>
#include "RWPEConstraintSolverIterative.hpp"
#include "RWPELogUtil.hpp"
#include "RWPELinearOptimizer.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwsimlibs::rwpe;

#define PROPERTY_SVD_PRECISION "RWPEConstraintSolverSVDPrecision"
#define PROPERTY_ITERATIVESVD_PRECISION "RWPEConstraintSolverIterativePrecision"
#define PROPERTY_ITERATIVESVD_ITERATIONS "RWPEConstraintSolverIterativeIterations"
#define PROPERTY_ITERATIVESVD_ALPHA "RWPEConstraintSolverIterativeAlpha"
#define PROPERTY_ITERATIVESVD_ALPHA_THRESHOLD "RWPEConstraintSolverIterativeAlphaThreshold"

RWPEConstraintSolverIterative::RWPEConstraintSolverIterative():
	RWPEConstraintSolver(NULL,Vector3D<>::zero())
{
}

RWPEConstraintSolverIterative::RWPEConstraintSolverIterative(const RWPEBodyConstraintGraph* manager, const Vector3D<double> &gravity):
	RWPEConstraintSolver(manager,gravity)
{
}

RWPEConstraintSolverIterative::~RWPEConstraintSolverIterative() {
}

const RWPEConstraintSolver* RWPEConstraintSolverIterative::createSolver(const RWPEBodyConstraintGraph* manager, const Vector3D<double> &gravity) const {
	return new RWPEConstraintSolverIterative(manager,gravity);
}

Eigen::VectorXd RWPEConstraintSolverIterative::solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
		Eigen::MatrixXd::Index constraintDim,
		const PropertyMap& pmap, RWPELogUtil* log) const
{
	static const double gammaMax = 2500;

	if (_manager == NULL)
		RW_THROW("RWPEConstraintSolverSVD (solve): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");

	//const bool doLog = (log == NULL) ? false : log->doLog();

	// First find the properties to use
	double SVD_PRECISION = pmap.get<double>(PROPERTY_SVD_PRECISION,-1.);
	double PRECISION = pmap.get<double>(PROPERTY_ITERATIVESVD_PRECISION,-1.);
	double ALPHA = pmap.get<double>(PROPERTY_ITERATIVESVD_ALPHA,0.01);
	double ALPHA_THRESHOLD = pmap.get<double>(PROPERTY_ITERATIVESVD_ALPHA_THRESHOLD,0.01);
	int ITERATIONS = pmap.get<int>(PROPERTY_ITERATIVESVD_ITERATIONS,-1.);
	if (SVD_PRECISION < 0 || PRECISION < 0 || ALPHA < 0 || ALPHA_THRESHOLD < 0 || ITERATIONS < 0) {
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
		if (ALPHA_THRESHOLD < 0)
			ALPHA_THRESHOLD = tmpMap.get<double>(PROPERTY_ITERATIVESVD_ALPHA_THRESHOLD,-1.);
		RW_ASSERT(ALPHA_THRESHOLD >= 0);
		if (ITERATIONS < 0)
			ITERATIONS = tmpMap.get<int>(PROPERTY_ITERATIVESVD_ITERATIONS,-1.);
		RW_ASSERT(ITERATIONS > 0);
	}
	RWPELinearOptimizer optimizer(0, 1, ALPHA, ALPHA, gammaMax, ALPHA_THRESHOLD, SVD_PRECISION);
	RWPELinearOptimizer::Vector solution;
	RW_ASSERT(A.rows() == b.rows());
	if (A.rows() > 0) {
		solution = optimizer.optimize(A,b,constraintDim,ITERATIONS,PRECISION,log);
	}
	return solution;
/*
	// Solution
	Eigen::VectorXd solution;
	RW_ASSERT(A.rows() == b.rows());
	if (A.rows() > 0) {
		const Eigen::MatrixXd::Index N = A.rows();

		if (doLog) {
			log->log("Equation System",RWPE_LOCATION) << A << "\n\n" << b;
		}

		// Do a full SVD
		const Eigen::JacobiSVD<Eigen::MatrixXd> svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
		const double tolerance = SVD_PRECISION * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs().maxCoeff();
		const Eigen::MatrixXd& U = svd.matrixU();
		const Eigen::ArrayXd Wdiag = (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array(), 0);
		//const Eigen::ArrayXd WinvDiag = (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0);
		const int K = (Wdiag != 0).count(); // the rank
		//const Eigen::DiagonalWrapper<const Eigen::VectorXd> W = Eigen::VectorXd(Wdiag).asDiagonal();
		const Eigen::MatrixXd& V = svd.matrixV();

		if (K == N) {
			// Full rank
			const Eigen::VectorXd WsDiag(Wdiag);
			const Eigen::DiagonalWrapper<const Eigen::VectorXd> Ws = WsDiag.asDiagonal();
			const Eigen::VectorXd sX = V*Ws.inverse()*U.adjoint()*b;
			if (sX.maxCoeff() <= 0)
				return sX;
		}

		double fObj = 0;
		int k = 0;

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
		//const Eigen::VectorXd BinvDiag( (WdiagHead.array().square() + 1).inverse() );
		//const Eigen::DiagonalWrapper<const Eigen::VectorXd> Binv = BinvDiag.asDiagonal();
		const Eigen::MatrixXd Gs = Us*Ws;
		const Eigen::MatrixXd GsT = Ws*Us.transpose();
		const Eigen::VectorXd zeroVecN = Eigen::VectorXd::Zero(N);

		// Initialize
		Eigen::VectorXd phi = Eigen::VectorXd::Zero(K);
		Eigen::VectorXd phiP = Eigen::VectorXd::Zero(N-K);
		Eigen::VectorXd sB = -b;
		Eigen::VectorXd sX = Eigen::VectorXd::Zero(N);
		//Eigen::Matrix<double,K,1> test;
		{
			const Eigen::VectorXd bViolation = sB.cwiseMax(zeroVecN);
			const Eigen::VectorXd xViolation = sX.cwiseMax(zeroVecN);
			fObj = 0.5*(bViolation.dot(bViolation)+xViolation.dot(xViolation));
		}
		//double err = 2*PRECISION;
		double a = ALPHA;

		if (doLog)
			log->beginSection("Iterative Algorithm",RWPE_LOCATION);
		for (; k < ITERATIONS && fObj > PRECISION; k++) {
			if (k > 0 && fObj <= ALPHA_THRESHOLD && a > 0) {
				a = 0;
				if (doLog)
					log->log("Alpha set to zero",RWPE_LOCATION) << "At iteration " << k;
			}
			Eigen::VectorXd dsB = -sB.cwiseMax(zeroVecN);
			Eigen::VectorXd dsX = -sX.cwiseMax(zeroVecN);
			if (a > 0) {
				dsB += a*(-sB).cwiseMax(zeroVecN);
				dsX += a*(-sX).cwiseMax(zeroVecN);
			}

			Eigen::VectorXd GammaDiagSq = Eigen::VectorXd::Constant(N,gammaMax*gammaMax);
			bool simple = true;
			for (Eigen::MatrixXd::Index i = constraintDim; i < N; i++) {
				if (sB[i] < 0) {
					GammaDiagSq[i] = GammaDiagSq[i]*std::min(1.,-dsB.norm()/sB[i]);
					simple = false;
				}
			}
			const Eigen::DiagonalWrapper<const Eigen::VectorXd> GammaSq = GammaDiagSq.asDiagonal();
			Eigen::VectorXd dPhi;
			if (simple) {
				if (doLog)
					log->log("SIMPLE",RWPE_LOCATION);
				const Eigen::VectorXd BinvDiag( (WdiagHead.array().square()*(gammaMax*gammaMax) + 1).inverse() );
				const Eigen::DiagonalWrapper<const Eigen::VectorXd> Binv = BinvDiag.asDiagonal();
				dPhi = Binv*(GsT*GammaSq*dsB+VsT*dsX);
			} else {
				if (doLog)
					log->log("!SIMPLE",RWPE_LOCATION);
				const Eigen::MatrixXd B = Eigen::MatrixXd::Identity(K,K)+GsT*GammaSq*Gs;
				const Eigen::MatrixXd Binv = B.inverse();
				dPhi = Binv*(GsT*GammaSq*dsB+VsT*dsX);
			}

			const Eigen::VectorXd dPhiP = VsPT*dsX;
			phi += dPhi;
			phiP += dPhiP;
			sB += Gs*dPhi;
			sX.noalias() += Vs*dPhi+VsP*dPhiP;
			double minForce = std::max(0.01, -sX.maxCoeff());
			fObj = (dPhi.norm()+dPhiP.norm())/minForce;
			if (doLog)
				log->log("sX",RWPE_LOCATION) << sX.transpose() << ".";
		}

		if (doLog) {
			log->endSection(__LINE__);

			std::vector<std::string> labels;
			std::vector<double> values;
			labels.push_back("Solver Iterations");
			labels.push_back("Solver Max Iterations");
			labels.push_back("Solver Precision");
			labels.push_back("Solver Precision Target");
			values.push_back(k);
			values.push_back(ITERATIONS);
			values.push_back(fObj);
			values.push_back(PRECISION);
			log->addValues("Iterative SVD", values, labels, RWPE_LOCATION);
		}
		solution = sX;
		if (doLog) {
			const Eigen::VectorXd residual = A*solution-b;
			//log->addEquationSystem("Equation System", A, b, RWPE_LOCATION);
			log->log(RWPE_LOCATION) << "Solution: " << solution.transpose() << ".";
			log->log(RWPE_LOCATION) << "-B: " << (-b).transpose() << ".";
			log->log(RWPE_LOCATION) << "Residual: " << residual.transpose() << ".";
			std::vector<std::string> labels;
			std::vector<double> values;
			labels.push_back("Iterations");
			values.push_back(k);
			labels.push_back("Iterations Max");
			values.push_back(ITERATIONS);
			labels.push_back("Force & Torque Error");
			values.push_back(fObj);
			labels.push_back("Precision Target");
			values.push_back(PRECISION);
			labels.push_back("Equation System Rows");
			values.push_back(A.rows());
			labels.push_back("Equation System Columns");
			values.push_back(A.cols());
			labels.push_back("Equation System Rank");
			values.push_back(K);
			log->addValues("Iterative SVD Algorithm",values,labels,RWPE_LOCATION);
		}
	} else
		solution = Eigen::VectorXd::Zero(0);
	return solution;
	*/
}

void RWPEConstraintSolverIterative::addDefaultProperties(PropertyMap& map) const {
	RWPEConstraintSolver::addDefaultProperties(map);
	map.add<double>(PROPERTY_SVD_PRECISION,"Precision of SVD - used for LinearAlgebra::pseudoInverse.",1e-6);
	map.add<double>(PROPERTY_ITERATIVESVD_PRECISION,"Precision of the method.",1e-4);
	map.add<int>(PROPERTY_ITERATIVESVD_ITERATIONS,"The maximum number of iterations of the method.",200);
	map.add<double>(PROPERTY_ITERATIVESVD_ALPHA,"The alpha parameter which adds the objective of small forces and torques.",0.01);
	map.add<double>(PROPERTY_ITERATIVESVD_ALPHA_THRESHOLD,"If this precision is achieved, alpha is set to zero.",1e-2);
}
