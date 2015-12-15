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

#include "RWPELinearOptimizer.hpp"
#include "RWPELogUtil.hpp"

using namespace rwsimlibs::rwpe;

RWPELinearOptimizer::RWPELinearOptimizer(double alpha, double alphaIn, double betaB, double betaX, double gammaMax, double epsilon, double svdPrecision):
	_alpha(alpha),
	_alphaIn(alphaIn),
	_betaB(betaB),
	_betaX(betaX),
	_gammaMaxSq(gammaMax*gammaMax),
	_epsilon(epsilon),
	_svdPrecision(svdPrecision)
{
}

RWPELinearOptimizer::~RWPELinearOptimizer() {
}

RWPELinearOptimizer::Vector RWPELinearOptimizer::optimize(
	const Matrix& A, const Vector& b,
	unsigned int Ne,
	unsigned int iterations,
	double eps,
	RWPELogUtil* log) const
{
	const bool doLog = (log == NULL)? false : log->doLog();
	const Matrix::Index N = A.rows();
	RW_ASSERT(N > 0);

	if (doLog) {
		log->log("Equation System",RWPE_LOCATION) << A << "\n\n" << b;
	}

	// Do a full SVD
	const Eigen::JacobiSVD<Matrix> svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	const double tolerance = _svdPrecision * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs().maxCoeff();
	const Matrix& U = svd.matrixU();
	const Eigen::ArrayXd Wdiag = (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array(), 0);
	const int K = (Wdiag != 0).count(); // the rank
	const Matrix& V = svd.matrixV();

	if (K == N) {
		// Full rank - note we can not be sure that all elements of sX becomes negative
		const Vector WsDiag(Wdiag);
		const Eigen::DiagonalWrapper<const Vector> Ws = WsDiag.asDiagonal();
		const Vector sX = V*Ws.inverse()*U.adjoint()*b;
		bool validSol = true;
		for (unsigned int i = Ne; i < N; i++) {
			if (sX[i] > 0) {
				validSol = false;
				break;
			}
		}
		if (validSol)
			return sX;
	}

	// Extract submatrices in range and null space
	const Eigen::Block<const Matrix> Us = U.topLeftCorner(N,K);
	const Eigen::ArrayXd WdiagHead = Wdiag.head(K);
	const Vector WsDiag(WdiagHead);
	const Eigen::DiagonalWrapper<const Vector> Ws = WsDiag.asDiagonal();
	const Matrix VT = V.transpose();
	const Eigen::Block<const Matrix> Vs = V.topLeftCorner(N,K);
	const Eigen::Block<const Matrix> VsT = VT.topLeftCorner(K,N);
	const Eigen::Block<const Matrix> VsP = V.bottomRightCorner(N,N-K);
	const Eigen::Block<const Matrix> VsPT = VT.bottomRightCorner(N-K,N);

	// Constant values
	const Matrix Gs = Us*Ws;
	const Matrix GsT = Ws*Us.transpose();

	// Initialize
	Vector phi = Vector::Zero(K);
	Vector phiP = Vector::Zero(N-K);
	Vector sB = -b;
	Vector sX = Vector::Zero(N);
	/*Eigen::VectorXd xViolation = sX.cwiseMax(zeroVecN);
	for (Eigen::MatrixXd::Index i = 0; i < Ne; i++) {
		xViolation[i] = 0;
	}*/
	//double fObj = 0.5*(sB.dot(sB)+xViolation.dot(xViolation));
	double fObj = std::numeric_limits<double>::max();
	double a = _alpha;
	double ai = _alphaIn;
	double bB = _betaB;
	double bX = _betaX;
	unsigned int k;
	if (doLog)
		log->beginSection("Iterative Algorithm",RWPE_LOCATION);
	for (k = 0; k < iterations && fObj > eps; k++) {
		if (fObj <= _epsilon && a > 0) {
			a = ai = bB = bX = 0;
			if (doLog)
				log->log("Additional objectives removed",RWPE_LOCATION) << "At iteration " << k;
		}

		// Impulse
		/*
		xViolation = sX.cwiseMax(zeroVecN);
		for (Eigen::MatrixXd::Index i = 0; i < Ne; i++) {
			xViolation[i] = 0;
		}

		const Eigen::VectorXd dsX = -xViolation+a*(-sX).cwiseMax(zeroVecN);
		const Eigen::VectorXd dsB = -sB;
		*/

		// Constraint
		/*
		Eigen::VectorXd dsB = -sB.cwiseMax(zeroVecN);
		Eigen::VectorXd dsX = -sX.cwiseMax(zeroVecN);
		if (a > 0) {
			dsB += a*(-sB).cwiseMax(zeroVecN);
			dsX += a*(-sX).cwiseMax(zeroVecN);
		}
		*/

		Vector dsB = Vector::Zero(N);
		Vector dsX = Vector::Zero(N);

		// Equality constraints
		for (unsigned int i = 0; i < Ne; i++) {
			dsB[i] -= sB[i];
		}
		// Minimise forces/impulses for equality constraints (for redundant configurations)
		if (_alpha > 0) {
			for (unsigned int i = 0; i < Ne; i++) {
				dsX[i] -= _alpha*sX[i];
			}
		}
		// Inequality constraints (non-penetration)
		for (unsigned int i = Ne; i < N; i++) {
			dsB[i] -= std::max<double>(sB[i],0);
		}
		// Ensure x < 0 (non-attracting impulses or forces)
		const double alphaComb = _alpha + _alphaIn;
		if (alphaComb > 0) {
			for (unsigned int i = Ne; i < N; i++) {
				dsX[i] -= alphaComb*std::max<double>(sX[i],0);
			}
		}
		// Keep close to inequality
		if (_betaB > 0) {
			for (unsigned int i = Ne; i < N; i++) {
				dsB[i] += _betaB*std::max<double>(-sB[i],0);
			}
		}
		const double betaComb = _betaX*alphaComb;
		if (betaComb > 0) {
			for (unsigned int i = Ne; i < N; i++) {
				dsX[i] += betaComb*std::max<double>(-sX[i],0);
			}
		}

		Vector GammaDiagSq = Vector::Constant(N,_gammaMaxSq);
		bool simple = true;
		for (Matrix::Index i = Ne; i < N; i++) {
			if (sB[i] > 0) {
				GammaDiagSq[i] = GammaDiagSq[i]*std::min(1.,dsB.norm()/sB[i]);
				simple = false;
			}
		}
		const Eigen::DiagonalWrapper<const Vector> GammaSq = GammaDiagSq.asDiagonal();
		Vector dPhi;
		if (simple) {
			const Vector BinvDiag( (WdiagHead.array().square()*(_gammaMaxSq) + 1).inverse() );
			const Eigen::DiagonalWrapper<const Vector> Binv = BinvDiag.asDiagonal();
			dPhi = Binv*(GsT*GammaSq*dsB+VsT*dsX);
		} else {
			const Matrix B = Matrix::Identity(K,K)+GsT*GammaSq*Gs;
			const Matrix Binv = B.inverse();
			dPhi = Binv*(GsT*GammaSq*dsB+VsT*dsX);
		}

		const Vector dPhiP = VsPT*dsX;
		if (doLog) {
			std::ostream& lstr = log->log("Info",RWPE_LOCATION);
			lstr << "dsX: " << dsX << "\n";
			lstr << "dsB: " << dsB << "\n";
			lstr << "dPhi: " << dPhi << "\n";
			lstr << "dPhiP: " << dPhiP << "\n";
		}
		phi += dPhi;
		phiP += dPhiP;
		sB += Gs*dPhi;
		sX += Vs*dPhi+VsP*dPhiP;
		fObj = dPhi.norm()+dPhiP.norm();
		if (doLog) {
			std::vector<std::string> labels;
			std::vector<double> values;
			labels.push_back("Linear Optimizer Iteration");
			labels.push_back("Linear Optimizer Precision");
			values.push_back(k);
			values.push_back(fObj);
			log->addValues("Iteration", values, labels, RWPE_LOCATION);
		}
		if (doLog) {
			log->log("sX",RWPE_LOCATION) << sX;
		}
	}
	if (doLog)
		log->endSection(__LINE__);
	if (doLog) {
		std::vector<std::string> labels;
		std::vector<double> values;
		labels.push_back("Linear Optimizer Iterations");
		labels.push_back("Linear Optimizer Max Iterations");
		labels.push_back("Linear Optimizer Precision");
		labels.push_back("Linear Optimizer Precision Target");
		values.push_back(k);
		values.push_back(iterations);
		values.push_back(fObj);
		values.push_back(eps);
		log->addValues("Iterative Solver", values, labels, RWPE_LOCATION);
	}
	return sX;
}
