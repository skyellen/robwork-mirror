/*
 * MultivariateNormalDistribution.hpp
 *
 *  Created on: Oct 6, 2012
 *      Author: bing
 */

#ifndef MULTIVARIATENORMALDISTRIBUTION_HPP_
#define MULTIVARIATENORMALDISTRIBUTION_HPP_

#include <boost/random.hpp>
#include <Eigen/Cholesky>

template<class T, size_t N>
class MultivariateNormalDistribution {
public:
	MultivariateNormalDistribution(unsigned int seed = time(0));

	MultivariateNormalDistribution(const Eigen::Matrix<T, N, N>& covarianceMatrix, unsigned int seed = time(0));

	virtual ~MultivariateNormalDistribution();

	void setCovarianceMatrix(const Eigen::Matrix<T, N, N>& covarianceMatrix);

	Eigen::Matrix<T, N, 1> draw();

	Eigen::Matrix<T, N, 1> draw(const Eigen::Matrix<T, N, N>& covarianceMatrix);

private:
	Eigen::Matrix<T, N, 1> draw(const Eigen::LLT<Eigen::Matrix<T, N, N> >& llt);

private:
	Eigen::Matrix<T, N, N> covarianceMatrix_;
	Eigen::LLT<Eigen::Matrix<T, N, N> > llt_;
	boost::mt19937 rng_;
};

template<class T, size_t N>
MultivariateNormalDistribution<T, N>::MultivariateNormalDistribution(unsigned int seed) :
		covarianceMatrix_(Eigen::Matrix<T, N, N>::Identity()), llt_(Eigen::Matrix<T, N, N>::Identity()), rng_(seed) {

}

template<class T, size_t N>
MultivariateNormalDistribution<T, N>::MultivariateNormalDistribution(const Eigen::Matrix<T, N, N>& covarianceMatrix, unsigned int seed) :
		covarianceMatrix_(covarianceMatrix), llt_(covarianceMatrix), rng_(seed) {

}

template<class T, size_t N>
MultivariateNormalDistribution<T, N>::~MultivariateNormalDistribution() {

}

template<class T, size_t N>
void MultivariateNormalDistribution<T, N>::setCovarianceMatrix(const Eigen::Matrix<T, N, N>& covarianceMatrix) {
	covarianceMatrix_ = covarianceMatrix;
	llt_ = Eigen::LLT<Eigen::Matrix<T, N, N> >(covarianceMatrix);
}

template<class T, size_t N>
Eigen::Matrix<T, N, 1> MultivariateNormalDistribution<T, N>::draw() {
	return draw(llt_);
}

template<class T, size_t N>
Eigen::Matrix<T, N, 1> MultivariateNormalDistribution<T, N>::draw(const Eigen::Matrix<T, N, N>& covarianceMatrix) {
	return draw(Eigen::LLT<Eigen::Matrix<T, N, N> >(covarianceMatrix));
}

template<class T, size_t N>
Eigen::Matrix<T, N, 1> MultivariateNormalDistribution<T, N>::draw(const Eigen::LLT<Eigen::Matrix<T, N, N> >& llt) {
	// Draw N independent random numbers from standard normal distribution.
	boost::normal_distribution<double> normalDistribution(0.0, 1.0);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > variateGenerator(rng_, normalDistribution);
	Eigen::Matrix<T, N, 1> r(N);
	for (unsigned int i = 0; i < N; i++) {
		r(i) = variateGenerator();
	}

	// Transform numbers to multivariate normal distribution.
	Eigen::Matrix<T, N, 1> z = llt.matrixL() * r;

	return z;
}

#endif /* MULTIVARIATENORMALDISTRIBUTION_HPP_ */
