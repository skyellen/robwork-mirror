#include "EncoderDecentralization.hpp"
#include <cmath>
//#include <iostream>

double rw::models::EncoderDecentralization::calcRealAngle(const double phi, const double tau, const double sigma)
{
	return phi-(sigma*cos(phi)+tau*sin(phi));
}

rw::math::Q rw::models::EncoderDecentralization::calcRealAngle(const rw::math::Q &phi, const rw::math::Q &tau, const rw::math::Q &sigma)
{
	rw::math::Q theta(phi);

	//Verify dimensions
	if(tau.size() < phi.size() || sigma.size() < phi.size())
	{
		RW_WARN("calcRealAngle -> Wrong dimensions, Need a pair of tau and sigma for each phi. Return theta as phi.");
		return theta;
	}

	//Calculate the real theta angle for each phi
	for(unsigned int i=0; i<phi.size(); ++i)
	{
		theta(i)=calcRealAngle(phi(i), tau(i), sigma(i));
	}

	return theta;
}

double rw::models::EncoderDecentralization::calcEncoderAngle(const double theta, const double tau, const double sigma, const double maxError, const unsigned int maxIterations)
{

	double phi = theta;
	unsigned int iteration = 0;
	//Newtons metode used to find phi = theta + (sigma*cos(phi)+tau*sin(phi))
	do {
		phi = phi-
				(theta+(sigma*cos(phi)+tau*sin(phi)))
				/(tau*cos(phi)-sigma*sin(phi));
		iteration++;
	} while(std::abs(calcRealAngle(phi,tau,sigma)-theta)>maxError && iteration<maxIterations);

//	std::cout << "calcEncoderAngle iterations: " << iteration << " Error: " << std::abs(calcRealAngle(phi,tau,sigma)-theta) << std::endl;
	return phi;
}

rw::math::Q rw::models::EncoderDecentralization::calcEncoderAngle(const rw::math::Q &theta, const rw::math::Q &tau, const rw::math::Q &sigma, const double maxError, const unsigned int maxIterations)
	{
		rw::math::Q phi(theta);

		//Verify dimensions
		if(tau.size() < theta.size() || sigma.size() < theta.size())
		{
			RW_WARN("calcRealAngle -> Wrong dimensions, Need a pair of tau and sigma for each theta. Return phi as theta.");
			return phi;
		}

		//Calculate the real phi angle for each theta
		for(unsigned int i=0; i<theta.size(); ++i)
		{
			phi(i)=calcEncoderAngle(theta(i), tau(i), sigma(i), maxError, maxIterations);
		}

		return phi;
	}




