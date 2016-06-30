#include "BFGS.hpp"

#include <rw/common/Log.hpp>

using namespace rwlibs::algorithms;

int BFGS::optimizer(
		vector &startguess,
		BFGS_function_struct function,
		double tolerance,
		unsigned int iterationLimit,
		double initialStepsize,
		double c1,
		double c2,
		double alphamax
		)
{
	int STATUS = SUCCESS;
	unsigned int dim = startguess.size();

	//Allocate array for inverse hessian approximation
	matrix Hk(dim,dim);
	matrix Hk1(dim,dim);
	matrix tempLeft(dim,dim);
	matrix tempRight(dim,dim);

	//Initialize H with identity matrix
	for(unsigned int i=0;i<dim;i++)
	{
		for(unsigned int j=0;j<dim;j++)
		{
			Hk(i,j) = 0;
		}
		Hk(i,i) = initialStepsize;
	}

	//Allocate memory for gradient
	vector gfk(dim);
	for(unsigned int i=0;i<dim;i++)
		gfk(i) = 0;

	//Allocate memory for next gradient
	vector gfk1(dim);
	for(unsigned int i=0;i<dim;i++)
		gfk1(i) = 0;

	//Allocate memory for gradient differences
	vector yk(dim);
	for(unsigned int i=0;i<dim;i++)
		yk(i) = 0;

	//Allocate memory for search direction
	vector pk(dim);
	for(unsigned int i=0;i<dim;i++)
		pk(i) = 0;

	//Allocate memory for step
	vector xk(dim);
	xk = startguess;

	//Allocate memory for next step
	vector xk1(dim);
	for(unsigned int i=0;i<dim;i++)
		xk1(i) = 0;

	//Allocate memory for step differences
	vector sk(dim);

	//Allocate memory for tempVec
	vector skrhok(dim);

	//Calculate gradient
	(function.df)(&xk, function.params, &gfk);

	//Start iterations
	unsigned int k=0;
	while( (norm_2(gfk)>tolerance) && (k<iterationLimit) )
	{
		//Compute search direction
		pk = prod(-Hk,gfk);

		//Compute step size

//		chapter 3 pdfside 31
		double alphak = BFGS::lineSearch( function, xk, pk, c1, c2, alphamax);

		xk1 = xk + alphak*pk;

		//Calculate step differences
		sk = xk1 - xk;

		//Calculate next gradient
		(function.df)(&xk1, function.params, &gfk1);

		//Calculate gradient differences
		yk = gfk1 - gfk;

/* Calculate next inverse hessian approximation using formula:
 * 		H_(k+1) = ( I - rhok*sk.yk^T ) Hk (I - rhok*yk.sk^T ) + rhok*sk.sk^T)
 * From Chapter 6 pdfpage 5-6 */

		//Calculate rhok = 1/(yk.sk)
		double ykdotsk = prec_inner_prod(yk,sk);
		if(std::abs(ykdotsk)<1e-12)
		{
			rw::common::Log::debugLog() <<"norm2(y(k)): "<<norm_2(yk)<<" norm2(s(k)): "<<norm_2(sk)<<" yk dot sk: "<<std::abs(ykdotsk)<< "\n";
			STATUS = GRADIENTWARNING;
			break;
		}
		double rhok = 1.0/ykdotsk;

		//Calculate sk*rhok to skrhok
		skrhok = sk;
		skrhok *= rhok;

		//Calculate rhok*sk.yk^T to tempLeft
		colDotRow(skrhok, yk, tempLeft);

		//Calculate rhok*yk.sk^T to tempRight
		colDotRow(yk, skrhok, tempRight);

		//Calculate I - tempLeft and I - tempRight
		for(unsigned int i=0;i<dim;i++)
			for(unsigned int j=0;j<dim;j++)
			{
				if(i==j)
				{
					tempLeft(i,j) = 1 - tempLeft(i,j);
					tempRight(i,j) = 1 - tempRight(i,j);
				}
				else
				{
					tempLeft(i,j) = -tempLeft(i,j);
					tempRight(i,j) = -tempRight(i,j);
				}
			}

		//Calculate tempLeft.Hk to tempLeft
		axpy_prod(tempLeft, Hk, Hk1, true);

		//Calculate Hk1.tempRight to Hk
		axpy_prod (Hk1, tempRight, Hk, true);

		//Calculate rhok*sk.sk^T to tempLeft
		colDotRow(skrhok,sk,tempLeft);

		//Calculate Hk+tempLeft to Hk
		Hk += tempLeft;

		//Iterate
		k = k + 1;
		xk = xk1;
		gfk = gfk1;
	}
	rw::common::Log::debugLog() << "Iterations: "<<k<<" gradientnorm: "<<norm_2(gfk) << "\n";
	rw::common::Log::debugLog() << "Solution x: "<< "\n";
	for(unsigned int i=0;i<xk.size();i++)
		rw::common::Log::debugLog() <<"	"<<xk(i)<< "\n";
	startguess = xk;
	return STATUS;
}

void BFGS::colDotRow(
		vector &colvec,
		vector &rowvec,
		matrix &result)
{
	//Calculate colvec.rowvec
	unsigned int dimRow = result.size1();
	unsigned int dimCol = result.size1();
	for(unsigned int i=0;i<dimRow;i++)
		for(unsigned int j=0;j<dimCol;j++)
			result(i,j) = colvec(i) * rowvec(j);


}

double BFGS::lineSearch(
		BFGS_function_struct function,
		vector &xk,
		vector &pk,
		double c1,
		double c2,
		double alphamax)
{
	double eps = 1e-6;

	//Allocate memory for alphastep
	vector alphastep(xk.size());

	//Allocate memory for alpha, phi(alpha) and gradient of phi(alpha)
	double alpha[3];
	double phi_alpha[3];//index 0->phi(0), 1->phi(i-1), 2->phi(i)
	double dphi_alpha[3];//index 0->phi(0), 1->phi(i-1), 2->phi(i)

	alpha[0] = 0.0;
	phi_alpha[0] = (function.f)(&xk, function.params);
	dphi_alpha[0] = phiGradient(function, xk, pk, alpha[0], alphastep, phi_alpha[0], eps);

	alpha[1] = alpha[0];
	phi_alpha[1] = phi_alpha[0];
	dphi_alpha[1] = dphi_alpha[0];

	alpha[2] = alphamax;//Initial step of max, recommended on page 59
	int i=1;

	while(i<20)
	{
		//Evaluate phi(alpha_i)
		alphastep = pk;
		alphastep *= alpha[2];
		alphastep += xk;
		phi_alpha[2] = (function.f)(&alphastep, function.params);

		if( ( phi_alpha[2] > (phi_alpha[0] + c1*alpha[2]*dphi_alpha[0]) ) || ( (phi_alpha[2] >= phi_alpha[1]) && (i>1) ) )
		{
			return zoom(
					alpha[1],
					alpha[2],
					phi_alpha[1],
					dphi_alpha[1],
					phi_alpha[2],
					function,
					xk,
					pk,
					alphastep,
					phi_alpha[0],
					dphi_alpha[0],
					c1,
					c2,
					eps);
		}

		//Evaluate dphi(alpha_i)
		dphi_alpha[2] = phiGradient(function, xk, pk, alpha[2], alphastep, phi_alpha[2], eps);

		if( std::abs(dphi_alpha[2]) <= -c2*dphi_alpha[0])
		{
			return alpha[2];
		}
		if( dphi_alpha[2] >= 0)
		{
			return zoom(
					alpha[2],
					alpha[1],
					phi_alpha[2],
					dphi_alpha[2],
					phi_alpha[1],
					function,
					xk,
					pk,
					alphastep,
					phi_alpha[0],
					dphi_alpha[0],
					c1,
					c2,
					eps);
		}

		//Iterate
		alpha[1] = alpha[2];
		phi_alpha[1] = phi_alpha[2];
		dphi_alpha[1] = dphi_alpha[2];
		alpha[2] = alpha[2]/2.0;
		i++;
	}
	return 0;
}

double BFGS::phiGradient(
		BFGS_function_struct function,
		vector &xk,
		vector &pk,
		double alpha,
		vector &tempArray,
		double phi_alpha,
		double eps)
{
	tempArray = pk;
	tempArray *= (alpha + eps);
	tempArray += xk;
	double result = (function.f)(&tempArray, function.params);

	return (result - phi_alpha)/eps;
}

double BFGS::zoom(
		double &alphalow,
		double &alphahigh,
		double &phi_alphalow,
		double &dphi_alphalow,
		double &phi_alphahigh,
		BFGS_function_struct function,
		vector &xk,
		vector &pk,
		vector &tempArray,
		double phi_alpha_zero,
		double dphi_alpha_zero,
		double c1,
		double c2,
		double eps)
{
	int ite=0;
	double alphaj=alphalow;
	double phi_alphaj=0.0;
	double dphi_alphaj=0.0;
	while((ite<100) && (std::abs(alphahigh-alphalow)>1e-6))
	{
		alphaj = quadraticInterpolation(phi_alphalow, dphi_alphalow, alphalow, phi_alphahigh, alphahigh);

		tempArray = pk;
		tempArray *= alphaj;
		tempArray += xk;
		phi_alphaj = (function.f)(&tempArray, function.params);

		if( (phi_alphaj > phi_alpha_zero + c1*alphaj*dphi_alpha_zero) || (phi_alphaj >= phi_alphalow) )
		{
			alphahigh = alphaj;
			phi_alphahigh = phi_alphaj;
		}
		else
		{
			dphi_alphaj = phiGradient(function, xk, pk, alphaj, tempArray, phi_alphaj, eps);
			if(std::abs(dphi_alphaj) <= -c2*dphi_alpha_zero)
				return alphaj;
			if(dphi_alphaj*(alphahigh-alphalow) >= 0)
			{
				alphahigh = alphalow;
				phi_alphahigh = phi_alphalow;
			}
			alphalow = alphaj;
			phi_alphalow = phi_alphaj;
			dphi_alphalow = dphi_alphaj;
		}
		++ite;
	}
	return alphaj;
}

double BFGS::quadraticInterpolation(
		double phi_alpha_lo,
		double dphi_alpha_lo,
		double alpha_lo,
		double phi_alpha_hi,
		double alpha_hi)
{
	//minima of quadratic function http://en.wikipedia.org/wiki/Quadratic_function
	//solution of 3 equations with 3 unknowns (a,b,c). Only a and b is needed to find minima
	double temp = (alpha_hi-alpha_lo);
	double a = (phi_alpha_hi - phi_alpha_lo - dphi_alpha_lo * alpha_hi + dphi_alpha_lo * alpha_lo)/(temp*temp);
	double b = dphi_alpha_lo - 2.0 * a * alpha_lo;
	return -b/(2.0*a);
}
