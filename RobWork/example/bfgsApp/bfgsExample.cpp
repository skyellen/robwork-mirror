#include <rwlibs/algorithms/BFGS.hpp>

using namespace rwlibs::algorithms;

//The rosenbrock function
static double rosenbrock(const boost::numeric::ublas::vector<double> * x, void * params)
{
	double temp = 1-(*x)(0);
	double temp2 = (*x)(1)-(*x)(0)*(*x)(0);
	return temp*temp+100*temp2*temp2;
}

//Differential equation of the rosenbrock function
static void drosenbrock(const boost::numeric::ublas::vector<double> * x, void * params, boost::numeric::ublas::vector<double> * g)
{
	(*g)(0) = -2*(1-(*x)(0))-400*(*x)(0)*(-(*x)(0)*(*x)(0)+(*x)(1));
	(*g)(1) = 200*(-(*x)(0)*(*x)(0)+(*x)(1));
}

int main(int argc, char** argv) {
	// Initialise the minimization functions
    BFGS::BFGS_function_struct rosenbrockfunc;
    rosenbrockfunc.f = &rosenbrock;
    rosenbrockfunc.df = &drosenbrock;
    rosenbrockfunc.params = NULL; //Pointer to pass external variables into the minimization problem.

    //Store the start guess
	BFGS::vector* startguess = new BFGS::vector(2);
    (*startguess)(0)=5;
    (*startguess)(1)=5;

	//Calculate the start error for the minimization problem
	BFGS::vector* gradient = new BFGS::vector(2);
    double fx=rosenbrock(startguess,rosenbrockfunc.params);
    drosenbrock(startguess,rosenbrockfunc.params,gradient);
    std::cout<<"f(x): "<<fx<<" df(x): "<<(*gradient)(0)<<std::endl;

	//Do optimization
    int result = BFGS::optimizer(*startguess,rosenbrockfunc,1e-4,1000,0.01,1e-4,0.9,1);
	if(result==BFGS::SUCCESS) {
		 std::cout<<"Optimization return with succes" << std::endl;
	}
	else {
		std::cout<<"Optimization return with warnings" << std::endl;
	}
	
	//Print result
	fx=rosenbrock(startguess,rosenbrockfunc.params);
    drosenbrock(startguess,rosenbrockfunc.params,gradient);
    std::cout<<"Optimization result i (" << (*startguess)(0) <<", " << (*startguess)(0) << ")" << std::endl;
	std::cout <<"f(x): "<<fx<<" df(x): "<<(*gradient)(0)<<std::endl;

}
