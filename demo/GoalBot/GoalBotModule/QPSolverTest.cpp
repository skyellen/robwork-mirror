#include "QPSolver.hpp"
#include <iostream>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

using namespace boost::numeric::ublas;
int main() {
    {
	matrix<double> H = zero_matrix<double>(2);
	vector<double> q(2);
	H(0,0) = 2;
	H(1,1) = 2;
	
	q(0) = -2;
	q(1) = -5;
	
	matrix<double> A(5,2);
	A(0,0) = 1;
	A(0,1) = -2;
    
	A(1,0) = -1;
	A(1,1) = -2;
	
	A(2,0) = -1;
	A(2,1) = 2;
	
	A(3,0) = 1;
	A(3,1) = 0;
	
	A(4,0) = 0;
	A(4,1) = 1;
    
	vector<double> b(5);
	b[0] = -2;
	b[1] = -6;
	b[2] = -2;
	b[3] = 0;
	b[4] = 0;
	
	vector<double> res = QPSolver::InequalitySolve(H, q, A, b);
	std::cout<<"Result 1 = ";
	for (size_t i = 0; i<res.size(); i++)
	    std::cout<<res(i)<<" "<<std::endl;
	
	char ch[2]; std::cin.getline(ch, 2);
    }
    
    {
	matrix<double> H(2,2);

	H(0,0) = 2;
	H(0,1) = 1;
	H(1,0) = 1;
	H(1,1) = 2;

	vector<double> q(2);
	q[0] = -6;
	q[1] = -6;
	
	matrix<double> A(4,2);
	A(0,0) = 1;
	A(0,1) = 0;
	
	A(1,0) = 0;
	A(1,1) = 1;
	
	A(2,0) = -1;
	A(2,1) = 0;
	
	A(3,0) = 0;
	A(3,1) = -1;
	
	//	A(4,0) = 1;
	//A(4,1) = 1;
	
	vector<double> b(4);
	b[0] = b[1] = 0;
	b[2] = -5;
	b[3] = -4;
	//	b[4] = 0;
	vector<double> res = QPSolver::InequalitySolve(H, q, A, b);
	std::cout<<"Result 2 = ";
	for (size_t i = 0; i<res.size(); i++)
	    std::cout<<res(i)<<" "<<std::endl;

	char ch[2]; std::cin.getline(ch, 2);
    }


}
