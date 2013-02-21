/*
    Copyright [yyyy] [name of copyright owner]

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

*/

#ifndef FDUTIL_HPP
#define FDUTIL_HPP

// #include <vector>
#include <boost/numeric/ublas/vector.hpp>

class FdUtil
{
    public:
	static void vectorDerivative(const boost::numeric::ublas::vector<double> &f, boost::numeric::ublas::vector<double> &df, const double h);
    
    
    
    private:
	/*
	typedef  double (FdUtil::*FdUtilMemFn)(double xi); 
	
	// caller macro, see http://www.parashift.com/c++-faq-lite/pointers-to-members.html
	#define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(ptrToMember)) 
	
	bool useCentered(const double xi, const double h, const int w) ;
	bool useForward(const double xi, const double h, const int w) ;
	bool useBackward(const double xi, const double h, const int w) ;
	
	double dFx(const double xi, FdUtilMemFn f) ;
	double ddFx(const double xi, FdUtilMemFn f) ;
	*/
};

#endif // FDUTIL_HPP
