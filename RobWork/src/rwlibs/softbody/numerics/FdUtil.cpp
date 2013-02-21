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

#include "FdUtil.hpp"

void FdUtil::vectorDerivative(const boost::numeric::ublas::vector< double >& f, boost::numeric::ublas::vector< double >& df, const double h)
{
    assert(f.size() > 1);
    
    for (int i = 0; i < f.size(); i++) {
// 	std::cout << "f.size(): " << f.size() << std::endl;
// 	std::cout << "df.size(): " << df.size() << std::endl;
// 	std::cout << "i: " << i << std::endl;
// 	
	if (0 == i) // forward difference
	    df[i] = ( f[i+1] - f[i] ) / h;
	else if (i == (f.size() -1))   { // bwd difference
	    df[i] = ( f[i] - f[i-1] ) / h;
	}
	else
	    df[i] = ( -f[i-1] + f[i+1] ) / (2 * h);
    }
}

/*
double FdUtil::dFx(const double xi, FdUtil::FdUtilMemFn f) {	    
    const double h = 1.0e-3;
    
    bool buseCentered = useCentered(xi, h, 1);
    bool buseForward = useForward(xi, h, 1);
    bool buseBackward = useBackward(xi, h, 1);
    
    if (buseCentered)
	return ( -CALL_MEMBER_FN(*this, f)(xi-h) + CALL_MEMBER_FN(*this, f)(xi+h) ) / (2 * h);
    if (buseForward)
	return ( CALL_MEMBER_FN(*this, f)(xi+h) - CALL_MEMBER_FN(*this, f)(xi) ) / h;
    if (buseBackward)
	return ( CALL_MEMBER_FN(*this, f)(xi) - CALL_MEMBER_FN(*this, f)(xi-h) ) / h;
    
    eassert(false);
    return 42.0;
}



double FdUtil::ddFx(const double xi, FdUtil::FdUtilMemFn f) {	    
    const double h = 1.0e-3;
    
    bool buseCentered = useCentered(xi, h, 2);
    bool buseForward = useForward(xi, h, 2);
    bool buseBackward = useBackward(xi, h, 2);
    
    if (buseCentered)
	return (CALL_MEMBER_FN(*this, f)(xi-h) - 2 * CALL_MEMBER_FN(*this, f)(xi) + CALL_MEMBER_FN(*this, f)(xi+h)) / (h * h);
    if (buseForward)
	return (CALL_MEMBER_FN(*this, f)(xi) - 2*CALL_MEMBER_FN(*this, f)(xi+h) + CALL_MEMBER_FN(*this, f)(xi+2*h)) / (h * h);
    if (buseBackward)
	return ( CALL_MEMBER_FN(*this, f)(xi-2*h) - 2*CALL_MEMBER_FN(*this, f)(xi-h) + CALL_MEMBER_FN(*this, f)(xi) ) / (h*h);
    
    eassert(false);
    return 42.0;
}


bool FdUtil::useCentered(const double xi, const double h, const int w) {
    return ( (xi - h*w) > _a ) && ( (xi + h*w) < _b );
}
bool FdUtil::useForward(const double xi, const double h, const int w) {
    return ( (xi + h*w) < _b ) && ( xi >= _a );
}
bool FdUtil::useBackward(const double xi, const double h, const int w) {
    return ( (xi - h*w) > _a ) && ( xi <= _b );
}
*/