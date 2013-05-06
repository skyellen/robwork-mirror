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

using namespace rwlibs::softbody;

void FdUtil::vectorDerivative(const boost::numeric::ublas::vector< double >& f, boost::numeric::ublas::vector< double >& df, const double h)
{
    assert(f.size() > 1);
    
    for (int i = 0; i < (int) f.size(); i++) {
        if (0 == i) // forward difference
            df[i] = ( f[i+1] - f[i] ) / h;
        else if (i == (f.size() -1))   { // bwd difference
            df[i] = ( f[i] - f[i-1] ) / h;
        }
        else {
            df[i] = ( -f[i-1] + f[i+1] ) / (2 * h);
        }
    }
}
