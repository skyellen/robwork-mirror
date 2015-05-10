/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#include "Q.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "Math.hpp"

using namespace rw::common;
using namespace rw::math;

Q::Q(size_t n, const double* values):
_vec(n)
{
    for (size_t i = 0; i<n; i++)
        _vec(i) = values[i];
}

void Q::init(size_t n, const double* values){
    for (size_t i = 0; i<n; i++)
        _vec(i) = values[i];
}

Q::Q(size_t n, double value):
    _vec(n)
{
    for (size_t i = 0; i<n; i++)
        _vec(i) = value;
}

Q::~Q(){

}

std::ostream& rw::math::operator<<(std::ostream& out, const Q& v)
{
    if (v.size() == 0)
        return out << "Q[0]{}";
    else {
        out << "Q[" << (int)v.size() << "]{";
        for (size_t i = 0; i < v.size() - 1; i++)
            out << v[i] << ", ";
        return out << v[v.size() - 1] << "}";
    }
}


std::istream& rw::math::operator>>(std::istream& in, Q& q) {
    char ch1, ch2;
	do {
		in.get(ch1);
	} while (ch1 == ' ' || ch1 == '\t'); //Ignore space and tab, but not line changes.


	int size = -1;

	if (ch1 == 'Q') {
		in.get(ch2);	
		if (ch1 != 'Q' || ch2 != '[')
			RW_THROW("Content of input stream does not match format of Q");
		in >> size;

		in.get(ch1);
		in.get(ch2);
		if (ch1 != ']' || ch2 != '{')
			RW_THROW("Content of input stream does not match format of Q");
	} else if (ch1 != '{') {
		RW_THROW("Content of input stream does not match format of Q");
	}
	
	std::vector<double> res;
	while (ch1 != '}') {
		double d;
		in >> d;
		if (!in.eof()) {
			res.push_back(d);
		}
		in.get(ch1);
	}

    if (ch1 != '}')
        RW_THROW("Content of input stream does not match format of Q");

	if (size > -1 && (int)res.size() != size) {
		RW_THROW("Length of Q does not match device");
	}

	q = Q(res.size(), &res[0]);
    return in;
}

bool rw::math::operator==(const Q& q1, const Q& q2)
{
    if (q1.size() != q2.size())
        return false;

    for (size_t i = 0; i < q1.size(); i++)
        if (q1(i) != q2(i))
            return false;
    return true;
}

double rw::math::dot(const Q& a, const Q& b)
{
	return a.e().dot(b.e());
    //return inner_prod(a.m(), b.m());
}

rw::math::Q rw::math::concat(const Q& q1, const Q& q2){
    Q q(q1.size()+q2.size());
    for(size_t i=0;i<q1.size();i++)
        q(i) = q1(i);
    for(size_t i=0;i<q2.size();i++)
        q(q1.size()+i) = q2(i);
    return q;
}


Q::Q(size_t n, double a0, double a1):_vec(n){
    if(n<2) RW_THROW("Vector size must be >= 2");
     _vec[0] = a0; _vec[1] = a1;
}
Q::Q(size_t n, double a0, double a1, double a2):_vec(n){
    if(n<3) RW_THROW("Vector size must be >= 3");
    _vec[0] = a0; _vec[1] = a1; _vec[2] = a2;
}
Q::Q(size_t n, double a0, double a1, double a2, double a3):_vec(n){
    if(n<4) RW_THROW("Vector size must be >= 4");
    _vec[0] = a0; _vec[1] = a1; _vec[2] = a2; _vec[3] = a3;
}
Q::Q(size_t n, double a0, double a1, double a2, double a3, double a4):_vec(n){
    if(n<5) RW_THROW("Vector size must be >= 5");
    _vec[0] = a0; _vec[1] = a1; _vec[2] = a2; _vec[3] = a3; _vec[4] = a4;
}
Q::Q(size_t n, double a0, double a1, double a2, double a3, double a4, double a5):_vec(n){
    if(n<6) RW_THROW("Vector size must be >= 6");
    _vec[0] = a0; _vec[1] = a1; _vec[2] = a2; _vec[3] = a3; _vec[4] = a4; _vec[5] = a5;
}
Q::Q(size_t n, double a0, double a1, double a2, double a3, double a4, double a5, double a6):_vec(n){
    if(n<7) RW_THROW("Vector size must be >= 7");
    _vec[0] = a0; _vec[1] = a1; _vec[2] = a2; _vec[3] = a3; _vec[4] = a4; _vec[5] = a5; _vec[6] = a6;
}
Q::Q(size_t n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7):_vec(n){
    if(n<8) RW_THROW("Vector size must be >= 8");
    _vec[0] = a0; _vec[1] = a1; _vec[2] = a2; _vec[3] = a3; _vec[4] = a4; _vec[5] = a5; _vec[6] = a6; _vec[7] = a7;
}
Q::Q(size_t n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8):_vec(n){
    if(n<9) RW_THROW("Vector size must be >= 9");
    _vec[0] = a0; _vec[1] = a1; _vec[2] = a2; _vec[3] = a3; _vec[4] = a4; _vec[5] = a5; _vec[6] = a6; _vec[7] = a7; _vec[8] = a8;
}
Q::Q(size_t n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9):_vec(n){
    if(n<10) RW_THROW("Vector size must be >= 10");
    _vec[0] = a0; _vec[1] = a1; _vec[2] = a2; _vec[3] = a3; _vec[4] = a4; _vec[5] = a5; _vec[6] = a6; _vec[7] = a7; _vec[8] = a8; _vec[9] = a9;
}

template<>
void rw::common::serialization::write(const Q& tmp, OutputArchive& oar, const std::string& id)
{
    oar.write( Math::toStdVector(tmp, (int)tmp.size()), id , "Q");
}
template<>
void rw::common::serialization::read(Q& tmp, InputArchive& iar, const std::string& id){
    std::vector<double> arr;
    iar.read(arr, id, "Q");
    tmp = Q(arr);
    //rw::math::Math::fromStdVector(arr, tmp);
}

