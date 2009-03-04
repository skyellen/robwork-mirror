/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "Q.hpp"

#include <rw/common/macros.hpp>

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

std::istream& operator>>(std::istream& in, rw::math::Q& q) {
    char ch1, ch2;
    in.get(ch1);
    in.get(ch2);
    if (ch1 != 'Q' || ch2 != '[')
        RW_THROW("Content of input stream does not match format of Q");
    int size = 0;
    in >> size;

    in.get(ch1);
    in.get(ch2);
    if (ch1 != ']' || ch2 != '{')
        RW_THROW("Content of input stream does not match format of Q");

    q = rw::math::Q(size);
    for (int i = 0; i<size; i++)
        in >> q(i);

    in.get(ch1);
    if (ch1 != '}')
        RW_THROW("Content of input stream does not match format of Q");

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
    return inner_prod(a.m(), b.m());
}
