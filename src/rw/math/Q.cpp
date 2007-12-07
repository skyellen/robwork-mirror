/*********************************************************************
 * RobWork Version 0.2
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

#define NS rw::math
using namespace NS;

std::ostream& NS::operator<<(std::ostream& out, const Q& v)
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
