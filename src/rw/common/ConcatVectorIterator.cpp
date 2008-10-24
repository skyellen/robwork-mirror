/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#include "ConcatVectorIterator.hpp"

using namespace rw::common;

namespace
{
    int f()
    {
        std::vector<char*> curr;
        std::vector<char*> next;

        ConcatVectorIterator<char> begin(&curr, curr.begin(), &next);

        ConstConcatVectorIterator<char> const_begin(begin);

        ConstConcatVectorIterator<char> f = begin;
        ++f;

        const_begin++;

        char& x = *begin;

        const char& c = *const_begin;
        if (c == x) return -1;

        return 0;
    }
}
