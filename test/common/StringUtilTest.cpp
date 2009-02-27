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


#include <rw/common/StringUtil.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/foreach.hpp>
#include <iostream>

using namespace rw::common;

BOOST_AUTO_TEST_CASE( StringUtilTest )
{
    typedef std::pair<std::string, std::string> Pair;
    const Pair pairs[] = {
        Pair("", ""),
        Pair("/", ""),
        Pair("\\", ""),
        Pair("whatever/", ""),
        Pair("whatever\\", ""),
        Pair("whatever", ""),
        Pair("whatever.", "."),
        Pair("whatever.x", ".x"),
        Pair("foo.bar/baz", ""),
        Pair("foo.bar\\baz", ""),
        Pair("foo.bar/baz.txt", ".txt"),
        Pair("foo.bar\\baz.txt", ".txt"),
    };
    const int len = sizeof(pairs) / sizeof(*pairs);
    for (int i = 0; i < len; i++) {
        const Pair& pair = pairs[i];
        BOOST_CHECK(StringUtil::getFileExtension(pair.first) == pair.second);
    }
}
