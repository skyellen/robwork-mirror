
#include <rw/common/StringUtil.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/foreach.hpp>
#include <iostream>

using namespace rw::common;

void StringUtilTest()
{
    BOOST_MESSAGE("- StringUtilTest");

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
