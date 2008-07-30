
#include <rw/common/StringUtil.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/foreach.hpp>
#include <iostream>

using namespace rw::common;

void StringUtilTest()
{
    BOOST_MESSAGE("- StringUtilTest");
    BOOST_CHECK(
        StringUtil::getFileExtension("") == "");

    BOOST_CHECK(
        StringUtil::getFileExtension("whatever") == "");


    BOOST_CHECK(
        StringUtil::getFileExtension("whatever.") == ".");

    BOOST_CHECK(
        StringUtil::getFileExtension("whatever.x") == ".x");

    BOOST_CHECK(
        StringUtil::getFileExtension("foo.bar/baz") == "");

    BOOST_CHECK(
        StringUtil::getFileExtension("foo.bar\\baz") == "");

    BOOST_CHECK(StringUtil::getFileExtension("foo.bar/baz.txt") == ".txt");

    BOOST_CHECK(StringUtil::getFileExtension("foo.bar\\baz.txt") == ".txt");

}
