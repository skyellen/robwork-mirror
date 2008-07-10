
#include <rw/common/StringUtil.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/foreach.hpp>
#include <iostream>

using namespace rw::common;

void StringUtilTest()
{
    BOOST_MESSAGE("StringUtilTest");
    BOOST_CHECK(
        StringUtil::getFileExtension("") == "");


    std::cout<<"A"<<std::endl;
    BOOST_CHECK(
        StringUtil::getFileExtension("whatever") == "");


    std::cout<<"A"<<std::endl;
    BOOST_CHECK(
        StringUtil::getFileExtension("whatever.") == ".");

    std::cout<<"A"<<std::endl;
    BOOST_CHECK(
        StringUtil::getFileExtension("whatever.x") == ".x");

    std::cout<<"A"<<std::endl;
    BOOST_CHECK(
        StringUtil::getFileExtension("foo.bar/baz") == "");

    std::cout<<"A"<<std::endl;
    BOOST_CHECK(
        StringUtil::getFileExtension("foo.bar\\baz") == "");

    std::cout<<"A"<<std::endl;
    BOOST_CHECK(StringUtil::getFileExtension("foo.bar/baz.txt") == ".txt");

    std::cout<<"A"<<std::endl;
    BOOST_CHECK(StringUtil::getFileExtension("foo.bar\\baz.txt") == ".txt");

    std::cout<<"Finished"<<std::endl;
}
