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

#include "../TestSuiteConfig.hpp"

#include <rw/common/StringUtil.hpp>

using namespace rw::common;

BOOST_AUTO_TEST_CASE( StringUtilGetFileExtensionTest )
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
        Pair("foo.bar\\baz.txt", ".txt")
    };
    const int len = sizeof(pairs) / sizeof(*pairs);
    for (int i = 0; i < len; i++) {
        const Pair& pair = pairs[i];
        BOOST_CHECK(StringUtil::getFileExtension(pair.first) == pair.second);
    }
	
}

BOOST_AUTO_TEST_CASE( StringUtilGetFileNameTest )
{
	BOOST_CHECK(StringUtil::getFileName("d:\\test\\folder1\\filename.txt") == "filename.txt");
	BOOST_CHECK(StringUtil::getFileName("/test/folder1/filename.txt") == "filename.txt");
	BOOST_CHECK(StringUtil::getFileName("d:\\test\\folder1\\filename") == "filename");
}
