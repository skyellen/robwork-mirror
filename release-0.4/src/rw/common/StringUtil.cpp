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


#include "StringUtil.hpp"

#include "macros.hpp"

#include <fstream>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <cstring>
#include <sstream>
#include <boost/foreach.hpp>

using namespace rw::common;

bool StringUtil::isAbsoluteFileName(const std::string& file)
{
    if (file.empty()) return false;

    // This is a really lame implementation of this check. Well, hopefully
    // it works OK in practice.
    return
        file[0] == '/' ||
        file[0] == '\\' ||
        (file.size() > 3 && file[1] == ':');
}

std::string StringUtil::replaceBackslash(const std::string& str)
{
    std::string newStr = str;
    for (int i = 0; i < (int)str.length(); i++) {
        if (str.at(i) == '\\')
            newStr.at(i) = '/';
    }
    return newStr;
}

std::string StringUtil::getDirectoryName(const std::string& path)
{
    const std::string::size_type pos = path.find_last_of("/\\");

    if (pos != std::string::npos)
        return path.substr(0, pos + 1);
    else
        return std::string();
}

std::string StringUtil::getFileExtension(
    const std::string& path)
{
    const std::string::size_type pos = path.find_last_of("./\\");

    if (pos == std::string::npos)
        return "";
    else {
        const char x = path[pos];

        // If we have a filename like foo.bar/values we don't have an extension.
        if (x == '/' || x == '\\')
            return "";
        else
            return path.substr(pos, path.size());
    }
}

std::string StringUtil::toUpper(const std::string& str)
{
    std::string result(str);
	for(size_t i=0; i < str.length(); i++)
		result[i] = toupper(str[i]);

    return result;
}

std::string StringUtil::toLower(const std::string& str)
{
    std::string result(str);
	for(size_t i=0; i < str.length(); i++)
		result[i] = tolower(str[i]);

    return result;
}

std::string StringUtil::quote(const std::string& str)
{
    const std::string q = "'";
    return q + str + q;
}

std::vector<std::string> StringUtil::words(const std::string& str)
{
    typedef std::string::const_iterator I;
    const I end = str.end();

    std::vector<std::string> result;

    I wordBegin = str.begin();
    while (true) {
        while (wordBegin != end && isspace(*wordBegin)) ++wordBegin;

        I wordEnd = wordBegin;
        while (wordEnd != end && !isspace(*wordEnd)) ++wordEnd;

        if (wordBegin == end) break;
        else {
            result.push_back(std::string(wordBegin, wordEnd));
            wordBegin = wordEnd;
        }
    }

    return result;
}

namespace
{
    template <class X>
    inline
    std::pair<bool, X> toX(const std::string& str)
    {
        using namespace std;
        pair<bool, X> nothing(false, X());

        istringstream buf(str);
        X x;
        buf >> x;
        if (!buf) return nothing;

        string rest;
        buf >> rest;
        if (buf) return nothing;
        else return make_pair(true, x);
    }

    template <class X>
    inline
    std::pair<bool, std::vector<X> > toXs(
        const std::vector<std::string>& words)
    {
        typedef std::vector<X> V;
        std::pair<bool, V> nothing(false, V());
        V vals;
        BOOST_FOREACH(const std::string& str, words) {
            std::pair<bool, X> okX = toX<X>(str);
            if (okX.first) vals.push_back(okX.second);
            else return nothing;
        }
        return make_pair(true, vals);
    }
}

std::pair<bool, double> StringUtil::toDouble(const std::string& str)
{
    return toX<double>(str);
}

std::pair<bool, int> StringUtil::toInt(const std::string& str)
{
    return toX<int>(str);
}

std::pair<bool, std::vector<double> > StringUtil::toDoubles(
    const std::vector<std::string>& words)
{
    return toXs<double>(words);
}

std::pair<bool, std::vector<int> > StringUtil::toInts(
    const std::vector<std::string>& words)
{
    return toXs<int>(words);
}
