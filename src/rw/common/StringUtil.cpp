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

#include "StringUtil.hpp"

#include "macros.hpp"

#include <fstream>
#include <iostream>
#include <cassert>
#include <algorithm>

using namespace rw::common;


bool StringUtil::IsAbsoluteFileName(const std::string& file)
{
    if (file.empty()) return false;

    // This is a really lame implementation of this check. Well, hopefully
    // it works OK in practice.
    return
        file[0] == '/' ||
        file[0] == '\\' ||
        (file.size() > 3 && file[1] == ':');
}


std::string StringUtil::ReplaceBackslash(const std::string& str)
{
    std::string newStr = str;
    for (int i = 0; i < (int)str.length(); i++) {
        if (str.at(i) == '\\')
            newStr.at(i) = '/';
    }
    return newStr;
}



std::string StringUtil::GetDirectoryName(const std::string& path)
{
    const std::string::size_type pos = path.find_last_of("/\\");

    if (pos != std::string::npos)
        return path.substr(0, pos + 1);
    else
        return std::string();
}

std::string StringUtil::GetFileExtension(
    const std::string& path)
{
    const std::string::size_type pos = path.find_last_of(".");

    if (pos != std::string::npos)
        return path.substr(pos, path.size());
    else
        return std::string();
}

std::string StringUtil::ToUpper(const std::string& str)
{
    std::string result(str);
	for(size_t i=0; i < str.length(); i++)
		result[i] = toupper(str[i]);

    return result;
}

std::string StringUtil::ToLower(const std::string& str)
{
    std::string result(str);
	for(size_t i=0; i < str.length(); i++)
		result[i] = tolower(str[i]);

    return result;
}

std::string StringUtil::Quote(const std::string& str)
{
    const std::string q = "'";
    return q + str + q;
}
