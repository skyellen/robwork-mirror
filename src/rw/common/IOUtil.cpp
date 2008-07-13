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

#include "IOUtil.hpp"

#include "StringUtil.hpp"
#include "macros.hpp"
#include "Log.hpp"
#include "Message.hpp"

#include <fstream>
#include <iostream>
#include <cassert>
#include <algorithm>

#define NS IOUtil

using namespace rw::common;

void NS::readFile(
    const std::string& file_name,
    std::vector<char>& result)
{
    std::ifstream inp(file_name.c_str(), std::ios::binary);

    // Check input.
    if (!inp.is_open())
        RW_THROW("Can't read file " << StringUtil::quote(file_name));

    // The file length.
    inp.seekg(0, std::ios::end);
    const int len = inp.tellg();
    inp.seekg(0, std::ios::beg);

    // The buffer.
    result.resize(len);

    // Read the file.
    inp.read(&result.front(), len);

    // Check if we succeeded.
    if (inp.bad())
        RW_THROW("Reading of file " << StringUtil::quote(file_name) << " failed.");

    // Close the file.
    inp.close();
}

namespace
{
    std::string intersperse(
        const std::vector<std::string>& vals,
        const std::string& sep)
    {
        std::stringstream sum;
        typedef std::vector<std::string>::const_iterator I;

        I p = vals.begin();
        if (p != vals.end())
            sum << *p;

        for (++p; p != vals.end(); ++p) {
            sum << sep << *p;
        }

        return sum.str();
    }

    bool isReadable(const std::string& file)
    {
        std::ifstream in(file.c_str());
        return in.is_open();
    }

    std::string resolveSuffix(
        const std::string& raw_filename,
        const std::string& suffix)
    {
        const std::string& v1 = raw_filename + suffix;
        const std::string& v2 = raw_filename + StringUtil::toLower(suffix);
        const std::string& v3 = raw_filename + StringUtil::toUpper(suffix);

        return
            isReadable(v1) ? v1 :
            isReadable(v2) ? v2 :
            isReadable(v3) ? v3 :
            "";
    }

    std::string resolveFileNameHelper(
        const std::string& raw_filename,
        const std::vector<std::string>& extensions)
    {
        typedef std::vector<std::string>::const_iterator I;
        for (I p = extensions.begin(); p != extensions.end(); ++p) {
            const std::string& file = resolveSuffix(raw_filename, *p);
            if (!file.empty()) return file;
        }
        return "";
    }
}

std::string NS::resolveFileName(
    const std::string& raw_filename,
    const std::vector<std::string>& extensions)
{
    // First check the raw file name.
    std::ifstream in(raw_filename.c_str());
    if (in.is_open()) {
        return raw_filename;
    }

    // Then check with suffixes.
    const std::string& file =
        resolveFileNameHelper(raw_filename, extensions);

    if (!file.empty()) {
        return file;
    } else {
        // Throw a nice exception.
        RW_THROW(
            "Can't open file "
            << StringUtil::quote(raw_filename)
            << ". Optional extensions considered: "
            << intersperse(extensions, " "));

        // To avoid a compiler warning.
        return "";
    }
}

void NS::rwAssert(const char* expression, const char* file, int line)
{
    Message msg(file, line, expression);
    Log::get(Log::errorId()).write(msg);
    exit(-1);
}
