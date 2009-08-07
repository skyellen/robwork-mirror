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


#include "IOUtil.hpp"

#include "StringUtil.hpp"
#include "macros.hpp"
#include "Log.hpp"
#include "Message.hpp"

#include <fstream>
#include <iostream>
#include <cassert>
#include <algorithm>

#if !(defined __MINGW32__) && !(defined _WIN32)
#include <unistd.h>	// for getcwd on linux
#endif
#ifdef _WIN32
#include <direct.h> // for getcwd
#endif



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
    Log::errorLog().write(msg);
    exit(-1);
}

std::string IOUtil::getAbsoluteFileName(const std::string& file){
	if( StringUtil::isAbsoluteFileName(file) ){
		return file;
	}
	// prepend the working directory
	char buffer[500];
	getcwd(buffer, 500);
	std::string workDir(buffer);
	return workDir+"/"+file;
}

bool IOUtil::isLittleEndian(){
    union
    {
        int testWord;
        char testByte[4];
    } endianTest;

    endianTest.testWord = 1;
    if (endianTest.testByte[0] == 1) {
        // litle endian - least significant byte first
        return true;
    }
    return false;
}

