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

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

#include <fstream>
#include <iostream>
#include <boost/version.hpp>
#if(BOOST_VERSION<104100)
#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_position_iterator.hpp>
#include <boost/spirit/include/phoenix1.hpp>
#include <boost/spirit/include/classic_symbols.hpp>

#include <boost/spirit/include/classic_common.hpp>
#include <boost/spirit/include/classic_ast.hpp>
#include <boost/spirit/include/classic_parse_tree.hpp>
#include <boost/spirit/iterator/file_iterator.hpp>

#else
//#include <boost/spirit/include/support_istream_iterator.hpp>
#include <boost/spirit/include/qi.hpp>
#endif

#include <boost/bind.hpp>
//#include <boost/lambda/lambda.hpp>

//#include <cassert>
//#include <algorithm>

#if !(defined __MINGW32__) && !(defined _WIN32)
#include <unistd.h>	// for getcwd on linux
#endif
#ifdef _WIN32
#include <direct.h> // for getcwd
#endif

using namespace rw::common;

void IOUtil::readFile(
    const std::string& file_name,
    std::vector<char>& result)
{
    std::ifstream inp(file_name.c_str(), std::ios::binary);

    // Check input.
    if (!inp.is_open())
        RW_THROW("Can't read file " << StringUtil::quote(file_name));

    // The file length.
    inp.seekg(0, std::ios::end);
    const int len = (int)inp.tellg();
	
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

std::string IOUtil::resolveFileName(const std::string& raw_filename,
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



void IOUtil::rwAssert(const char* expression, const char* file, int line)
{
    Message msg(file, line, expression);
    Log::errorLog().write(msg);
    Log::errorLog() << std::endl;
    //exit(-1);
    abort();
}

std::string IOUtil::getAbsoluteFileName(const std::string& file){
	if( StringUtil::isAbsoluteFileName(file) ){
		return file;
	}
	// prepend the working directory
	char buffer[500];

	char *res = NULL;
	//Previously this was called without the _. This however should be more compliant with ISO C++ (according to Visual Studio)
#ifdef _MSC_VER
	res = _getcwd(buffer, 500);
#else
	res = getcwd(buffer, 500);
#endif
	if(res==NULL)
	    RW_THROW("Absolute filename could not be derived, for file: " << file);
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


void IOUtil::getFilesInFolder(const std::string& path, const std::string& fileMask, bool recursive, bool addPath, std::vector<std::string>& result) {
    try
    {
        //Depending on how string has been generated boost::replace_all sometimes fails. 
        //A fix of this is to convert it with c_str().
        std::string regStr = fileMask.c_str();

	    boost::replace_all(regStr, "\\", "\\\\");
	    boost::replace_all(regStr, ".", "\\.");
	    boost::replace_all(regStr, "*", ".*");
	    boost::replace_all(regStr, "(", "\\(");
	    boost::replace_all(regStr, ")", "\\)");
	    boost::replace_all(regStr, "+", "\\+");

        const boost::regex regex(regStr);
        boost::cmatch match;

        boost::filesystem::directory_iterator end;
        for (boost::filesystem::directory_iterator it(path); it != end; it++)
        {			        
            if (boost::filesystem::is_directory(it->status())) {
                if(recursive)
                    getFilesInFolder(it->path().string(), fileMask, recursive, addPath, result);
                continue;
            }

	        if (!boost::filesystem::is_regular_file(it->status())) //If not a regular file
                continue;
#if(BOOST_FILESYSTEM_VERSION==2)
	        std::string filename = it->path().filename();
#else
	        std::string filename = it->path().filename().string();
#endif
            if (!boost::regex_match(filename.c_str(), match, regex)) 
                continue;

	        if (addPath)
		        result.push_back(it->path().string());
	        else{
#if(BOOST_FILESYSTEM_VERSION==2)
                result.push_back(it->path().filename());
#else
                result.push_back(it->path().filename().string());
#endif
	        }
        }
    }
    catch (const std::exception& e)
    {
        RW_THROW("Unable to retrieve files in folder: "<<e.what());
    }
}

std::vector<std::string> IOUtil::getFilesInFolder(const std::string& path, bool recursive, bool addPath, const std::string& fileMask) {
    std::vector<std::string> result;
    getFilesInFolder(path, fileMask, recursive, addPath, result);
    /*BOOST_FOREACH(std::string p, result){
        std::cout << p << std::endl;
    }*/
    return result;
}


size_t IOUtil::getFileSize(const std::string& filename){
    namespace fs = boost::filesystem;
    fs::path p( filename );
    return (size_t)fs::file_size( p );
}

std::time_t IOUtil::getLastFileWrite(const std::string& filename){
    namespace fs = boost::filesystem;
    fs::path p( filename );
    std::time_t ft = fs::last_write_time( p );
    return ft;
}

std::string IOUtil::getFirstXMLElement(const std::string& filename){
    using namespace boost;

#if(BOOST_VERSION<104100)
    using namespace boost::spirit;
    using namespace boost::spirit::classic;
    using namespace phoenix;

    file_iterator<> first(filename.c_str());

    if (!first)
    {
       // Clean up, throw an exception, whatever
       RW_THROW("Unable to open file!\n");
    }

    file_iterator<> last = first.make_end();

    std::string result;
    // use iterator to parse file data
    parse_info<file_iterator<char> > info = parse(first, last,
                                        !("<?" >> *(anychar_p - '>')//[std::cout << boost::lambda::_1]
                                        >> '>')
                                        >> "<"
                                        //>> *( (anychar_p - '>') | (anychar_p -' '))[boost::bind(static_cast<std::string& (std::string::*)( size_t, char )>(&std::string::append),&result, 1, ::_1)]
                                        >> *( (anychar_p - '>') | (anychar_p -' '))[var( result ) = construct_<std::string>(arg1,arg2)]
                                        >> *(anychar_p - '>')//[std::cout << boost::lambda::_1]
                                        >> ">"
                       // THE skip parser comes next
                      , (blank_p - ' ')
                      | (space_p - ' ')
                      | ("<!--" >> *(anychar_p - '>') >> '>')

        );
    if ( !info.hit )
        RW_THROW("file \""<<filename<<"\" is not a wellformed xml document!");
    return result;


#else
    using namespace boost::spirit::qi;
    // check the first element of the file
    std::ifstream input(filename.c_str());
    input.unsetf(std::ios::skipws);

    spirit::istream_iterator begin(input);
    spirit::istream_iterator end;
    std::string result;
    // use iterator to parse file data
    bool res = spirit::qi::phrase_parse(begin, end,
                                        -("<?" >> *(char_ - '>')/*[std::cout << boost::lambda::_1]*/ >> char_('>')) >>
                                        "<" >>
                                        *(char_ - (char_('>')))[boost::bind(static_cast<std::string& (std::string::*)( size_t, char )>(&std::string::append),&result, 1, ::_1)] >>
                                        *(char_ - char_('>'))/*[std::cout << boost::lambda::_1]*/ >> ">"
                       // THE skip parser comes next
                      , spirit::qi::blank
                      | spirit::qi::space
                      | ("<!--" >> *(char_ - '>') >> '>')

        );
    if(!res)
        RW_THROW("file \""<<filename<<"\" is not a wellformed xml document! " << result);
    return result;
#endif
}

std::string IOUtil::getFirstXMLElement(std::istream & inputStream){
    using namespace boost;

#if(BOOST_VERSION<104100)
#warning Your boost version is not new enough
    RW_THROW("Your boost version is not new enough!");
#else
    using namespace boost::spirit::qi;
    inputStream.unsetf(std::ios::skipws);

    spirit::istream_iterator begin(inputStream);
    spirit::istream_iterator end;
    std::string result;
    // use iterator to parse file data
    bool res = spirit::qi::phrase_parse(begin, end,
                                        -("<?" >> *(char_ - '>')/*[std::cout << boost::lambda::_1]*/ >> char_('>')) >>
                                        "<" >>
                                        *(char_ - (char_('>')))[boost::bind(static_cast<std::string& (std::string::*)( size_t, char )>(&std::string::append),&result, 1, ::_1)] >>
                                        *(char_ - char_('>'))/*[std::cout << boost::lambda::_1]*/ >> ">"
                       // THE skip parser comes next
                      , spirit::qi::blank
                      | spirit::qi::space
                      | ("<!--" >> *(char_ - '>') >> '>')

        );
    if(!res)
        RW_THROW("The provided ifstream does not consist of a wellformed xml document! " << result);
    return result;
#endif
}
