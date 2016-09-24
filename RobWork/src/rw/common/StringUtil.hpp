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


#ifndef RW_COMMON_STRINGUTIL_HPP
#define RW_COMMON_STRINGUTIL_HPP

/**
 * @file StringUtil.hpp
 */


#include <string>
#include <vector>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Collection of string manipulation utilities
     */
    class StringUtil
    {
    public:
        /**
         * @brief True if the file name \b file is an absolute file name.
         */
        static bool isAbsoluteFileName(const std::string& file);


        /**
         * @brief Replace '\' with '/' everywhere in the file name \b file.
         */
        static std::string replaceBackslash(const std::string& file);


        /**
         * @brief Removes space, tab and new line from string
         */
		static std::string removeWhiteSpace(const std::string& str);

        /**
         * @brief The directory part of a path name.
         *
         * The function simply returns everything up to and including the last slash
         * or backslash. If there is no such slash or backslash, the empty string is
         * returned.
         *
         * @param path [in] The path name.
         *
         * @return The directory name for the path.
         */
        static std::string getDirectoryName(const std::string& path);

        /**
         * @brief Extract the file name (including extension) from a string containing full name including the directory.
         *
         * The function simply returns everything efter the last slash
         * or backslash. If there is no such slash or backslash, the original string is returned
         *
         * @param pathAndFileName [in] The path and filename .
         *
         * @return The file name.
         */
		static std::string getFileName(const std::string& pathAndFileName);

        /**
         * @brief The extension of a file name.
         *
         * The extension is everything that follows the last '.'.
         *
         * The '.' is included in the extension returned, so that if that if the
         * file has no extension, then the empty string can be returned.
         *
         * @param filename [in] The file name.
         *
         * @return The file name extension.
         */
        static std::string getFileExtension(const std::string& filename);


        /**
         * @brief The uppercase of a string.
         *
         * @param str [in] A string to uppercase.
         *
         * @return \b str converted to upper case.
         */
        static std::string toUpper(const std::string& str);


        /**
         * @brief The lowercase of a string.
         *
         * @param str [in] A string to lowercase.
         *
         * @return \b str converted to lower case.
         */
        static std::string toLower(const std::string& str);


        /**
         * @brief The string \b str enclosed in single-quotes.
         *
         * Use this for quoting of strings in user error messages.
         */
        static std::string quote(const std::string& str);


        /**
           @brief Split \b str into words at space borders.
        */
        static std::vector<std::string> words(const std::string& str);

        /**
           @brief Return (true, val) if \b str parses as a double with value \b
           val and (false, 0) otherwise.
        */
        static std::pair<bool, double> toDouble(const std::string& str);

        /**
           @brief Return (true, val) if \b str parses as a int with value \b
           val and (false, 0) otherwise.
        */
        static std::pair<bool, int> toInt(const std::string& str);

        /**
		   @brief Return (true, val) if \b str parses as a int with value \b
		   val and (false, 0) otherwise.
		*/
		static std::pair<bool, unsigned int> toUInt(const std::string& str);

		/**
		   @brief Return (true, val) if \b str parses as a int with value \b
		   val and (false, 0) otherwise.
		*/
		static std::pair<bool, long> toLong(const std::string& str);

		/**
		   @brief Return (true, val) if \b str parses as a int with value \b
		   val and (false, 0) otherwise.
		*/
		static std::pair<bool, unsigned long> toULong(const std::string& str);

        /**
           @brief Return (true, vals) if \b words parses as a sequence of doubles with values \b
           vals and (false, []) otherwise.
        */
        static std::pair<bool, std::vector<double> > toDoubles(
            const std::vector<std::string>& words);

        /**
           @brief Return (true, vals) if \b words parses as a sequence of ints
           with values \b vals and (false, []) otherwise.
        */
        static std::pair<bool, std::vector<int> > toInts(const std::vector<std::string>& words);

        /**
           @brief Return (true, vals) if \b words parses as a sequence of ints
           with values \b vals and (false, []) otherwise.
        */
        static std::pair<bool, std::vector<unsigned int> > toUInts(const std::vector<std::string>& words);

        /**
		   @brief Return (true, vals) if \b words parses as a sequence of ints
		   with values \b vals and (false, []) otherwise.
		*/
		static std::pair<bool, std::vector<long> > toLongs(const std::vector<std::string>& words);

		/**
		   @brief Return (true, vals) if \b words parses as a sequence of ints
		   with values \b vals and (false, []) otherwise.
		*/
		static std::pair<bool, std::vector<unsigned long> > toULongs(const std::vector<std::string>& words);

		/**
		 * @brief Converts a string pattern with wild card characters * and ? into a regular expression
		 * 
		 * The format of the regular expression matches boost::regex.
		 *
		 * @param pattern [in] pattern with wild card characters
		 * @return patterns expressed as regular expression.
		 */
		static std::string patternToRegEx(const std::string& pattern);


		/**
		 * @brief creates a name based on a prefix and a random integer between
		 * 0xFF and 0xFFFFFF
		 * @param prefix
		 * @return
		 */
        static std::string ranName(const std::string& prefix);

	};

    /**
     * @brief Convenient definition of a pair of strings.
     * @relatesalso rw::common::StringUtil
     */
    typedef std::pair<std::string, std::string> StringPair;

    /**
     * @brief Convenient definition of a list of string pairs.
     * @relatesalso rw::common::StringUtil
     */
    typedef std::vector<StringPair> StringPairList;

	/**@}*/


}} // end namespaces

#endif // end include guard
