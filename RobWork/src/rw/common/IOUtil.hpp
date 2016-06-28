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


#ifndef RW_COMMON_IOUTIL_HPP
#define RW_COMMON_IOUTIL_HPP

/**
 * @file IOUtil.hpp
 */

#include <string>
#include <vector>
#include <ctime>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Collection of IO related utilities
     */
    class IOUtil
    {
    public:
        /**
         * @brief Read the contents of a file.
         *
         * The contents of the file \b file_name are read into the buffer \b result. The
         * \b result buffer is resized to match the length of the input.
         *
         * Files can be lengthy which is why the result is returned via the \b result
         * parameter (an alternative would be to return a smart pointer to a vector).
         *
         * An exception is thrown if the file can't be opened or can't be read.
         *
         * @param file_name [in] The name of the file.
         *
         * @param result [out] Buffer to which the file contents are written.
         */
        static void readFile(const std::string& file_name, std::vector<char>& result);


        /** @brief Attach to \b filename the proper extension.
         *
         * Find the the suffix of \b extensions that when appended to \b filename
         * yields the file name for readable file.
         *
         * For suffix of \b extensions, lower case and upper case versions of this
         * suffix are also considered.
         *
         * The file name \b filename as is with nothing prepended at all is
         * considered as the first thing.
         *
         * If \b filename can't be resolved to a readable file, an informative error
         * message is thrown.
         *
         * @param filename [in] The file name to resolve.
         *
         * @param extensions [in] The sequence of file name suffixes.
         *
         * @return The filename with extension
         */
        static std::string resolveFileName(const std::string& filename,
                                           const std::vector<std::string>& extensions);


        /**
         * @brief Emit an assertion message and kill the program.
         *
         * rwAssert() is called by the RW_ASSERT() macro.
         *
         * rwAssert() is for internal use only.
         *
         * @param expression [in] The thing that wasn't true.
         *
         * @param file [in] The originating file name.
         *
         * @param line [in] The originating line number.
         */
        static void rwAssert(const char* expression, const char* file, int line);


        /**
         * @brief returns the absolute filename of file. If file is
         * absolute filename then \b file is returned. If not, then
         * working directory path is appended to \b file.
         * @param file [in] the relative or absolute filename
         */
        static std::string getAbsoluteFileName(const std::string& file);

        /**
         * @brief tests if this machine use little or big endian.
         * @return true if machine
         */
        static bool isLittleEndian();

        /**
         * @brief Returns files in the folder specified by path
         *
         * Throws rw::common::Exception if unable to obtain file names.
         *
         * @param path [in] Path to search in
         * @param recursive [in] if true files in subfolders are also added
         * @param addPath [in] If true both path and filenames are returned. Otherwise only the filenames are returned.
         * @param mask [in] Mask to filter filenames. Example: "*.dll" to return all DLL files.
         * @return Files located at \b path.
         */
        static std::vector<std::string> getFilesInFolder(const std::string& path, bool recursive, bool addPath = true, const std::string& mask = "*");

        /**
         * @brief get all files in a folder
         * @param path [in] folder path
         * @param fileMask [in] file mask. Example: "*.dll" to return all DLL files.
         * @param recursive [in] if true files in subfolders are also added
         * @param addPath [in] if true then folder paths are added to solution
         * @param result [out] all files and if \b addPath
         */
        static void getFilesInFolder(const std::string& path, const std::string& fileMask, bool recursive, bool addPath, std::vector<std::string>& result);

        /**
         * @brief get size of file
         * @param filename [in] name of file
         * @return the size of file in bytes
         */
        static size_t getFileSize(const std::string& filename);

        /**
         * @brief get the last date of writing of the file \b filename
         * @param filename [in] name of file
         * @return time of modification
         */
        static std::time_t getLastFileWrite(const std::string& filename);

        /**
         * @brief extracts the first element tag in any xml document.
         *
         * throws if not xml document or file cannot be openned
         * @param filename [in] name of the xml file
         * @return name of first xml element
         */
        static std::string getFirstXMLElement(const std::string& filename);

        /**
         * @brief extracts the first element tag in any xml document.
         *
         * throws if not xml document
         * @param inputStream [in] ifstream containing the xml document
         * @return name of first xml element
         */
        static std::string getFirstXMLElement(std::istream & inputStream);
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
