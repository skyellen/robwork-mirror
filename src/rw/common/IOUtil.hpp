/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#ifndef rw_common_IOUtil_HPP
#define rw_common_IOUtil_HPP

/**
 * @file IOUtil.hpp
 */

#include <string>
#include <vector>

#include <string>
#include <vector>

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

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static void ReadFile(const std::string& file_name, std::vector<char>& result)
        { return ReadFile(file_name, result); }
#endif /* RW_REMOVE_DEPRECATED */

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
        static std::string resolveFileName(
            const std::string& filename,
            const std::vector<std::string>& extensions);

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static std::string ResolveFileName(
            const std::string& filename,
            const std::vector<std::string>& extensions)
        { return resolveFileName(filename, extensions); }
#endif /* RW_REMOVE_DEPRECATED */

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
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
