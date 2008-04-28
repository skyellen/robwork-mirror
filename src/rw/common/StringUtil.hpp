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

#ifndef rw_common_StringUtil_HPP
#define rw_common_StringUtil_HPP

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

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static bool IsAbsoluteFileName(const std::string& file)
        { return isAbsoluteFileName(file); }
#endif /* RW_REMOVE_DEPRECATED */

        /**
         * @brief Replace '\' with '/' everywhere in the file name \b file.
         */
        static std::string replaceBackslash(const std::string& file);

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static std::string ReplaceBackslash(const std::string& file)
        { return replaceBackslash(file); }
#endif /* RW_REMOVE_DEPRECATED */

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

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static std::string GetDirectoryName(const std::string& path)
        { return getDirectoryName(path); }
#endif /* RW_REMOVE_DEPRECATED */

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

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static std::string GetFileExtension(const std::string& filename)
        { return GetFileExtension(filename); }
#endif /* RW_REMOVE_DEPRECATED */

        /**
         * @brief The uppercase of a string.
         *
         * @param str [in] A string to uppercase.
         *
         * @return \b str converted to upper case.
         */
        static std::string toUpper(const std::string& str);

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static std::string ToUpper(const std::string& str)
        { return toUpper(str); }
#endif /* RW_REMOVE_DEPRECATED */

        /**
         * @brief The lowercase of a string.
         *
         * @param str [in] A string to lowercase.
         *
         * @return \b str converted to lower case.
         */
        static std::string toLower(const std::string& str);

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static std::string ToLower(const std::string& str)
        { return toLower(str); }
#endif /* RW_REMOVE_DEPRECATED */

        /** @brief The string \b str enclosed in single-quotes.
         *
         * Use this for quoting of strings in user error messages.
         */
        static std::string quote(const std::string& str);

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        static std::string Quote(const std::string& str)
        { return quote(str); }
#endif /* RW_REMOVE_DEPRECATED */
    };

    /**@}*/

}} // end namespaces

#endif // end include guard
