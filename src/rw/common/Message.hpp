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

#ifndef rw_common_Message_HPP
#define rw_common_Message_HPP

/**
 * @file Message.hpp
 */

#include <iostream>
#include <string>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /** 
     * @brief Standard type for user messages of robwork.
     *
     * Messages are used for exception, warnings, and other things that are
     * reported to the user.
     *
     * Message values should contain the source file name and line number so
     * that it is easy to look up the place in the code responsible for the
     * generation of the message.
     *
     * RW_THROW and RW_WARN of macros.hpp have been introduced for the throwing
     * of exceptions and emission of warnings.
     */
    class Message
    {
    public:
        /** 
         * @brief Constructor
         *
         * Messages of RobWork are all annotated by the originating file name,
         * the originating line number, and a message text for the user.
         *
         * Supplying all the file, line, and message parameters can be a little
         * painfull, so a utility for creating messages is available from the
         * file macros.hpp.
         *
         * @param file [in] The source file name.
         *
         * @param line [in] The source file line number.
         *
         * @param message [in] A message for a user.
         */
        Message(
            const std::string& file,
            int line,
            const std::string& message)
            :
            _file(file),
            _line(line),
            _message(message)
        {}

        /** 
         * @brief The name of source file within which the message was
         * constructed.
         *
         * @return The exception file name.
         */
        const std::string& getFile() const { return _file; }

        /** 
         * @brief The line number for the file at where the message was
         * constructed.
         *
         * @return The exception line number.
         */
        int getLine() const { return _line; }

        /** 
         * @brief The message text meant for the user.
         *
         * @return The message text.
         */
        const std::string& getText() const { return _message; }

    private:
        std::string _file;
        int _line;
        std::string _message;
    };

    /** @brief Format to \b out the message \b msg.
     *
     * The format for the exception is
\code
<file>:<line> <message>
\endcode
     * @return The stream \b out.
     */
    std::ostream& operator<<(std::ostream& out, const  Message& msg);

    /*@}*/
}} // end namespaces

#endif // end include guard
