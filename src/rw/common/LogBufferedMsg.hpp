/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_COMMON_LOGBUFFEREDMSG_HPP
#define RW_COMMON_LOGBUFFEREDMSG_HPP

#include <ostream>
#include <vector>
#include <string>
#include "LogWriter.hpp"

namespace rw { namespace common {

    /**
     * @brief Buffers messages before writing them to the output stream.
     *
     * The size of the buffer is not fixed and will grow until flush is called.
     * To have a fixed size buffer use LogBufferedChar instead.
     */
    class LogBufferedMsg: public LogWriter
    {
    public:
        /**
         * @brief Constructs LogBufferedMsg with a target ostream
         *
         * The LogBufferedMsg keeps a reference to the stream object. Destroying
         * the stream object while the LogBufferedMsg has a reference to it
         * results in undefined behavior.
         *
         * @param stream [in] Stream to write to
         */
        LogBufferedMsg(std::ostream* stream);

        /**
         * @brief Destructor
         *
         * Calls flush before destruction
         */
        virtual ~LogBufferedMsg();

        /**
         * @brief Writes str to the buffer
         * @param str [in] str to write
         */
        virtual void write(const std::string& str);

        /**
         * @brief Write content of buffer to output stream and flush it
         */
        virtual void flush();

    private:
        std::vector<std::string> _buffer;
        std::ostream* _stream;
    };

}} // end namespaces

#endif // end include guard
