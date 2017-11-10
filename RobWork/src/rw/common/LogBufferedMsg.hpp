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

	protected:
        /**
         * @brief Writes str to the buffer
         * @param str [in] str to write
         */
        virtual void doWrite(const std::string& str);

        /**
         * @brief Write content of buffer to output stream and flush it
         */
        virtual void doFlush();

		/**
		 * @copydoc LogWriter::setTabLevel
		 */
		virtual void doSetTabLevel(int tablevel);

    private:
		std::vector<std::pair<std::string, int> > _buffer;
        std::ostream* _stream;
		int _tabLevel;
    };

}} // end namespaces

#endif // end include guard
