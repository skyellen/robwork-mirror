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


#ifndef RW_COMMON_LOGBUFFEREDCHAR_HPP
#define RW_COMMON_LOGBUFFEREDCHAR_HPP

#include <ostream>
#include <vector>
#include "LogWriter.hpp"

namespace rw {
namespace common {

/** @addtogroup common */
/*@{*/

/**
 * @brief Buffers characters before writing them to the output stream.
 *
 * The size of the buffer is fixed. On overflow the behavior depends on the
 * specified OverFlowPolicy. If a single message is larger than
 * the entire content of the buffer it is truncated.
 *
 */
class LogBufferedChar: public LogWriter
{
public:
    /**
     * @brief Behaviors for the OverflowPolicy
     */
    enum OverflowPolicy {
        //! Remove the first added content (circular queue)
        REMOVE_FIRST,
        //! Skip the content which does not fit input the buffer
        REMOVE_LAST,
        //! Automatically calls flush to write the buffer and the new message to the output stream. Using AUTO_FLUSH it is possible to write messages larger than the buffer size.
        AUTO_FLUSH };

    /**
     * @brief Constructs a LogBufferedChar
     *
     * The LogBufferedMsg keeps a reference to the stream object. Destroying
     * the stream object while the LogBufferedChar has a reference to it
     * results in undefined behavior.
     *
     * @param size [in] Size of buffer (in characters)
     * @param stream [in] Stream to write to
     * @param policy [in] Overflow policy. Default is REMOVE_FIRST
     */
	LogBufferedChar(size_t size, std::ostream* stream, OverflowPolicy policy = REMOVE_FIRST);

	/**
	 * @brief Destructor
	 *
	 * Calls flush before destroying the object
	 */
	virtual ~LogBufferedChar();

protected:
	/**
	 * @brief Writes str to the buffer
	 * @param str [in] String to write
	 */
	void doWrite(const std::string& str);


	/**
	 * @brief Flushes the buffer to the output stream
	 */
	void doFlush();

	/**
	 * @copydoc LogWriter::setTabLevel
	 */
	void doSetTabLevel(int tablevel);

private:
    std::ostream* _stream;
    std::vector<char> _buffer;
    size_t _size;
    int _index;
    OverflowPolicy _policy;
    bool _overflow;
	
    char* get(int index) { return &_buffer[0] + index; }
	int _tabLevel;
};


/* @} */

} //end for namespace sandbox
} //end for namespace rw

#endif /*RW_COMMON_LOGBUFFEREDCHAR_HPP*/
