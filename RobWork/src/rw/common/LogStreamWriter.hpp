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


#ifndef RW_COMMON_LOGSTREAMWRITER_HPP
#define RW_COMMON_LOGSTREAMWRITER_HPP

#include <iosfwd>
#include "LogWriter.hpp"

namespace rw { namespace common {

	/** @addtogroup common */
	/*@{*/

	/**
     * @brief Writes log output to a std::ostream
     */
    class LogStreamWriter : public LogWriter
    {
    public:
        /**
         * @brief Constructs LogStreamWriter with a target output stream
         *
         * The LogStreamWriter keeps a reference to the stream object. Destroying
         * the stream object while the LogStreamWriter has a reference to it
         * results in undefined behavior.
         *
         * Ownership of the stream is not taken.
         *
         * @param stream [in] Stream to write to
         */
        LogStreamWriter(std::ostream* stream);

        /**
         * @brief Destructor
         *
         * Calls flush on the output stream before destruction
         */
        ~LogStreamWriter();

	protected:
        /**
         * @copydoc LogWriter::write(const std::string&)
         */
        void doWrite(const std::string& str);

        /**
         * @brief Calls flush on the ostream
         */
        void doFlush();

		/** 
	 	 * @copydoc LogWriter::setTabLevel(int)
		 */
		void doSetTabLevel(int tabLevel);

    private:
        std::ostream* _stream;
		int _tabLevel;
    };

	/*@}*/
}} // end namespaces

#endif // end include guard
