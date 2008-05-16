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

#ifndef RW_COMMON_LOGSTREAMWRITER_HPP_
#define RW_COMMON_LOGSTREAMWRITER_HPP_

#include <ostream>
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

        /**
         * @copydoc LogWriter::write(const std::string&)
         */
        void write(const std::string& str);

        /**
         * @brief Calls flush on the ostream
         */
        void flush();

    private:
        std::ostream* _stream;
    };

	/*@}*/
}} // end namespaces

#endif // end include guard
