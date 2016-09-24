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

#ifndef RW_COMMON_ARCHIVE_HPP
#define RW_COMMON_ARCHIVE_HPP

#include <iosfwd>
#include <string>

namespace rw {
namespace common {
	/**
	 * @brief archive interface for serializaion classes.
	 */
	class Archive {
	public:
		//! destructor
		virtual ~Archive(){};

		/**
		 * @brief open file for reading and writing
		 * @param filename
		 */
		void open(const std::string& filename){ doOpenArchive(filename); };

		/**
		 * @brief initialize archive for reading and/or writing to a stream
		 * @param stream [in] the stream
		 */
		void open(std::iostream& stream){ doOpenArchive(stream); };

		/**
		 * @brief open an output stream for writing
		 */
		void open(std::ostream& ofs){ doOpenOutput(ofs); }

	    //! @brief open an inputstream for reading
	    void open(std::istream& ifs){ doOpenInput(ifs); };



		/**
		 * @brief test if this archive is openned for manipulation. If this is false then
		 * no storage will be performed.
		 * @return true if Archive is ready for streaming
		 */
		virtual bool isOpen() = 0;

		/**
		 * @brief close the archive.
		 */
		virtual void close() = 0;

		/**
		 * @brief flush the archive. Anything stored in buffers will be flushed to the
		 * actual media that has been openned.
		 */
		virtual void flush() = 0;

		// TODO: make extension point for archives
	protected:
		//! @copydoc open(const std::string&)
		virtual void doOpenArchive(const std::string& filename) = 0;
		//! @copydoc open(std::iostream&)
		virtual void doOpenArchive(std::iostream& stream) = 0;
		//! @copydoc open(std::istream&)
		virtual void doOpenInput(std::istream& ifs) = 0;
		//! @copydoc open(std::ostream&)
		virtual void doOpenOutput(std::ostream& ofs) = 0;
	};

}}

#endif
