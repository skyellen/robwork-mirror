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


#ifndef RW_LOADERS_XMLDEVICEPARSER_HPP
#define RW_LOADERS_XMLDEVICEPARSER_HPP

#include "XMLParserUtil.hpp"

#include <vector>

#include <boost/spirit/include/classic_position_iterator.hpp>
#include <boost/shared_ptr.hpp>

namespace rw { namespace loaders {
	/** @addtogroup loaders */
	/*@{*/

	/**
	 * @brief parses a workcell from the xml fileformat into a dummmy workcell
	 * representation.
	 */
    class XMLRWParser
        {
        public:

            /**
             * @brief parse a workcell from a filename
             * @param filename [in]
             * @return dummy representation of a workcell
             */
            static boost::shared_ptr<DummyWorkcell>
                    parseWorkcell(const std::string& filename);

            /**
             * @brief parse a workcell from a vector of char data and a file pos map
             * @param data [in]
             * @param filemap [in]
             * @return dummy representation of a workcell
             */
            static boost::shared_ptr<DummyWorkcell> parseWorkcell(boost::shared_ptr<std::vector<char> > &data,
                                                                  boost::shared_ptr<std::vector< std::pair<size_t,boost::spirit::classic::file_position> > > &filemap);
        };

    /*@}*/
}} // end namespaces

#endif // end include guard
