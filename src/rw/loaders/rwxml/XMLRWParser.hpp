/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_LOADERS_XMLDEVICEPARSER_HPP
#define RW_LOADERS_XMLDEVICEPARSER_HPP

#include "XMLParserUtil.hpp"

#include <vector>

#include <boost/numeric/ublas/vector.hpp>

#include <boost/spirit.hpp>
#include <boost/spirit/core.hpp>
#include <boost/spirit/phoenix.hpp>
#include <boost/spirit/symbols/symbols.hpp>

#include <boost/spirit/tree/common.hpp>
#include <boost/spirit/tree/ast.hpp>
#include <boost/spirit/tree/parse_tree.hpp>
#include <boost/spirit/iterator/position_iterator.hpp>

#include <rw/math/Constants.hpp>

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
                                                                  boost::shared_ptr<std::vector< std::pair<size_t,boost::spirit::file_position> > > &filemap);
        };

    /*@}*/
}} // end namespaces

#endif // end include guard
