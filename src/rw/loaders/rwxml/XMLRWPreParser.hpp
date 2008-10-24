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

#ifndef RW_LOADERS_XMLPREPARSER_HPP
#define RW_LOADERS_XMLPREPARSER_HPP

#include <boost/spirit.hpp>
#include <boost/spirit/core.hpp>
#include <boost/spirit/phoenix.hpp>
#include <boost/spirit/tree/common.hpp>
#include <boost/spirit/tree/ast.hpp>
#include <boost/spirit/tree/parse_tree.hpp>
#include <boost/spirit/iterator/position_iterator.hpp>
#include <boost/spirit/utility/functor_parser.hpp>
#include <boost/spirit/error_handling/exceptions.hpp>
#include <boost/spirit/actor.hpp>
#include <iostream>
#include <cassert>
#include <vector>

#include <sstream>

#include <boost/numeric/ublas/vector.hpp>

#include "DependencyGraph.hpp"

#include <rw/common/StringUtil.hpp>

namespace rw {
namespace loaders {
	/** @addtogroup loaders */
	/*@{*/

	/**
	 * @brief Pre-parser for the XML RW format. All include, define, comments and use
	 * elements are handled.
	 */
    class XMLRWPreParser {
    public:

        /**
         * @brief Pre-parser for the XML RW format. All include, define, comments and use
         * elements are handled.
         *
         * @param filename [in] name of file to parse
         * @param output [out] the data result of parsing the file
         * @param filemap [out] the position to file info result of parsing the file
         *
         * @return true if pasing was a succes, false otherwise
         */
        static bool parse(
            const std::string& filename,
            std::vector<char> &output,
            std::vector< std::pair<size_t,boost::spirit::file_position> > &filemap);

        /**
         * @brief Pre-parses for the XML RW format. All include, define, comments and use
         * elements are handled.
		 *
         * @param filename [in] name of file to parse
         * @param output [out] the data result of parsing the file
         * @param filemap [out] the position to file info result of parsing the file
         * @param graph [in] a dependency graph between file includes
         *
         * @return true if pasing was a succes, false otherwise
         */
        static bool parse(
            const std::string& filename,
            std::vector<char> &output,
            std::vector< std::pair<size_t,boost::spirit::file_position> > &filemap,
            DependencyGraph &graph );

    };
    /*@}*/
}
}
#endif /*RW_LOADERS_XMLPREPARSER_HPP*/

