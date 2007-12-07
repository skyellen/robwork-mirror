#ifndef XMLPREPARSER_H_
#define XMLPREPARSER_H_

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
#endif /*XMLPREPARSER_H_*/

