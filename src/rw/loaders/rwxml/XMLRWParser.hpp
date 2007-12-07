#ifndef XMLDEVICEPARSER_HPP_
#define XMLDEVICEPARSER_HPP_

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
    class XMLRWParser {
    public:
    	
    	/**
    	 * @brief parse a workcell from a filename
    	 * @param filename [in] 
    	 * @return dummy representation of a workcell 
    	 */
        static boost::shared_ptr<DummyWorkcell> parseWorkcell( 
        		const std::string& filename);

    	/**
    	 * @brief parse a workcell from a vector of char data and a file pos map
    	 * @param data [in]
    	 * @return dummy representation of a workcell 
    	 */
        static boost::shared_ptr<DummyWorkcell> parseWorkcell(
                boost::shared_ptr<std::vector<char> > &data,
                boost::shared_ptr<std::vector< std::pair<size_t,boost::spirit::file_position> > > &filemap);
        
    };
    /*@}*/
}}

#endif /*XMLDEVICEPARSER_HPP_*/
