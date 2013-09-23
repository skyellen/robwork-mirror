/**
 * @file XMLHelpers.hpp
 * @author Adam Wolniakowski
 * @brief Based on RobWork source. Compiled here for convenience.
 */

#pragma once

#include <string>
#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>



namespace rwlibs {
	namespace xml {

/// Boost property tree shorthand.	
typedef boost::property_tree::ptree PTree;

/// Boost property tree iterator shorthand.
typedef PTree::iterator CI;
		
/**
 * @class XMLHelpers
 * @brief Contains assorted methods for parsing XML files.
 */
class XMLHelpers
{
	public:		
	// static methods
		static std::vector<double> readArray(PTree& tree);
		static double readInt(PTree& tree);
		static double readDouble(PTree& tree);
		static rw::math::Q readQ(PTree& tree);
	
	protected:
	// static methods
		static std::pair<bool, int> toInt(const std::string& str);
		static std::pair<bool, double> toDouble(const std::string& str);
};

}} // end namespaces
