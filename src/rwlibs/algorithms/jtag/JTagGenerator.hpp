#ifndef RWLIBS_ALGORITHMS_JTAGGENERATOR_HPP_
#define RWLIBS_ALGORITHMS_JTAGGENERATOR_HPP_

#include "JTagMarker.hpp"

namespace rwlibs {
namespace algorithms {

class JTagGenerator
{
public:
	
	static void DrawJTagToPS(const JTagMarker &mark, const std::string& filename, double width);
	
};

} // namespace algorithms
} // namespace rwlibs

#endif /*JTAGGENERATOR_HPP_*/
