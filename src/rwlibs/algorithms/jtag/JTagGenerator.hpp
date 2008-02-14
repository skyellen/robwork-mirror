#ifndef RWLIBS_ALGORITHMS_JTAGGENERATOR_HPP_
#define RWLIBS_ALGORITHMS_JTAGGENERATOR_HPP_

namespace rwlibs {
namespace algorithms {

class JTagGenerator
{
public:
	JTagGenerator();
	virtual ~JTagGenerator();
	
	static void drawJTag(int id, rw::math::Vector2D<> pos, double angle);
	
};

} // namespace algorithms
} // namespace rwlibs

#endif /*JTAGGENERATOR_HPP_*/
