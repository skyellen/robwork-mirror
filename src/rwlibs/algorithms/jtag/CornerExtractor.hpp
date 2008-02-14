#ifndef RWLIBS_ALGORITHMS_CORNEREXTRACTOR_HPP_
#define RWLIBS_ALGORITHMS_CORNEREXTRACTOR_HPP_

#include <rw/sensor/Image.hpp>
#include "ContourExtractor.hpp"
#include "Line2D.hpp"

#include <iostream>
#include <vector>

#include <rw/math/Vector2D.hpp>

namespace rwlibs {
namespace algorithms {


class CornerExtractor
{
public:
	
	struct Corner {
		rw::math::Vector2D<> p;
		Line2D l1,l2;
	};
	
	CornerExtractor(float angleThres, size_t minnr=0, size_t maxnr=100, 
					float minSize=0.0, float maxSize=1000.0):
		_minNr(minnr),_maxNr(maxnr),
		_minSize(minSize),_maxSize(maxSize)
	{
		
	}
	
	virtual ~CornerExtractor(){};
	
	/**
	 * @brief searches for cornors in the contour. If too few or too many
	 * corners are found false is returned. 
	 */
	bool findCorners(rw::sensor::Image& srcimg,
					const ContourExtractor::Contour &contour,
					std::vector<Corner> &cornors);

private:
	
private:
	size_t _minNr, _maxNr;
	float _minSize, _maxSize;
	
};

} // namespace algorithms
} // namespace rwlibs

#endif /*CCORNOREXTRACTOR_HPP_*/
