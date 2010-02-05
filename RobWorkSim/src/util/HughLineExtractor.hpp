/*
 * HughLineExtractor.hpp
 *
 *  Created on: 06-01-2009
 *      Author: jimali
 */

#ifndef HUGHLINEEXTRACTOR_HPP_
#define HUGHLINEEXTRACTOR_HPP_

#include <rw/sensor/Image.hpp>
#include <rw/math/Line2D.hpp>
#include <vector>

class HughLineExtractor {
public:
	static std::vector<rw::math::Line2D > extractLines(rw::sensor::Image& img);

};

#endif /* HUGHLINEEXTRACTOR_HPP_ */
