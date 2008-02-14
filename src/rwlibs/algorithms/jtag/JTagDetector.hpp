#ifndef RWLIBS_ALGORITHMS_JTAGDETECTOR_HPP_
#define RWLIBS_ALGORITHMS_JTAGDETECTOR_HPP_

#include <boost/numeric/ublas/matrix.hpp>

#include "JTagMarker.hpp"
#include <rw/math/Vector2D.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/math/PerspectiveTransform2D.hpp>

#include "CornerExtractor.hpp"
#include "ContourExtractor.hpp"

namespace rwlibs {
namespace algorithms {


/**
 * @brief Detects JTagMarkers in an area of an image. The area is rectangular.
 * The algorithm searches sequentially for JTagMarkers starting from some given 
 * offset in the AOI. The search is at an end, when the algorithm has covert 
 * all pixels in the AOI.
 */
class JTagDetector
{
public:
	/**
	 * @brief Constructs a JTagDetector to be used in some area of interest (AOI) 
	 * in an image
	 * @param width [in] pixel width of the area of interest (AOI)
	 * @param height [in] pixel height of the AOI
	 * @param offset_x [in] pixel offset of AOI in x-axis
	 * @param offset_y [in] pixel offset of AOI in y-axis
	 */
	JTagDetector(size_t offset_x, size_t offset_y,
				 size_t width, size_t height);
	
	/**
	 * @brief Destructor
	 */
	virtual ~JTagDetector();
	
	/**
	 * @brief Sets the dimensions of the area of interest
	 * @param width [in] pixel width of the area of interest (AOI)
	 * @param height [in] pixel height of the AOI
	 * @param offset_x [in] pixel offset of AOI in x-axis
	 * @param offset_y [in] pixel offset of AOI in y-axis  
	 */
	void setAOI(size_t width, size_t height,
				size_t offset_x=0, size_t offset_y=0);
	
	/**
	 * @brief 
	 */
	void reset(rw::sensor::Image& img){
		_img = &img;
		_atEnd = false;
		_contourExt.reset(img, _thres);
		
	}
	
	/**
	 * @brief searches for a specific JTagMarker in the AOI, starting 
	 * at (x,y).
	 * @param id [in] the id to search for
	 * @param mark [out] the result if the marker is found
	 * @return true if mark is detected
	 */
	bool detect(int id, JTagMarker& mark);
	
	/**
	 * @brief detects all markers in the AOI and returns them in a list. 
	 * Only returns markers detected after last reset.
	 * @return all markers that has been detected in the AOI.
	 */
	std::vector<JTagMarker> detectAll();
	
	/**
	 * @brief use to check if end of AOI is reached
	 * @return true if end of AOI is reached, false otherwise
	 */
	bool isAtEnd(){
		return _atEnd;
	}
	
private:
	
	bool locateNext(size_t x, size_t y, JTagMarker &marker);

private:
	size_t _width, _height;
	rw::sensor::Image *_img;
	ContourExtractor _contourExt;
	CornerExtractor _cornerExt;
	ContourExtractor::Contour _contour;
	
	int _thres,_whiteThres,_blackThres;
	
	bool _atEnd;
	size_t _x,_y;
};

} // namespace algorithms
} // namespace rwlibs

#endif /*JTagDetector_HPP_*/
