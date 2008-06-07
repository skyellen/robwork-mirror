#ifndef RWLIBS_ALGORITHMS_CONTOUREXTRACTOR_HPP_
#define RWLIBS_ALGORITHMS_CONTOUREXTRACTOR_HPP_

#include <rw/sensor/Image.hpp>
#include <boost/dynamic_bitset.hpp>

namespace rwlibs {
namespace algorithms {


/**
 * @brief Extracts contours in an Area Of Interest (AOI) of an image. 
 */

class ContourExtractor
{
public:	
	typedef std::vector<size_t> Contour; 
	
	/**
	 * @brief Constructs an contour extractor that extract contours in the 
	 * specified Area Of Interest 
	 */
	ContourExtractor(size_t offset_x, size_t offset_y, size_t width, size_t height):
		_xoff(offset_x),_yoff(offset_y),
		_width(width),_height(height),
		_minRegionSize(40),_treated(_width*_height)
	{ }
	
	/**
	 * @brief destructor
	 */
	virtual ~ContourExtractor(){};
	
	/**
	 * @brief resets the ContourExtractor and sets a new target image
	 * and threshold
	 * @param image [in] a grayscale image that should be used to search for contours
	 * @param threshold [in] threshold in the range [0-1]
	 */
	void reset(rw::sensor::Image& image, double threshold);
	
	/**
	 * @brief searches for a contour in the specified AOI, 
	 * returns false if no contour was found
	 * @param contour [out] data container for contour
	 * @return true if contour detected before end of AOI, false otherwise. 
	 */
	bool findNextContour(Contour& contour);
	
	/**
	 * @brief sets the AOI for this contour extractor. Search position will be
	 * reset to (0,0) of AOI which is (offset_x,offset_y). 
	 * @param offset_x [in] the start in x-axis of AOI 
	 * @param offset_y [in] the start in y-axis of AOI
	 * @param width [in] the width of AOI
	 * @param height [in] the height of AOI
	 */
	void setAOI(size_t offset_x, size_t offset_y, size_t width, size_t height);
	
	/**
	 * @brief gets the current pixel position of the contour search 
	 */
	std::pair<size_t,size_t> getSearchPos(){
		return std::pair<size_t,size_t>(_nextStart%_width,_nextStart/_width);
	}
	
	bool isAtEnd(){
	    return _nextStart == _width*(_height-1);
	}
	
private:

	struct Coord {
		Coord(int xp,int yp):x(xp),y(yp){}
		int x,y;
	};

	struct Region {
		size_t getSize();
	};
		
	inline bool isItContour(size_t i){
		return _img[i] < _thres; 
	}

	void followContour(int thres, size_t startIdx, Contour &c);

	void setImageBorder(unsigned char bordercolor, 
						int thickness);
	
	inline bool isTreated(Coord pos){
		return _treated[ _width*pos.y + pos.x ];
	}
	
private: // private variables
	
	size_t _xoff,_yoff,_width,_height;
	size_t _minRegionSize;
	boost::dynamic_bitset<>  _treated;
	
	size_t _nextStart;
	
	// aux vars
	unsigned char *_img;
	size_t _thres;
};

} // namespace algorithms
} // namespace rwlibs

#endif /*CONTOUREXTRACTOR_HPP_*/
