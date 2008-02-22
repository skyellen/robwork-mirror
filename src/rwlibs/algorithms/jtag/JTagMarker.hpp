#ifndef RWLIBS_ALGORITHMS_JTAGMARKER_HPP_
#define RWLIBS_ALGORITHMS_JTAGMARKER_HPP_

#include <rw/math/Vector2D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/PerspectiveTransform2D.hpp>

#include <rw/sensor/Image.hpp>

namespace rwlibs {
namespace algorithms {

/**
 * @brief The JTagMarker class defines a visual 2d quadrant barcode marker. It 
 * is defined as a nxn matrix of black and white equally sized quadrants, where the inner 
 * matrix (n-2)x(n-2) defines the orientation and identification of the marker. 
 * The inner matrix starts at (1,1) where the outer starts at (0,0). The outer matrix is
 * a boundary and this boundary is completely black.
 * 
 * Example of a 5x5 marker, where orientation is identity matrix:
 * 
 * D  D  D  D  D 
 * D  B  1  C  D
 * D  0  2  4  D
 * D  A  3  5  D
 * D  D  D  D  D
 * 
 * where D is always black, A, B and C is used for orientation determination and are
 * always (black, black and white), the numbers from 0 to 5 indicate a bit in the 
 * id. If a bit is black its value is 1 else its 0. 
 * 
 */

class JTagMarker
{
public:
	/**
	 * @brief Constructs a JTagMarker with a specific id.
	 * @param id [in] id of the marker that is to be created.
	 */
	JTagMarker(int id=0);
	
	/**
	 * @brief Destructor
	 */
	virtual ~JTagMarker(){};

	/**
	 * @brief calculates the transform of this JTagMarker given the
	 * actual width and height.
	 * @param width [in] width of physical marker in mm
	 * @param height [in] height of physical marker in mm
	 * @return 3d transform of the marker relative to the camera pinhole
	 */
	rw::math::Transform3D<> getTransform3D(double width, double height);
	
	/**
	 * @brief gets the center position of the JTagMarker in the image
	 * @return position of the center of the marker in pixel values 
	 */
	rw::math::Vector2D<> getPos2D(){
		return _pos;
	};
		
	/**
	 * @brief Parses a (n-2)x(n-2) size matrix of probabilities to a JTagMarker.
	 * The matrix describe the probability/blackness of the individual bits.
	 * where 0 is completely white and 1 is completely black. 
	 * If the matrix does not describe a valid JTagMarker false is returned.  
	 */
	static bool ParseMatrix(boost::numeric::ublas::matrix<double> valueMatrix,
							JTagMarker &marker);
	
	
	rw::math::PerspectiveTransform2D<> _htf;

	int _id;
	rw::math::Vector2D<> _pos;
	bool _initialized;
	
	rw::math::Vector2D<> _corners;
	boost::numeric::ublas::matrix<double> _vals;

private:
	void setArray(int id);	
};

} // namespace algorithms
} // namespace rwlibs

#endif /*JTAGMARKER_H_*/
