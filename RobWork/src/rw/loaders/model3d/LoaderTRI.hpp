#ifndef RW_DRAWABLE_LOADERTRI_H
#define RW_DRAWABLE_LOADERTRI_H

#include <rw/graphics/Model3D.hpp>
#include "../Model3DLoader.hpp"

#include <string>

namespace rw {
namespace loaders {
    //! @addtogroup graphics
	// @{

	/**
	 * @brief loader for a simple ASCI based triangle format. The format is NOT
	 * a standard. There are two elements in the format each is defined in a line
	 *
	 * \verbatim
	 * color <float> <float> <float>
	 * point <float> <float> <float> <float> <float> <float>
	 * \endverbatim
	 *
	 * line comments can be added by starting the line with '#', '!', '\0'(blank line) or '$'
	 *
	 * the color is RGB and the point has a position (first three floats) and
	 * a normal (last three floats). Points are implicitly structured as triangles
	 * such that:
	 *
	 * \code
	 * triangle(points[nrPoints%3],points[nrPoints%3+1],points[nrPoints%3+2]);
	 * \endcode
	 *
	 * all triangles after a specific color element, will be drawn with the RGB values specified
	 *
	 */
	class LoaderTRI: public Model3DLoader
	{
	public:
		/**
		 * @brief constructor
		 */
		LoaderTRI();

		/**
		 * @brief destructor
		 */
		virtual ~LoaderTRI();

		//! @copydoc Model3DLoader::load
		rw::graphics::Model3D::Ptr load(const std::string& filename);

	};

	//! @}
}
}
#endif // RW_DRAWABLE_LOADER_3DS_H
