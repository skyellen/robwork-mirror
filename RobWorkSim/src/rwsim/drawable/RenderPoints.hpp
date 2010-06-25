#ifndef RWSIM_DRAWABLE_RENDERPOINTS_HPP_
#define RWSIM_DRAWABLE_RENDERPOINTS_HPP_

//! @file RenderPoints.hpp

#include <list>
#include <vector>

#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/State.hpp>

#include <rwlibs/drawable/Render.hpp>

namespace rwsim {
namespace drawable {
	//! @addtogroup drawable @{

	/**
	 * @brief renderer for rendering points
	 */
	class RenderPoints: public rwlibs::drawable::Render
	{
	public:
		/**
		 * @brief constructor
		 */
		RenderPoints();

		virtual ~RenderPoints();

		/**
		 * @brief add a single point
		 * @param p [in] point to add
		 */
		void addPoint(const rw::math::Vector3D<>& p){ _points.push_back(p); };

		/**
		 * @brief adds points to the list of points
		 * @param points [in] points to be added
		 */
		void addPoints(const std::vector<rw::math::Vector3D<> >& points);

		/**
		 * @brief sets the points that are t be rendered
		 * @param points [in] points to be rendered
		 */
		void setPoints(const std::vector<rw::math::Vector3D<> >& points);

		/**
		 * @brief set the color used for the model
		 * @param r [in] red color value
		 * @param g [in] green color value
		 * @param b [in] blue color value
		 */
		void setColor(double r, double g, double b);

		/**
		 * @brief clear the list of points
		 */
		void clear();

		//! @copydoc Render::draw
		void draw(DrawType type, double alpha) const;
	private:
		GLUquadricObj* _sphereObj;
		std::vector<rw::math::Vector3D<> > _points;
		float _color[3];
	};
	//! @}
}
}

#endif /*RenderGhost_HPP_*/
