/*
 * GiftWrapHull.hpp
 *
 *  Created on: 30-03-2009
 *      Author: jimali
 */

#ifndef RW_GEOMETRY_GIFTWRAPHULL2D_HPP_
#define RW_GEOMETRY_GIFTWRAPHULL2D_HPP_

#include "ConvexHull2D.hpp"
#include <stack>
#include <set>
#include <vector>

namespace rw {
namespace geometry {
	//! @addtogroup geometry @{

	/**
	 * @brief a convexhull calculator that use the giftwrap algorithm to compute
	 * 2d convex hulls
	 */
	class GiftWrapHull2D : public ConvexHull2D {
	public:
		//! @brief constructor
		GiftWrapHull2D(){};

		//! @brief destructor
		virtual ~GiftWrapHull2D(){};

		//! @copydoc ConvexHull2D::rebuild
		void rebuild(const std::vector<rw::math::Vector2D<> >& vertices);

		//! @copydoc ConvexHull2D::isInside
		bool isInside(const rw::math::Vector2D<>& vertex);

		//! @copydoc ConvexHull2D::getMinDist
		double getMinDist(const rw::math::Vector2D<>& vertex);

		/**
		 * @brief returns a contour representation of the convex hull
		 */
		std::vector<rw::math::Vector2D<> >* toContour();

	public:
		typedef std::pair<int,int> EdgeIdx;

	private:
		int search(const EdgeIdx& edge);

	private:
		// vertices on the hull
		std::vector<rw::math::Vector3D<> > _vertices;
		// edges between vertices
		std::set<EdgeIdx> _edgeSet;
		std::stack<std::pair<EdgeIdx,int> > _edgeStack;
		// triangles composed of edges
	};

	//! @}
}
}
#endif /* GIFTWRAPHULL_HPP_ */
