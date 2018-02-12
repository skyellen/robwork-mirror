/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICBREP_HPP_
#define RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICBREP_HPP_

/**
 * @file QuadraticBREP.hpp
 *
 * \copydoc rw::geometry::QuadraticBREP
 */

#include "QuadraticCurve.hpp"
#include "QuadraticFace.hpp"
#include "QuadraticShell.hpp"
#include "QuadraticSurface.hpp"

#include <rw/geometry/analytic/BREP.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>

#include <set>

namespace rw {
namespace geometry {

class PlainQuadraticShell;

//! @addtogroup geometry

//! @{
/**
 * @brief A boundary representation where the geometry of a face is a QuadraticSurface,
 * and an edge curve is a QuadraticCurve.
 *
 * This implementation adds the geometry related to Quadrics, based on BREP which handles the topology.
 *
 * To build a geometry composed of Quadratic surfaces and curves, follow the following procedure:
 *
 * 1. Add all the needed vertices by using the addVertex function.
 * Each vertex is given an index starting, increasing from zero.
 *
 * 2. Add the edges with the addEdge function. An edge is added between two vertices using their vertex indexes.
 * The edge requires a QuadraticCurve and two vertex indicies. Notice that the curve must have limits, such that
 * it start in the first vertex and ends in the second vertex (the curve has a direction).
 * Each edge is given an index, increasing from zero.
 *
 * 3. Use the makeLoop function to form loops, based on the edges just added. The makeLoop takes an arbitrary
 * number of edge indexes. Notice that makeLoop expects the edge indexing to start at 1, and supports negative indices
 * to indicate direction of the edge. To form a loop, a list of these indexes is given, to form a counter clockwise
 * loop of edges.
 * Each loop is given a loop index, increasing from zero.
 *
 * 4. Attach a QuadraticSurface to a loop, by using the setSurface function.
 * Each surface is given an increasing surface index, starting from zero. Notice that this index is not
 * necessarily the same as the loop index.
 *
 * Together this forms a connected set of surfaces, curves and vertices, that forms a closed geometry.
 * Functions are provided that makes it easy to search for various neighbouring primitives in the boundary representation.
 *
 * It is also possible to retrieve a non-connected shell representation, either by making a PlainQuadraticShell
 * with all information fully copied, or by using a the more lightweight shellProxy which retrieves its information from
 * the underlying QuadraticBREP.
 */
class QuadraticBREP: public BREP {
public:
    //! @brief Smart pointer type to QuadraticBREP
    typedef rw::common::Ptr<QuadraticBREP> Ptr;

    //! @brief Constructor.
	QuadraticBREP();

    //! @brief Destructor.
	virtual ~QuadraticBREP();

	//! @copydoc BREP::getType
	virtual GeometryType getType() const;

	//! @copydoc BREP::getSurface
	virtual const QuadraticSurface& getSurface(std::size_t surfaceIndex) const;

	//! @copydoc BREP::getCurve
	virtual const QuadraticCurve& getCurve(std::size_t curveIndex) const;

	//! @copydoc BREP::scale
	virtual void scale(double factor);

	//! @copydoc BREP::clone
	QuadraticBREP::Ptr clone() const;

	//! @copydoc BREP::shellProxy
	rw::common::Ptr<const QuadraticShell> shellProxy() const;

	/**
	 * @brief Get a QuadraticShell representation by copying geometric information to a concrete PlainQuadraticShell object.
	 * @return smart pointer to a PlainQuadraticShell object.
	 */
	rw::common::Ptr<PlainQuadraticShell> shell() const;

	//! @copydoc BREP::getCurves
	std::vector<rw::common::Ptr<QuadraticCurve> > getCurves(std::size_t loopIdx) const;

	//! @brief Convenience type for a set of curves in a QuadraticBREP.
	class CommonQuadraticCurveSet: public CommonCurveSet {
	public:
	    //! @brief Smart pointer type to CommonQuadraticCurveSet
	    typedef rw::common::Ptr<const CommonQuadraticCurveSet> CPtr;

	    //! @brief Constructor.
		CommonQuadraticCurveSet(): CommonCurveSet() {}

		//! @brief Destructor.
		virtual ~CommonQuadraticCurveSet() {}

		//! @copydoc BREP::CommonCurveSet::size
		virtual std::size_t size() const = 0;

		//! @copydoc BREP::CommonCurveSet::curve
		virtual const QuadraticCurve& curve(std::size_t index) const = 0;

		//! @copydoc BREP::CommonCurveSet::surfaceLeft
		virtual const QuadraticSurface& surfaceLeft(std::size_t index) const = 0;

		//! @copydoc BREP::CommonCurveSet::surfaceRight
		virtual const QuadraticSurface& surfaceRight(std::size_t index) const = 0;
	};

	//! @copydoc BREP::getCommonCurves
	CommonQuadraticCurveSet::CPtr getCommonCurves(const std::set<std::size_t>& faces) const;

	/**
	 * @brief Add a QuadraticCurve to the BREP.
	 *
	 * Notice that the curve has direction. It is expected to have limits such that it starts in vertex \b v1 and end in \b v2.
	 *
	 * @param curve [in] curve to add.
	 * @param v1 [in] the start vertex.
	 * @param v2 [in] the end vertex.
	 */
	void addEdge(const QuadraticCurve& curve, std::size_t v1, std::size_t v2);

	/**
	 * @brief Attach a QuadraticSurface to a face of the BREP.
	 * @param surface [in] surface to add.
	 * @param loop [in] the loop index for the loop to attach surface to.
	 */
	void setFace(const QuadraticSurface& surface, std::size_t loop);


private:
	class CommonQuadraticCurveSetImpl;
	virtual rw::common::Ptr<const Shell> doShellProxyBREP() const;
	virtual BREP::Ptr doClone() const { return clone(); }

	// Geometry
	std::vector<QuadraticCurve::CPtr> _curves;
	std::vector<QuadraticSurface::CPtr> _surfaces;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICBREP_HPP_ */
