/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_GEOMETRY_ANALYTIC_BREP_HPP_
#define RW_GEOMETRY_ANALYTIC_BREP_HPP_

/**
 * @file BREP.hpp
 *
 * \copydoc rw::geometry::BREP
 */

#include <rw/common/Ptr.hpp>
#include <rw/geometry/GeometryData.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Vector3D.hpp>

#include <set>

namespace rw {
namespace geometry {

class Curve;
class Face;
class Shell;
class Surface;
class TriMesh;

//! @addtogroup geometry

//! @{
/**
 * @brief Boundary representation (or B-Rep) of a geometric shape, using a collection of connected surfaces, edges and vertices.
 *
 * In the Shell representation, the geometric shape is formed as a collection of disconnected faces.
 * The BREP representation adds more information about the topology, as surface and curve elements are connected.
 * For a certain surface, curve or vertex, it is possible to find information about the directly connected neighbouring
 * surfaces, edges and vertices. From a BREP it is also possible to retrieve a Shell representation, but in the Shell
 * representation information about connectedness is lost.
 *
 * The half-edge data structure is used internally to store the topological information about the faces, edges and
 * vertices, and how they are connected. Subtypes of BREP implements the concrete Surface and Curve geometries that
 * can be attached to the faces and edges.
 *
 * In general, the procedure for forming a BREP is the following:
 *
 * 1. Add all the needed vertices by using the addVertex function.
 * Each vertex is given an index starting, increasing from zero.
 *
 * 2. Add the edges. An edge is added between two vertices using their vertex indexes.
 * Addition of edges is documented under the specific BREP implementation (depending on the type of Curve that is expected).
 * The edge is in general added with a Curve and two vertex indices. Notice that the curve must have limits, such that
 * it starts in the first vertex and ends in the second vertex (the curve has a direction).
 * Each edge is given an index, increasing from zero.
 *
 * 3. Use the makeLoop function to form loops, based on the edges just added. The makeLoop takes an arbitrary
 * number of edge indices. Notice that makeLoop expects the edge indexing to start at 1, and supports negative indices
 * to indicate opposite direction of the edge. To form a loop, a list of these indexes is given, to form a counter
 * clockwise loop of edges.
 * Each loop is given a loop index, increasing from zero.
 *
 * 4. Attach a Surface to each loop. Again, the addition of surfaces is documented under the specific BREP implementation
 * (depending on the type of Surface that is expected).
 * Each surface is given an increasing surface index, starting from zero. Notice that this index is not
 * necessarily the same as the loop index.
 */
class BREP: public rw::geometry::GeometryData {
public:
    //! @brief Smart pointer type to BREP
    typedef rw::common::Ptr<BREP> Ptr;

    //! @brief Smart pointer type to const BREP
    typedef rw::common::Ptr<const BREP> CPtr;

	//! @brief Destructor.
	virtual ~BREP();

	//! @copydoc GeometryData::getType
	virtual GeometryType getType() const = 0;

	/**
	 * @brief Create a TriMesh representation from this boundary representation.
	 *
	 * This function relies on the resolution set with setMeshResolution.
	 * The resolution is passed on to Curve::discretizeAdaptive and Surface::setDiscretizationResolution.
	 *
	 * @param forceCopy [in] generate a new copy, or use a cached TriMesh.
	 * @return a new TriMesh if \b forceCopy is true, or a shared cached TriMesh if \b forceCopy is false.
	 */
	virtual rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);

	//! @copydoc GeometryData::isConvex
	virtual bool isConvex();

	//! @copydoc GeometryData::isConvex
	virtual bool isConvex() const;

	// Pure virtual interface
	/**
	 * @brief Get surface.
	 * @param surfaceIndex [in] the index of the surface. Should be less than the number returned by size().
	 * @return a reference to the surface.
	 */
	virtual const Surface& getSurface(std::size_t surfaceIndex) const = 0;

	/**
	 * @brief Get curve.
	 * @param curveIndex [in] index of the curve. Should be less than the number returned by edgeCount().
	 * @return a reference to the curve.
	 */
	virtual const Curve& getCurve(std::size_t curveIndex) const = 0;

	/**
	 * @brief Scale the object.
	 * @param factor [in] the factor to scale with.
	 */
	virtual void scale(double factor) = 0;

	// Functions that should be overwritten by more concrete functions in subclasses

	/**
	 * @brief Make a deep copy of the BREP.
	 * @return a copy of the BREP.
	 */
	BREP::Ptr clone() const { return doClone(); }

	/**
	 * @brief Get a Shell representation as a proxy to the BREP.
	 * @return smart pointer to a Shell proxy object.
	 */
	inline rw::common::Ptr<const Shell> shellProxy() const { return doShellProxyBREP(); }

	/**
	 * @brief Get the curves in a given loop.
	 *
	 * The curves will be traversed in an ordered way, and curves will have a direction that leads to the next curve.
	 *
	 * @param loopIdx [in] the loop index.
	 * @return an ordered vector of curves.
	 */
	std::vector<rw::common::Ptr<Curve> > getCurves(std::size_t loopIdx) const;

	//! @brief Convenience type for a set of curves in a BREP.
	class CommonCurveSet {
	public:
	    //! @brief Smart pointer type to CommonCurveSet
	    typedef rw::common::Ptr<const CommonCurveSet> CPtr;

	    //! @brief Constructor.
		CommonCurveSet() {}

		//! @brief Destructor.
		virtual ~CommonCurveSet() {}

		/**
		 * @brief Get the number of curves in the set.
		 * @return the number of curves.
		 */
		virtual std::size_t size() const = 0;

		/**
		 * @brief Get a curve in the set.
		 * @param index [in] the curve index, which should be less than size().
		 * @return a reference to the curve data.
		 */
		virtual const Curve& curve(std::size_t index) const = 0;

		/**
		 * @brief Get one of the neighbour surfaces to the curve.
		 * @param index [in] the curve index, which should be less than size().
		 * @return a reference to the surface.
		 */
		virtual const Surface& surfaceLeft(std::size_t index) const = 0;

		/**
		 * @brief Get the other neighbour surfaces to the curve.
		 * @param index [in] the curve index, which should be less than size().
		 * @return a reference to the surface.
		 */
		virtual const Surface& surfaceRight(std::size_t index) const = 0;
	};

	/**
	 * @brief Get a set of common curves between a set of faces.
	 * @param faces [in] loop indexes for the faces to consider.
	 * @return set of common curves as a CommonCurveSet.
	 */
	CommonCurveSet::CPtr getCommonCurves(const std::set<std::size_t>& faces) const;

	// Functions common to all BREPs
	/**
	 * @brief The number of faces.
	 *
	 * The number of faces is the number of loops, where a surface has been attached.
	 *
	 * @return the number of faces.
	 */
	std::size_t faceCount() const;

	/**
	 * @brief The number of loops.
	 * @return the number of loops.
	 */
	std::size_t loopCount() const { return _faces.size(); }

	/**
	 * @brief The number of edges.
	 * @return the number of edges.
	 */
	std::size_t edgeCount() const { return _edges.size(); }

	/**
	 * @brief The number of vertices.
	 * @return the number of vertices.
	 */
	std::size_t verticeCount() const { return _vertices.size(); }

	/**
	 * @brief Get vertex.
	 * @param vertexIndex [in] index of the vertex. Should be less than the number returned by vertices().
	 * @return reference to the vertex.
	 */
	const rw::math::Vector3D<>& getVertex(std::size_t vertexIndex) const { return _vertices[vertexIndex]->point; }

	/**
	 * @brief Get the vertices in a given loop.
	 *
	 * The curves will be traversed in an ordered way around the loop.
	 *
	 * @param loopIdx [in] the loop index.
	 * @return a collection of vertices.
	 */
	std::vector<rw::math::Vector3D<> > getVertices(std::size_t loopIdx) const;

	/**
	 * @brief Check if a certain loop has a surface set.
	 * @param loop [in] index of the loop, which should be less than loopCount().
	 * @return true if a surface is set, false otherwise.
	 */
	bool hasSurfaceSet(std::size_t loop) { return _faces[loop]->surfaceSet; }

	/**
	 * @brief Get the surface index of a loop.
	 * @param loop [in] the loop index.
	 * @return the corresponding surface index.
	 */
	std::size_t getSurfaceIndex(std::size_t loop);

	/**
	 * @brief Create Oriented Bounding Box with certain principal directions.
	 * @param R [in] the directions for the bounding box.
	 * @return an OBB around the BREP.
	 */
	OBB<> obb(const rw::math::Rotation3D<>& R);

	/**
	 * @brief Create Oriented Bounding Box where the directions are estimated.
	 *
	 * This method is more expensive than obb(const rw::math::Rotation3D<>&),
	 * because a TriMesh is formed to estimated the principal directions of the OBB.
	 *
	 * @return an OBB around the BREP.
	 */
	OBB<> obb();

	/**
	 * @brief Add a vertex to the BREP.
	 * @param point [in] the vertex to add.
	 */
	void addVertex(const rw::math::Vector3D<>& point);

	/**
	 * @brief Create a loop containing a single edge (typically for circles and ellipses and similar).
	 *
	 * The half-edge structure requires that an edge must start and end in a vertex. Sometimes
	 * it is possible to have an edge without any vertices. This is, for example, the case for a
	 * circular or elliptic cylinder, where there will be two circular or elliptic edges.
	 * It is necessary to place one vertex on the circle or ellipse that can act as both the start
	 * and end vertex for the curve.
	 *
	 * @param singleEdgeId [in] id of the edge to create loop for. 1-indexing is expected, with a sign that indicates the edge direction.
	 */
	void makeLoop(int singleEdgeId);

	/**
	 * @brief Create a loop containing two edges.
	 * @param first [in] id of the first edge. 1-indexing is expected, with a sign that indicates the edge direction.
	 * @param second [in] id of the following edge. 1-indexing is expected, with a sign that indicates the edge direction.
	 * @return the first index, \b first.
	 */
	template<typename T>
	int makeLoop(T first, T second) {
		setEdgeOrder(static_cast<int>(first), static_cast<int>(second));
		return static_cast<int>(first);
	}

	/**
	 * @brief Create a loop with a variable number of edges.
	 * @param first [in] id of the first edge. 1-indexing is expected, with a sign that indicates the edge direction.
	 * @param args [in] id of the following edges. Any number of arguments can be given.
	 * @return the first index, \b first.
	 */
	template<typename T, typename... Args>
	int makeLoop(T first, Args... args) {
		setEdgeOrder(static_cast<int>(first), makeLoop(args...));
		return static_cast<int>(first);
	}

	/**
	 * @brief Create Oriented Bounding Box for a face.
	 * @param faceIndex [in] the face index, which should be less than loopCount().
	 * @return OBB for the given face.
	 */
	OBB<> faceOBB(std::size_t faceIndex);

	/**
	 * @brief Create Oriented Bounding Rectangle for an edge.
	 * @param edge [in] the edge index, which should be less than edgeCount().
	 * @return OBB for the given edge (with third half-length set to zero).
	 */
	OBB<> edgeOBR(std::size_t edge) const;

	/**
	 * @brief Find the extent of the surface along a specific direction.
	 *
	 * If the surface has no lower bound, the value -%std::numeric_limits\<double\>::%max() can be returned
	 * to indicate that the surface has unbounded minimum value in the given \b direction.
	 *
	 * If the surface has no upper bound, the value %std::numeric_limits\<double\>::%max() can be returned
	 * to indicate that the surface has unbounded maximum value in the given \b direction.
	 *
	 * @param faceIndex [in] the face index, which should be less than loopCount().
	 * @param dir [in] a normalized direction vector.
	 * @return the minimum and maximum values along the given direction.
	 */
	std::pair<double,double> faceExtremums(std::size_t faceIndex, const rw::math::Vector3D<>& dir) const;

	/**
	 * @brief Construct a Triangle Mesh for a face.
	 * @param faceIndex [in] the face index, which should be less than loopCount().
	 * @return a triangle mesh.
	 */
	rw::common::Ptr<TriMesh> faceTriMesh(std::size_t faceIndex);

	/**
	 * @brief Set the resolution used for discretization in the getTriMesh and faceTriMesh functions.
	 *
	 * The meaning of this parameter depends on the type of surface.
	 *
	 * @param resolution [in] the resolution parameter.
	 */
	void setMeshResolution(double resolution = 10) { _resolution = resolution; }

	//! @brief Print the structure of the BREP for debugging purposes.
	virtual void print();

protected:
	struct HalfEdge;

	/**
	 * @brief Vertex for the half-edge structure.
	 *
	 * In the half-edge structure, a vertex is a points that has a pointer to a neighbour half-edge.
	 */
	struct Vertex {
		/**
		 * @brief Construct new vertex in the given \b point.
		 * @param point [in] the points to construct vertex for.
		 */
		Vertex(const rw::math::Vector3D<>& point): point(point), nextEdge(NULL) {}

		//! @brief Destructor.
		~Vertex() {}

		//! @brief Vertex point.
		rw::math::Vector3D<> point;
		//! @brief Pointer to the next half-edge.
		const HalfEdge* nextEdge;
	};

	/**
	 * @brief Face concept in the half-edge structure.
	 *
	 * A Face has a surface and a pointer to on of the half-edges on the loop sourounding it.
	 */
	struct Face {
		//! @brief Constructor.
		Face(): surfaceSet(false), surfaceIndex(0), edge(NULL) {}

		//! @brief Destructor.
		~Face() {}

		//! @brief Variable to indicate if a surface is assigned to the Face. Otherwise it is considered an empty loop.
		bool surfaceSet;
		//! @brief Set the index to the surface data.
		std::size_t surfaceIndex;
		//! @brief Pointer to one of the half-edges on the loop sourounding the face.
		const HalfEdge* edge;
	};

	/**
	 * @brief Half-edge structure.
	 *
	 * Each edge in the BREP is represented as a pair of half-edges.
	 * A half-edge has pointers to the previous and next vertices, previous and next edges, and a face.
	 */
	struct HalfEdge {
		/**
		 * @brief Constructor.
		 * @param curveIndex [in] the curve index (a pair of half-edges will point to the same curve).
		 */
		HalfEdge(std::size_t curveIndex): curveIndex(curveIndex), previousVertex(NULL), nextVertex(NULL), face(NULL), nextEdge(NULL), previousEdge(NULL), oppositeEdge(NULL), reversed(false) {}

		//! @brief Destructor.
		~HalfEdge() {}

		//! @brief Index of the geometric curve data.
		const std::size_t curveIndex;
		//! @brief Pointer to the vertex at beginning of curve (previous and next might be the same vertex).
		const Vertex* previousVertex;
		//! @brief Pointer to the vertex at end of curve (previous and next might be the same vertex).
		const Vertex* nextVertex;
		//! @brief Pointer to the face (all half-edges forming a loop will point to the same face).
		const Face* face;
		//! @brief Pointer to the next edge in a loop (can be a pointer to itself).
		HalfEdge* nextEdge;
		//! @brief Pointer to the previous edge in a loop (can be a pointer to itself).
		HalfEdge* previousEdge;
		//! @brief Pointer to the opposite edge (part of the loop of an adjacent face).
		HalfEdge* oppositeEdge;
		//! @brief True if the curve is in the reverse direction. As a pair of half-edges points to the same curve data, one of the half-edges will have to be reversed.
		bool reversed;
	};
protected:
    //! @brief Constructor.
	BREP();
	/**
	 * @brief Copy the topology of this BREP to another \b brep.
	 * @param brep [in/out] the other brep to copy data to.
	 */
	void copyTopologyTo(BREP::Ptr brep) const;

	/**
	 * @brief Add a edge (will insert a pair of half-edges).
	 *
	 * Notice that a curve has direction, so it should start in \b vertex1 and end in \b vertex2.
	 *
	 * @param curveIndex [in] index of the curve data for the edge.
	 * @param vertex1 [in] the first vertex.
	 * @param vertex2 [in] the end vertex.
	 */
	void addBREPEdge(std::size_t curveIndex, std::size_t vertex1, std::size_t vertex2);

	/**
	 * @brief Attach a surface to a loop.
	 * @param surfaceIndex [in] index of the surface data for the loop.
	 * @param loop [in] the loop index.
	 */
	void setBREPFace(std::size_t surfaceIndex, std::size_t loop);

private:
	class CommonCurveSetGeneric;
	virtual rw::common::Ptr<const Shell> doShellProxyBREP() const;
	virtual BREP::Ptr doClone() const = 0;

	void setEdgeOrder(int before, int after);
	void setHalfEdgeOrder(HalfEdge* before, HalfEdge* after);
	HalfEdge* freeHalfEdge(const Vertex& endVertex, const HalfEdge* from = NULL, const HalfEdge* to = NULL) const;

protected:
	// Topology
	//! @brief The vertices in the BREP.
	std::vector<Vertex*> _vertices;
	//! @brief The half-edges in the BREP.
	std::vector<std::pair<HalfEdge*, HalfEdge*> > _edges;
	//! @brief The faces in the BREP. A face is added for each loop constructed.
	std::vector<Face*> _faces;

	// Resolution
	//! @brief Resolution used for discretization functions.
	double _resolution;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_BREP_HPP_ */
