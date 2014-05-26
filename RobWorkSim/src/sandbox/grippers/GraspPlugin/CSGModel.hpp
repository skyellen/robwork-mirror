/**
 * @file CSGModel.hpp
 * 
 * @brief Contains implementation of CSGModel class being the frontend to csgjs library.
 * 
 * @author Adam Wolniakowski
 */
 
#pragma once



#include <iostream>
#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/geometry/TriMesh.hpp>
#define CSGJS_HEADER_ONLY
#include "csgjs.cpp"



namespace rw {
	namespace csg {
		/**
		 * @brief A frontend class for doing CSG.
		 */
		class CSGModel
		{
			public:
				//! @brief Smart pointer to this type of class
				typedef rw::common::Ptr<CSGModel> Ptr;
		
				// constructors
				/** @brief Constructor. */
				CSGModel();
				
				/** @brief Destructor. */
				virtual ~CSGModel() {}
				
				/** @brief Copying constructor. */
				CSGModel(const CSGModel& csgmodel);
				
				/** @brief Constructs CSGModel from TriMesh. */
				CSGModel(const geometry::TriMesh& trimesh);
				
				/**
				 * @brief Constructs cube primitive.
				 * @param x, y, z length, width and depth
				 */
				static CSGModel& makeBox(double x, double y, double z);
				
				/**
				 * @brief Constructs cylinder primitive.
				 * @param r, h radius, height
				 */
				static CSGModel& makeCylinder(double r, double h);
				
				/**
				 * @brief Constructs sphere primitive.
				 * @param r radius
				 */
				static CSGModel& makeSphere(double r);
				
				/**
				 * @brief Constructs plane. \n
				 * Creates fake half-space, which is actually a large cube.
				 * Useful for chamfering.
				 * @param point, normal point and normal
				 */
				static CSGModel& makePlane(math::Vector3D<> point=math::Vector3D<>(), math::Vector3D<> normal=math::Vector3D<>::z());
				
				/**
				 * @brief Constructs wedge. \n
				 * An intersection of two planes useful for making cutouts.
				 * The wedge tip is located at origin and the edge runs along z axis.
				 * The wedge axis runs along x axis
				 */
				static CSGModel& makeWedge(double angle);
				
				//... need more primitives?
				// - mostly for convenience
				
				// manipulation
				/** @brief Translates the csgmodel. */
				CSGModel& translate(double x, double y, double z);
				
				/** @brief Returns translated copy. */
				CSGModel& translated(double x, double y, double z) const;
				
				/** @brief Rotates the csgmodel. */
				CSGModel& rotate(double r, double p, double y);
				
				/** @brief Returns rotated copy. */
				CSGModel& rotated(double r, double p, double y) const;
				
				/** @brief Applies RobWork transformation. */
				CSGModel& transform(const math::Transform3D<>& T);
				
				/** @brief Returns copy after transformation. */
				CSGModel& transformed(const math::Transform3D<>& T) const;
				
				// operators
				CSGModel& operator+=(const CSGModel& csgmodel);
				
				CSGModel& operator+(const CSGModel& csgmodel);
				
				CSGModel& operator-=(const CSGModel& csgmodel);
				
				CSGModel& operator-(const CSGModel& csgmodel);
				
				CSGModel& operator*=(const CSGModel& csgmodel);
				
				CSGModel& operator*(const CSGModel& csgmodel);
				
				CSGModel& operator/=(const CSGModel& csgmodel);
				
				CSGModel& operator/(const CSGModel& csgmodel);
				
				// conversion
				/** @brief Returns RobWork geometry representation. */
				geometry::TriMesh::Ptr getTriMesh();
				
				// display and saving
				/** @brief Print the CSGModel (for debug mostly). */
				void print() const;
				
				/** @brief Saves the CSGModel in Stl format. */
				void saveToStl(const std::string& filename);
				
				friend std::ostream& operator<<(std::ostream& stream, const CSGModel& csgmodel);
				
			private:
				// private methods
				/** @brief Converts internal csgjs geometry representation to TriMesh. */
				void _convertToTriMesh();
				
				// data
				bool _needsConversion; // is it neccessary to convert to TriMesh?
				
				csgjs_model _model; // csgjs library geometry representation
				geometry::TriMesh::Ptr _mesh; // RobWork geometry representation
		};
		
		
		
		std::ostream& operator<<(std::ostream& stream, const CSGModel& csgmodel);
	} // csg
} //rw
