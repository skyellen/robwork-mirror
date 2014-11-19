#pragma once

#include <iostream>
#include <string>
#include <rw/geometry/Primitive.hpp>



namespace rw {
    namespace geometry {
/**
 * @brief Parametrized jaw geometry primitive.
 */
class JawPrimitive: public Primitive {
	public:
	// typedefs
		//! @brief Smart pointer
		typedef rw::common::Ptr<JawPrimitive> Ptr;
		
		//! @brief Cutout types
		enum CutoutType {
			Prismatic,
			Cylindrical
		};
		
	// constructors
		/**
		 * @brief constructor (generates basic box jaw shape: length, width, depth)
		 */
		JawPrimitive(double length=0.1, double width=0.025, double depth=0.02) :
			_type(Prismatic),
			_length(length),
			_width(width),
			_depth(depth),
			_chamferDepth(0.0),
			_chamferAngle(0.0),
			_cutPosition(_length/2),
			_cutDepth(0.0),
			_cutAngle(90.0*rw::math::Deg2Rad),
			_cutRadius(0.0),
			_cutTilt(0.0)
		{}

		/**
		 * @brief constructor
		 * @param initQ [in] vector with (length, width, depth, chamfer depth, chamfer angle, cut position, cut depth, cut angle, cut radius, cut tilt)
		 */
		JawPrimitive(const rw::math::Q& initQ);

		//! @brief destructor
		virtual ~JawPrimitive();
		
	// methods
		double getLength() const { return _length; }
		double getWidth() const { return _width; }
		double getDepth() const { return _depth; }
		double getChamferDepth() const { return _chamferDepth; }
		double getChamferAngle() const { return _chamferAngle; }
		double getCutPosition() const { return _cutPosition; }
		double getCutDepth() const { return _cutDepth; }
		double getCutAngle() const { return _cutAngle; }
		double getCutRadius() const { return _cutRadius; }
		CutoutType getCutType() const { return _type; }
		double getCutTilt() const { return _cutTilt; }
		
		void setLength(double length) { _length = length; }
		void setWidth(double width) { _width = width; }
		void setDepth(double depth) { _depth = depth; }
		void setChamferDepth(double depth) { _chamferDepth = depth; }
		void setChamferAngle(double angle) { _chamferAngle = angle; }
		void setCutPosition(double pos) { _cutPosition = pos; }
		void setCutDepth(double depth) { _cutDepth = depth; }
		void setCutAngle(double angle) { _cutAngle = angle; }
		void setCutRadius(double radius) { _cutRadius = radius; }
		void setCutType(CutoutType type) { _type = type; }
		void setCutTilt(double tilt) { _cutTilt = tilt; }
		
		//! @brief Saves parameters to string (for use with XML saver)
		virtual std::string toString() const;

		// inherited from Primitive
		//! @copydoc Primitive::createMesh
		virtual TriMesh::Ptr createMesh(int resolution=0) const;

		//! @copydoc Primitive::getParameters
		virtual rw::math::Q getParameters() const;

		//! @copydoc GeometryData::getType
		GeometryType getType() const { return UserType; }

	protected:
	// data
		CutoutType _type;
	
		double _length, _width, _depth;
		double _chamferDepth, _chamferAngle;
		double _cutPosition, _cutDepth, _cutAngle, _cutRadius, _cutTilt;
};

}} // end namespaces
