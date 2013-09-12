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
				//! @brief Smart pointer
				typedef rw::common::Ptr<JawPrimitive> Ptr;
				
                /**
                 * @brief constructor (generates basic box jaw shape: length, width, depth)
                 */
                JawPrimitive(double length=0.1, double width=0.025, double depth=0.02) :
                    _length(length),
                    _width(width),
                    _depth(depth),
                    _chamferDepth(0.0),
                    _chamferAngle(0.0),
                    _cutPosition(_length/2),
                    _cutDepth(0.0),
                    _cutAngle(90.0*rw::math::Deg2Rad)
                {}

                /**
                 * @brief constructor
                 * @param initQ [in] vector with (length, width, depth, chamfer depth, chamfer angle, cut position, cut depth, cut angle)
                 */
                JawPrimitive(const rw::math::Q& initQ);

                //! @brief destructor
                virtual ~JawPrimitive();
                
                double getLength() const { return _length; }
                double getWidth() const { return _width; }
                double getDepth() const { return _depth; }
                double getChamferDepth() const { return _chamferDepth; }
                double getChamferAngle() const { return _chamferAngle; }
                double getCutPosition() const { return _cutPosition; }
                double getCutDepth() const { return _cutDepth; }
                double getCutAngle() const { return _cutAngle; }
                
                void setLength(double length) { _length = length; }
                void setWidth(double width) { _width = width; }
                void setDepth(double depth) { _depth = depth; }
                void setChamferDepth(double depth) { _chamferDepth = depth; }
                void setChamferAngle(double angle) { _chamferAngle = angle; }
                void setCutPosition(double pos) { _cutPosition = pos; }
                void setCutDepth(double depth) { _cutDepth = depth; }
                void setCutAngle(double angle) { _cutAngle = angle; }
                
                //! @brief Saves parameters to string (for use with XML saver)
                std::string toString() const;

                // inherited from Primitive
                //! @copydoc Primitive::createMesh
                TriMesh::Ptr createMesh(int resolution=0) const;

                //! @copydoc Primitive::getParameters
                rw::math::Q getParameters() const;

                //! @copydoc GeometryData::getType
                GeometryType getType() const { return UserType; }

            private:
                double _length, _width, _depth;
                double _chamferDepth, _chamferAngle;
                double _cutPosition, _cutDepth, _cutAngle;
        };
}} // end namespaces
