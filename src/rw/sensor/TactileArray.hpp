/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_SENSOR_TACTILEARRAY_HPP
#define RW_SENSOR_TACTILEARRAY_HPP

#include "Sensor.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/multi_array.hpp>

#include <rw/kinematics/State.hpp>

#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>


namespace rw {
namespace sensor {

/**
 * @brief
 */
class TactileArray : public Sensor {
public:
    typedef boost::numeric::ublas::matrix<float> ValueMatrix;
    typedef boost::multi_array<rw::math::Vector3D<>, 2> VertexMatrix;

    /**
     * @param name
     * @param frame
     * @return
     */
    TactileArray(const std::string& name, rw::kinematics::Frame* frame):
        Sensor(frame, name)
    {

    }

    /**
     * @brief destructor
     * @return
     */
    virtual ~TactileArray(){}

    /**
     * @brief acquires force data from the tactile cells
     * @param state
     */
    virtual void acquire() = 0;

    /**
     * @brief returns the pressure on each texel of the TactileArray in
     * the unit N/m^2.
     * @return matrix of texel pressure values
     */
    virtual ValueMatrix getTexelData() const = 0;

    /**
     * @brief gets the size of an individual tactile cell with coordinates (x,y)
     * @param x
     * @param y
     * @return the dimensions of the tactile cell in meters
     */
    virtual rw::math::Vector2D<> getTexelSize(int x, int y) const = 0;

    /**
     * @brief get the minimum and maximum pressure capability of any tactile
     * cell in the TactileArray
     * @return
     */
    virtual std::pair<double,double> getPressureLimit() const = 0;

    /**
     * @brief gets the 3d geometry of this tactilearray. The vertexes are expressed
     * realtive to the transform.
     * @return
     */
    virtual const VertexMatrix& getVertexGrid() const = 0;

    /**
     * @brief a transformation from the sensor frame to the geometric data of
     * the tactile array.
     * @return
     */
    virtual const rw::math::Transform3D<>& getTransform() const = 0;

    //virtual boost::numeric::ublas::matrix<bool> getMatrixMask() = 0;
    /**
     * @brief a matrix with position of each tactile cell center. The coordinates
     * are described relative to the TactileArray transform (see getTransform())
     * @return a matrix describing the center of each tactile cell.
     */
    virtual const VertexMatrix& getCenters() const = 0;

    /**
     * @brief a matrix of normals that are described relative to each tactile
     * cell center.
     * @return
     */
    virtual const VertexMatrix& getNormals()  const = 0;

};

}
}

#endif /*RW_SENSOR_TACTILEARRAY_HPP*/
