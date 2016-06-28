/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#ifndef RW_SENSOR_TACTILEARRAY_HPP
#define RW_SENSOR_TACTILEARRAY_HPP

#include "Sensor.hpp"

#include "TactileArrayModel.hpp"
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
    //! smart pointer type
    typedef rw::common::Ptr<TactileArray> Ptr;

    //! type of tactile array readings
    typedef TactileArrayModel::ValueMatrix ValueMatrix;
    //! type of vertices describing geometry of sensor
    typedef TactileArrayModel::VertexMatrix VertexMatrix;

    /**
     * @brief constructor
     * @param name [in] name of sensor
     */
    TactileArray(const std::string& name):
        Sensor(name)
    {
    }

    /**
     * @brief destructor
     */
    virtual ~TactileArray(){}

    rw::kinematics::Frame* getFrame() const { return getSensorModel()->getFrame(); }

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

    virtual int getWidth() const = 0;

    virtual int getHeight() const = 0;


    //************** the statefull interface (dynamic states) ***************

    /**
     * @brief acquires force data from the tactile cells
     */
    virtual void acquire() = 0;

    /**
     * @brief returns the pressure on each texel of the TactileArray in
     * the unit N/m^2.
     * @return matrix of texel pressure values
     */
    virtual const TactileArrayModel::ValueMatrix& getTexelData( ) const = 0;

    //virtual boost::numeric::ublas::matrix<bool> getMatrixMask() = 0;

};

}
}

#endif /*RW_SENSOR_TACTILEARRAY_HPP*/
