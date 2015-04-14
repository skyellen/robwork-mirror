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


#ifndef RW_SENSOR_TACTILEARRAYMODEL_HPP
#define RW_SENSOR_TACTILEARRAYMODEL_HPP

#include "SensorModel.hpp"

#include <boost/multi_array.hpp>

#include <rw/kinematics/State.hpp>

#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>


namespace rw {
namespace sensor {

/**
 * @brief the TactileArrayModel describes tactile sensor consisting of
 * arrays of tactile cells that can be placed on a defined shape. The shape is described
 * with a matrix of 3d vertices. Such that tactil (0,0) maps to the quad
 * defined by the four vertices {(0,0),(0,1),(1,1),(1,0)}. Notice that the
 * normal is defined by sequence of the vertices and that the normal defines
 * the direction of tactile sensing.
 */
class TactileArrayModel : public SensorModel {
public:
    //! smart pointer type
    typedef rw::common::Ptr<TactileArrayModel> Ptr;

    //! type of tactile array readings
    typedef Eigen::MatrixXf ValueMatrix;
    //! type of vertices describing geometry of sensor
    typedef boost::multi_array<rw::math::Vector3D<>, 2> VertexMatrix;

    /**
     * @brief constructor
     * @param name [in] name of sensor
     * @param sensorframe [in] the frame to which the sensor is attached
     * @param fThmap [in] transformation from sensor frame to the heightmap definition
     * @param heightMap [in] a height map defining the height of each corner in the tactile array
     * @param cell_width [in] width of cell
     * @param cell_height [in] height of cell
     */
    TactileArrayModel(const std::string& name,
    		rw::kinematics::Frame* sensorframe,
			const rw::math::Transform3D<>& fThmap,
			const ValueMatrix& heightMap,
			double cell_width, double cell_height);

    /**
     * @brief destructor
     */
    virtual ~TactileArrayModel();

    /**
     * @brief gets the size of an individual tactile cell with coordinates (x,y)
     * @param x
     * @param y
     * @return the dimensions of the tactile cell in meters
     */
    rw::math::Vector2D<> getTexelSize(int x, int y) const;

    /**
     * @brief get the minimum and maximum pressure capability of any tactile
     * cell in the TactileArray
     * @return min and max pressure in Pa
     */
    std::pair<double,double> getPressureLimit() const;


    /**
     * @brief set pressure limits. should define min max of any tactile cell in array
     * @param min [in] min pressure in Pa
     * @param max [in] max pressure in Pa
     */
    void setPressureLimit(double min, double max);
    void setPressureLimit(std::pair<double,double> range){ setPressureLimit(range.first,range.second);};

    /**
     * @brief gets the 3d geometry of this tactilearray. The vertexes are expressed
     * realtive to the transform.
     * @return
     */
    const VertexMatrix& getVertexGrid() const;

    /**
     * @brief a transformation from the sensor frame to the geometric data of
     * the tactile array.
     * @return
     */
    const rw::math::Transform3D<>& getTransform() const;

    /**
     * @brief a matrix with position of each tactile cell center. The coordinates
     * are described relative to the TactileArray transform (see getTransform())
     * @return a matrix describing the center of each tactile cell.
     */
    const VertexMatrix& getCenters() const;

    /**
     * @brief a matrix of normals that are described relative to each tactile
     * cell center.
     * @return
     */
    const VertexMatrix& getNormals()  const;

    //! get width of tactile array
    int getWidth() const;

    //! get height of tactile array
    int getHeight() const;

    //************** the statefull interface (dynamic states) ***************
    /**
     * @brief returns the pressure on each texel of the TactileArray in
     * the unit Pa (N/m^2).
     * @param state [in] state to get the values from
     * @return matrix of texel pressure values
     */
    ValueMatrix& getTexelData(rw::kinematics::State& state) const;
    const ValueMatrix& getTexelData(const rw::kinematics::State& state) const;

    /**
     * @brief set the pressure on each texel of the TactileArray in
     * the unit Pa (N/m^2).
     * @param data [in] pressure values
     * @param state [in] state to set the values in
     */
    void setTexelData(const ValueMatrix& data, rw::kinematics::State& state) const;

protected:

    //! cache to store state information
    class TactileModelCache: public rw::kinematics::StateCache {
	public:
		typedef rw::common::Ptr<TactileModelCache> Ptr;
		rw::common::Ptr<ValueMatrix> _data;

		//! constructor
		TactileModelCache()
		{
		};

		//! @copydoc
		size_t size() const{
			size_t stmp = 0;
			if(_data!=NULL)
				stmp+=_data->cols()*_data->rows()*sizeof(float);
			return stmp;
		};

		virtual rw::common::Ptr<StateCache> clone() const{
			TactileModelCache::Ptr cache = rw::common::ownedPtr( new TactileModelCache(*this) );
			if(_data!=NULL)
				cache->_data = rw::common::ownedPtr( new ValueMatrix( *_data ));
			return cache;
		};
	};



private:
    rw::math::Transform3D<> _fThmap;
    ValueMatrix _heightMap;
    double _cellWidth, _cellHeight;
    double _minPressure, _maxPressure;

    rw::kinematics::StatelessData<int> _sdata;

    VertexMatrix _vertexGrid, _cellCenters, _cellNormals;

};

}
}

#endif /*RW_SENSOR_TACTILEARRAY_HPP*/
