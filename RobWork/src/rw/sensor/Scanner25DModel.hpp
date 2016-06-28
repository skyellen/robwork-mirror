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

#ifndef RW_SENSOR_SCANNER25DMODEL_HPP
#define RW_SENSOR_SCANNER25DMODEL_HPP

#include <rw/geometry/PointCloud.hpp>
#include "SensorModel.hpp"
#include <rw/common/Ptr.hpp>

namespace rw {
namespace sensor {

/** @addtogroup sensor */
/* @{ */

/**
 * @brief Model of a 25D (2D with depth information) scanner. The images are
 * essentially point clouds.
 */
class Scanner25DModel: public SensorModel
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<Scanner25DModel> Ptr;

    /**
     * @brief constructor
     * @param frame [in] the frame that the scanner is attached to
     * @param name [in] name of scanner sensor
     */
    Scanner25DModel(const std::string& name, int width, int height, rw::kinematics::Frame* frame );

    /**
     * @brief Destructor. Closes scanner connection if not already closed.
     */
    virtual ~Scanner25DModel();

    /**
     * @brief get handle to point cloud data in state.
     * @param state [in] the state with point cloud data
     */
    rw::geometry::PointCloud& getScan(const rw::kinematics::State& state);

    /**
     * @brief set point cloud data in state
     * @param data [in] point cloud data to set
     * @param state [in] state in which to set the point cloud
     */
    void setScan(const rw::geometry::PointCloud& data, const rw::kinematics::State& state);

    //! width of images taken with 25 sensor
    int getWidth() const { return _width; }

    //! height of images taken with 25 sensor
    int getHeight() const { return _height; }

    //! get the min and maximum depth of this scanner in meters
    std::pair<double,double> getRange() const { return std::make_pair(_rangeMin,_rangeMax); }
    //! set the min and maximum depth of this scanner in meters
    void setRange(double min, double max) { _rangeMin = min; _rangeMax = max;}
    //! set the min and maximum depth of this scanner in meters
    void setRange(const std::pair<double,double>& range) { _rangeMin = range.first; _rangeMax = range.second;}

protected:
    //! cache object for storing relevant state information
    class Scanner25DModelCache: public rw::kinematics::StateCache {
	public:
		typedef rw::common::Ptr<Scanner25DModelCache> Ptr;
		rw::geometry::PointCloud _cloud;
		//! constructor
		Scanner25DModelCache(int width, int height):
			_cloud(width,height)
		{
		};

		//! @copydoc rw::kinematics::StateCache::size
		size_t size() const{ return _cloud.size()*sizeof( rw::geometry::PointCloud::point_type); };

		//! @copydoc rw::kinematics::StateCache::clone
		virtual rw::common::Ptr<StateCache> clone() const{
			Scanner25DModelCache::Ptr cache = rw::common::ownedPtr( new Scanner25DModelCache(*this) );
			return cache;
		};
	};

    rw::kinematics::StatelessData<int> _sstate;
    int _width, _height;
    double _rangeMin,_rangeMax;
};

/*@}*/

}
}

#endif /*RW_SENSOR_SCANNER3D_HPP*/
