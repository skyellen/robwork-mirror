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

#ifndef RW_SENSOR_SCANNER2DMODEL_HPP
#define RW_SENSOR_SCANNER2DMODEL_HPP

/**
 * @file Scanner2DModel.hpp
 */

#include "SensorModel.hpp"
#include <rw/geometry/PointCloud.hpp>

namespace rw {
namespace sensor {

/** @addtogroup sensor */
/* @{ */

/**
 * @brief The Scanner2DModel encapsulate the basic model of a
 * 2 dimensional range scanning device such as SICK or Hokyuo laser
 * range scanners.
 *
 *  The model supports any range scanner that measures distance in
 * an arc around the origin of the sensor. The scanner scans in the z-x plane
 * with z-axis being the 0 angle measurement.
 *
 * TODO: enable the selection of internal format, either pointcloud (large) or
 * range-array (compact).
 *
 */
class Scanner2DModel: public SensorModel
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<Scanner2DModel> Ptr;

    /**
     * @brief constructor
     * @param name [in] name of scanner sensor
     * @param angularRangeInRad [in] angular range in rad, with middle scan
     * point pointin along z-axis
     * @brief maxDataPoints [in] the number of scan points
     * @brief frame [in] the sensor frame
     */
    Scanner2DModel(const std::string& name, double angularRangeInRad, int maxDataPoints, rw::kinematics::Frame* frame );

    /**
     * @brief Destructor. Closes scanner connection if not already closed.
     */
    virtual ~Scanner2DModel();

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

    /**
     * @brief Returns the min and max angular range of the scanner, where
     * the angles represent the beginning and end of scanning in the z-x plane.
     * Hence, angles represent rotation of z-axis around the y-axis. Normally range would
     * be something like -170 to 170 degree for a Hokyo or Sick scanner
     *
     * @return Angular range in radians
     */
    std::pair<double,double> getAngularRange() const { return _angleRange;}

    /**
     * @brief Returns the number of scan points
     */
    size_t getMeasurementCount() const{ return _width;}

    /**
     * @brief get the min an max range in meters that is scannable by the 2D scanner
     * @return range in meters
     */
    std::pair<double,double> getDistanceRange() const{ return _distRange;}

    /**
     * @brief set distance range
     * @param range
     */
    void setDistanceRange(const std::pair<double,double>& range) { _distRange = range;}

    /**
     * @brief set distance range
     * @param range
     */
    void setDistanceRange(double min, double max ) { _distRange = std::make_pair(min,max);}

protected:
    //! cache to allow storing state information
    class Scanner2DModelCache: public rw::kinematics::StateCache {
	public:
		typedef rw::common::Ptr<Scanner2DModelCache> Ptr;
		rw::geometry::PointCloud _cloud;

		//! constructor
		Scanner2DModelCache(int width):
			_cloud(width,1)
		{
		};
		//! @copydoc rw::kinematics::StateCache::size
		size_t size() const{ return _cloud.size()*sizeof(rw::geometry::PointCloud::point_type); };
		//! @copydoc rw::kinematics::StateCache::clone
		virtual rw::common::Ptr<StateCache> clone() const{
			Scanner2DModelCache::Ptr cache = rw::common::ownedPtr( new Scanner2DModelCache(*this) );
			return cache;
		};
	};

    rw::kinematics::StatelessData<int> _sstate;
    size_t _width;
    std::pair<double,double> _angleRange, _distRange;

};
/*@}*/

}
}

#endif /*RW_SENSOR_SCANNER2D_HPP_*/
