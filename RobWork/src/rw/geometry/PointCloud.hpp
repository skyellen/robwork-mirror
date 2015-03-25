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

#ifndef RW_GEOMETRY_POINTCLOUD_HPP_
#define RW_GEOMETRY_POINTCLOUD_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <vector>
#include "GeometryData.hpp"

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

	/**
	 * @brief A simple point cloud data structure. Points may be ordered or not. An ordered set is
	 * kept as a single array in row major order and with a width and a height. An unordered array
	 * must have height==1 and width equal to the number of points.
	 */
	class PointCloud: public GeometryData {
	public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<PointCloud> Ptr;
        //! type of point used internally in pointcloud
        typedef rw::math::Vector3D<float> point_type;

        /**
         * @brief constructor
         */
        PointCloud():_width(0),_height(0){};

        /**
         * @brief constructor
         * @param w
         * @param h
         */
        PointCloud(int w, int h):_width(w),_height(h),_data(w*h){};

		/**
		 * @brief destructor
		 */
		virtual ~PointCloud(){};

		//! @copydoc GeometryData::getType
		 GeometryType getType() const{ return GeometryData::PointCloud; };

		/**
		 * @brief gets the number of points in the point cloud.
		 */
		virtual size_t size() const { return _data.size(); };

		bool isOrdered(){ return _height>1; }

	    /**
	     * @brief returns a char pointer to the image data
	     * @return char pointer to the image data
	     */
	    std::vector<rw::math::Vector3D<float> >& getData() { return _data; };

	    /**
	     * @brief returns a char pointer to the image data
	     * @return const char pointer to the image data
	     */
	    const std::vector<rw::math::Vector3D<float> >& getData() const{ return _data; };

	    /**
	     * @brief access points in point cloud.
	     * @param x [in] x coordinate, must be in [0;width[
	     * @param y [in] y coordinate, must be in [0;height[
	     * @return point
	     */
        const rw::math::Vector3D<float>& operator()(int x, int y) const { return _data[y*_width+x]; }

        /**
         * @brief access points in point cloud.
         * @param x [in] x coordinate, must be in [0;width[
         * @param y [in] y coordinate, must be in [0;height[
         * @return point
         */
	    rw::math::Vector3D<float>& operator()(int x, int y){ return _data[y*_width+x]; }

	    /**
	     * @brief width of the point cloud data. If the data is unordered then this
	     * will be equal to the number of points.
	     * @return width of data points
	     */
	    int getWidth() const { return _width; }


        /**
         * @brief
         * @return
         */
        int getHeight() const { return _height; }

	    /**
	     * @brief set width of point cloud. Data elements are accessed as [x+y*width].
	     *
	     * If the current data array cannot contain the elements then it will be resized to
	     * be able to it.
	     * @param w [in] new width
	     * @param h [in] new height
	     */
	    void resize(int w, int h){
	        _width = w;
	        _height=h;
	        if(_width*_height> (int) _data.size())
	            _data.resize(_width*_height);
	    };

		//! @copydoc GeometryData::getTriMesh
		rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);

		//! @copydoc getTriMesh
		rw::common::Ptr<const TriMesh> getTriMesh(bool forceCopy=true) const;

		const rw::math::Transform3D<float>& getDataTransform() const { return _sensorTransform; }

		/**
		 * @brief load point cloud from PCD file
		 * @param filename [in] name of PCD file
		 * @return a point cloud
		 */
		static rw::common::Ptr<PointCloud> loadPCD( const std::string& filename );

		/**
		 * @brief save point cloud in PCD file format (PCL library format)
		 * @param cloud [in] the point cloud to save
		 * @param filename [in] the name of the file to save to
		 * @param t3d [in] the transformation of the point cloud
		 */
        static void savePCD( const PointCloud& cloud,
                                                    const std::string& filename ,
                                                    const rw::math::Transform3D<float>& t3d =
                                                            rw::math::Transform3D<float>::identity());

	private:
		int _width, _height;
		std::vector<rw::math::Vector3D<float> > _data;
		rw::math::Transform3D<float> _sensorTransform;
	};

	//! @}
} // geometry
} // rw



#endif /*TRIMESH_HPP_*/
