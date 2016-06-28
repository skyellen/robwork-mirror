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


#ifndef RW_GRASPPLANNING_GRASPTABLE_HPP_
#define RW_GRASPPLANNING_GRASPTABLE_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Pose6D.hpp>
#include <rw/common/PropertyMap.hpp>
#include "Grasp3D.hpp"
#include <rw/sensor/TactileArray.hpp>


namespace rw {
namespace graspplanning {

	/**
	 * @brief A table of grasp configurations that has been generated using a robot hand,
	 * a number of preshapes, and some grasp policy.
	 *
	 */
	class GraspTable {
	public:
	    typedef rw::common::Ptr<GraspTable> Ptr;

		//! @brief this version increase each time the file format is changed
		static const unsigned int GTABLE_VERSION = 0x00001;

		/**
		 * @brief data for describing a single grasp
		 */
		struct GraspData {
		    //! constructor
			GraspData(): hp(0,0,0,0,0,0), op(0,0,0,0,0,0){}
			//! approach relative to object
			rw::math::Vector3D<> approach;
			//! contact configuration
			rw::math::Q cq;
			//! preshape configuration
			rw::math::Q pq;
			//! hand pose
			rw::math::Pose6D<> hp;
			//! object pose
			rw::math::Pose6D<> op;
			//! grasp information
			Grasp3D grasp;
			//! quality or value list
			rw::math::Q quality;
			//! tactile array data
			std::vector<rw::sensor::TactileArray::ValueMatrix> _tactiledata;
			//! tactile contacts
			std::vector<std::vector<rw::sensor::Contact3D> > tactileContacts;
		};

	public:
		/**
		 * @brief constructor
		 * @param handName [in] name of robot hand
		 * @param objectId [in] name of object that is being grasped
		 */
		GraspTable(const std::string& handName, const std::string& objectId);

		//! @brief destructor
		virtual ~GraspTable(){};

        /**
         * @brief gets the index of the calibration force if its used. The calibration force is stored in
         * the 'quality' list in the GraspData objects.
         * @return index of calibration force if used, -1 otherwise
         */
        int getCalibForceIndex();

        /**
         * @brief set the index of the calibration force
         * @param idx [in] calibration force index
         */
		void setCalibForceIndex(int idx){_calibForceIndex=idx;};

		/**
		 * @brief add a grasp to this GraspTable
		 * @param data [in] Grasp data
		 */
		void addGrasp(GraspData& data);

		/**
		 * @brief get the nr of grasps in this GraspTable
		 * @return nr of grasps in this GraspTable
		 */
		size_t size(){ return _graspData.size();};

        /**
         * @brief get all grasp data
         * @return vector of grasps
         */
		std::vector<GraspData>& getData(){return _graspData;};

		/**
		 * @brief get name of hand
		 * @return name of hand
		 */
		const std::string& getHandName(){return _handName;};

        /**
         * @brief get name of object
         * @return name of object
         */
		const std::string& getObjectName(){return _objectId;};

		/**
		 * @brief get the number of tactile arrays on this hand
		 * @return
		 */
		int nrTactileArrayGrasp();

		/**
		 * @brief get the dimensions of the i'th tactile array
		 * @param i [in] the tactile array id
		 * @return
		 */
		std::pair<int,int> getTactileArrayDim(int i);

		/**
		 * @brief check if this table has calib force data
		 * @return true if this table has calib force data (equal to getCalibForceIndex()>=0), false otherwise
		 */
		bool hasCalibForce();


        /**
         * @brief load a grasp table from file
         * @param filename [in] name of file
         * @return GraspTable
         */
        static rw::common::Ptr<GraspTable> load(const std::string& filename);

        /**
         * @brief save this grasp table to file \b filename
         * @param filename [in] name of file
         */
        void save(const std::string& filename);

		/**
		 *
		 * @return
		 */
		//std::vector<GraspData> findGrasps(GraspValidateFilter* filter);
		//std::vector<GraspData> findGrasps(rw::math::Q& minQ, GraspValidateFilter* filter = NULL);
		//std::vector<GraspData> findGrasps(const rw::math::Vector3D<>& approach, double maxAngle,GraspValidateFilter* filter = NULL);
		//std::vector<GraspData> findGrasps(rw::math::Q& minQ, const rw::math::Vector3D<>& approach, double maxAngle,GraspValidateFilter* filter = NULL);

	private:
        std::string _handName;
        std::string _objectId;
        int _calibForceIndex;

		rw::common::PropertyMap _properties;
		std::vector<GraspData> _graspData;

	};

}
}

#endif /* GRASPTABLE_HPP_ */
