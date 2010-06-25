/*
 * GraspTable.hpp
 *
 *  Created on: 22-09-2009
 *      Author: jimali
 */

#ifndef GRASPTABLE_HPP_
#define GRASPTABLE_HPP_

#include <rw/math.hpp>
#include <rw/common.hpp>
#include "Grasp3D.hpp"
#include <rw/sensor/TactileArray.hpp>



namespace rw {
namespace graspplanning {

	/**
	 * @brief A table of grasp configurations that has been generated using a robot hand,
	 * a number of preshapes, and some grasp policy.
	 *
	 *
	 *
	 *
	 */
	class GraspTable {
	public:
		// this should increase each time the file format is changed
		static const int GTABLE_VERSION = 0x00001;

		struct GraspData {
			GraspData(): hp(0,0,0,0,0,0), op(0,0,0,0,0,0)
			{}
			rw::math::Vector3D<> approach; // approach relative to object
			rw::math::Q cq; // contact configuration
			rw::math::Q pq; // preshape configuration
			rw::math::Pose6D<> hp; // hand pose
			rw::math::Pose6D<> op; // object pose
			Grasp3D grasp;
			rw::math::Q quality;
			std::vector<rw::sensor::TactileArray::ValueMatrix> _tactiledata;
			std::vector<std::vector<rw::sensor::Contact3D> > tactileContacts;
		};

	public:
		GraspTable(const std::string& handName, const std::string& objectId);

		virtual ~GraspTable(){};

		void setCalibForceIndex(int idx){_calibForceIndex=idx;};

		void addGrasp(GraspData& data);

		size_t size(){ return _graspData.size();};

		static GraspTable* load(const std::string& filename);

		void save(const std::string& filename);

		std::vector<GraspData>& getData(){return _graspData;};

		const std::string& getHandName(){return _handName;};

		const std::string& getObjectName(){return _objectId;};

		int nrTactileArrayGrasp();

		std::pair<int,int> getTactileArrayDim(int i);

		bool hasCalibForce();

		int getCalibForceIndex();

		/**
		 *
		 * @return
		 */
		//std::vector<GraspData> findGrasps(GraspValidateFilter* filter);
		//std::vector<GraspData> findGrasps(rw::math::Q& minQ, GraspValidateFilter* filter = NULL);
		//std::vector<GraspData> findGrasps(const rw::math::Vector3D<>& approach, double maxAngle,GraspValidateFilter* filter = NULL);
		//std::vector<GraspData> findGrasps(rw::math::Q& minQ, const rw::math::Vector3D<>& approach, double maxAngle,GraspValidateFilter* filter = NULL);

	private:
		rw::common::PropertyMap _properties;
		int _calibForceIndex;
		std::vector<GraspData> _graspData;

		std::string _handName;
		std::string _objectId;
	};

}
}

#endif /* GRASPTABLE_HPP_ */
