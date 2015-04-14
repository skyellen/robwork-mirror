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

#ifndef RWSIM_SENSOR_TACTILEARRAYSENSOR_HPP_
#define RWSIM_SENSOR_TACTILEARRAYSENSOR_HPP_

#include <rw/sensor/TactileArray.hpp>
#include <rw/sensor/TactileArrayModel.hpp>
#include <rw/sensor/Contact3D.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <boost/multi_array.hpp>
#include <rw/models/Object.hpp>
#include <rw/geometry.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <vector>
#include <map>

#include "SimulatedTactileSensor.hpp"

namespace rwsim {
namespace sensor {
	//! @addtogroup rwsim_dynamics
	//! @{

	/**
	 * @brief the TactileMatrixSensor class combines a TactileMatrix data
	 * type with the actual shape of a tactile sensor. The shape is described
	 * with a matrix of 3d vertices. Such that tactil (0,0) maps to the quad
	 * defined by the four vertices {(0,0),(0,1),(1,1),(1,0)}. Notice that the
	 * normal is defined by sequence of the vertices and that the normal defines
	 * the direction of tactile sensing.
	 */
	class TactileArraySensor: public SimulatedTactileSensor {
	public:

	    typedef rw::common::Ptr<TactileArraySensor> Ptr;

	    typedef rw::sensor::TactileArrayModel::ValueMatrix ValueMatrix;
	    typedef rw::sensor::TactileArrayModel::VertexMatrix VertexMatrix;

		/**
		 * @brief Creates a TactileSensor with a geometry specified by a height map and
		 * equally sized Texels in an matrix of a given dimension. The transform describe the
		 * location of the lower left corner of the texel (0,0).
		 * @param frame [in]
		 * @param fThmap [in]
		 * @param heightMap [in]
		 * @param texelSize [in]
		 */
		TactileArraySensor(const std::string& name,
		                    dynamics::Body::Ptr obj,
							const rw::math::Transform3D<>& fThmap,
							const ValueMatrix& heightMap,
							const rw::math::Vector2D<double>& texelSize);


		/**
		 * @brief destructor
		 */
		virtual ~TactileArraySensor();


		/**
		 * @brief get last sampled texel pressure values
		 * @param state [in] state in which values are to be found
		 */
		rw::sensor::TactileArrayModel::ValueMatrix& getTexelData(rw::kinematics::State& state)  const{ return _tmodel->getTexelData(state);}

		/**
		 * @brief get last sampled texel pressure values
		 */
		const rw::sensor::TactileArrayModel::ValueMatrix& getTexelData(const rw::kinematics::State& state)  const{ return _tmodel->getTexelData(state);}



        ///// From SimulatedTactileSensor interface

        //! @copydoc SimulatedTactileSensor::reset
        void reset(const rw::kinematics::State& state);

        //! @copydoc rwlibs::simulation::SimulatedTactileSensor::addForceW
        void addForceW(const rw::math::Vector3D<>& point,
                       const rw::math::Vector3D<>& force,
                       const rw::math::Vector3D<>& snormal,
                       rw::kinematics::State& state,
                       dynamics::Body::Ptr body = NULL);

        //! @copydoc rwlibs::simulation::SimulatedTactileSensor::addForce
        void addForce(const rw::math::Vector3D<>& point,
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& snormal,
                      rw::kinematics::State& state,
                      dynamics::Body::Ptr body = NULL);

        //! @copydoc rwlibs::simulation::SimulatedTactileSensor::addWrenchToCOM
        void addWrenchToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body::Ptr body=NULL);

        //! @copydoc rwlibs::simulation::SimulatedTactileSensor::addWrenchWToCOM
        void addWrenchWToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body::Ptr body=NULL);

        //! @copydoc rwlibs::simulation::SimulatedSensor::update
        void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

        //! @brief all contacts that was accumulated into pressure
        const std::vector<rw::sensor::Contact3D>& getActualContacts(const rw::kinematics::State& state);

        //****************** The static state interface

        //! @copydoc rw::sensor::TactileArrayModel::getTexelSize
        rw::math::Vector2D<> getTexelSize(int x, int y)  const{ return _tmodel->getTexelSize(x,y); }

        //! @copydoc rw::sensor::TactileArrayModel::getWidth
        int getWidth() const { return _tmodel->getWidth(); }
        //! @copydoc rw::sensor::TactileArrayModel::getHeight
        int getHeight() const { return _tmodel->getHeight(); }

        //! @copydoc rw::sensor::TactileArrayModel::getPressureLimit
		std::pair<double,double> getPressureLimit() const { return _tmodel->getPressureLimit(); }

		//! @copydoc rw::sensor::TactileArrayModel::getVertexGrid
		const VertexMatrix& getVertexGrid()  const {return _tmodel->getVertexGrid();};

		//! @copydoc rw::sensor::TactileArrayModel::getCenters
		const VertexMatrix& getCenters() const {return _tmodel->getCenters();};

		//! @copydoc rw::sensor::TactileArrayModel::getCenters
		const VertexMatrix& getNormals() const {return _tmodel->getNormals();}

		//! @copydoc rw::sensor::TactileArrayModel::getTransform
		const rw::math::Transform3D<>& getTransform()  const {return _tmodel->getTransform();}

		//! @copydoc rw::sensor::TactileArrayModel::getFrame
		rw::kinematics::Frame * getSensorFrame(){ return _tmodel->getFrame(); }

		rw::sensor::TactileArrayModel::Ptr getTactileArrayModel(){ return _tmodel;}

		//! @copydoc rwlibs::simulation::SimulatedSensor::getSensor
		rw::sensor::Sensor::Ptr getSensor(rwlibs::simulation::Simulator::Ptr sim);

		/**
		 * @brief get a handle to the statefull instance of the simulated sensor
		 * @param sim [in] simulator in which instance is active.
		 * @return handle to instance of simulated sensor
		 */
		rw::sensor::TactileArray::Ptr getTactileArraySensor(rwlibs::simulation::Simulator::Ptr sim);


		////////////// following belongs to the specific modeling of this type of tactile array

		/**
		 * @brief sets the deformation mask used for determining the deformation
		 * of the sensor surface when a point force is applied to it.
		 *
		 * @note make sure that the sum of all elements in the mask is 1 or less.
		 */
		void setDeformationMask(const ValueMatrix& dmask, double width, double height);

		/**
		 * @brief set max penetration in meters. The penetration is really max deformation
		 * at any point on the elastic surface of the tactile sensor
		 */
		void setMaxPenetration(double penetration){ _maxPenetration = penetration; }

	public:

		struct DistPoint {
			DistPoint():p1(0,0,0),p2(0,0,0),dist(10000000.0){}
			rw::math::Vector3D<> p1;
			rw::math::Vector3D<> p2;
			double dist;
		};

		/**
		 * @brief get triangle mesh representing the tactile array
		 * @return triangle mesh
		 */
		const rw::geometry::PlainTriMeshD& getMesh(){return *_ntrimesh;}


	protected:

		//! class for keeping statefull information
		class ClassState: public rw::kinematics::StateCache {
		public:
		    typedef rw::common::Ptr<TactileArraySensor::ClassState> Ptr;

		    ClassState(TactileArraySensor* tsensor,  size_t dim_x, size_t dim_y);

	        //! @copydoc TactileArray::acquire
		     void acquire();

	        //! @copydoc TactileArray::getTexelData
	         Eigen::MatrixXf getTexelData() const;

	         void setTexelData(const Eigen::MatrixXf& data);

	        ///// From SimulatedTactileSensor interface
	         /**
	           * @copydoc SimulatedTactileSensor::reset
	           */
	         void reset(const rw::kinematics::State& state);

	        //! @copydoc rwlibs::simulation::SimulatedTactileSensor::addForceW
	         void addForceW(const rw::math::Vector3D<>& point,
	                       const rw::math::Vector3D<>& force,
	                       const rw::math::Vector3D<>& snormal,
	                       dynamics::Body::Ptr body = NULL);

	        //! @copydoc rwlibs::simulation::SimulatedTactileSensor::addForce
	         void addForce(const rw::math::Vector3D<>& point,
	                      const rw::math::Vector3D<>& force,
	                      const rw::math::Vector3D<>& snormal,
	                      dynamics::Body::Ptr body = NULL);

            //! @copydoc rwlibs::simulation::SimulatedSensor::update
             void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

            //! @copydoc rwlibs::simulation::TactileArraySensor::getActualContacts
             const std::vector<rw::sensor::Contact3D>& getActualContacts(){ return _allForces; };

            std::vector<TactileArraySensor::DistPoint>
                generateContacts(dynamics::Body *body, const rw::math::Vector3D<>& normal, const rw::kinematics::State& state);

             size_t size() const{
                return 0;
            }

            /**
             * @brief this creates a deep copy of this cache
             */
            rw::common::Ptr<rw::kinematics::StateCache> clone() const{
                return rw::common::ownedPtr( new ClassState( *this ) );
            }

            TactileArraySensor* _tsensor;

            Eigen::MatrixXf _distMatrix;
            ValueMatrix _accForces,_pressure;
            double _accTime, _stime;
            rw::math::Transform3D<> _wTf, _fTw;
            std::vector<rw::sensor::Contact3D> _allAccForces,_allForces;
            std::map<dynamics::Body::Ptr, std::vector<rw::sensor::Contact3D> > _forces;
            rw::proximity::ProximityStrategyData _pdata;

        };

		ClassState::Ptr getClassState(rw::kinematics::State& state) const;
        ClassState::Ptr getClassState(rw::kinematics::State& state);

	protected:
		rw::kinematics::StatelessData<int> _sdata;

		// matrix containing the surface normal of each tactil. Calculated from VertexShape
		VertexMatrix _contactMatrix;
		VertexMatrix _distCenterMatrix;

		//rw::common::Ptr<StateModel> _model;

		Eigen::MatrixXf _distDefMatrix;
		const rw::math::Vector2D<> _texelSize;
		double _texelArea;
		const rw::math::Transform3D<> _fThmap,_hmapTf;

		// distribution mask, a mask where the sum of all elements is 1.
		// it describes the deformation around a point force.
		Eigen::MatrixXf _dmask;

		rwlibs::proximitystrategies::ProximityStrategyPQP *_narrowStrategy;

		// max penetration in meter
		double _maxPenetration,_elasticity;
		// lowpass filter time constant
		double _tau;

		dynamics::Body::Ptr _body;

		double _maskWidth, _maskHeight;

		// the
		//const VertexMatrix& _vMatrix;
		//std::vector<Contact3D> _forces;


		rw::geometry::Geometry::Ptr _ngeom;
		rw::common::Ptr<rw::geometry::PlainTriMesh<rw::geometry::Triangle<> > > _ntrimesh;
		rw::proximity::ProximityModel::Ptr _nmodel;
		std::map<rw::kinematics::Frame*, std::vector<rw::geometry::Geometry::Ptr> > _frameToGeoms;

        rw::sensor::TactileArrayModel::Ptr _tmodel;
	};
	//! @}
}
}
#endif /*TactileArraySensor_HPP_*/
