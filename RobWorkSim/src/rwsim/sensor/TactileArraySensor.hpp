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
#include <rw/sensor/Contact3D.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/multi_array.hpp>
#include <rw/geometry.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <vector>
#include <map>

#include "SimulatedTactileSensor.hpp"

namespace rwsim {
namespace sensor {
	//! @addtogroup dynamics
	//! @{

	/**
	 * @brief the TactileMatrixSensor class combines a TactileMatrix data
	 * type with the actual shape of a tactile sensor. The shape is described
	 * with a matrix of 3d vertices. Such that tactil (0,0) maps to the quad
	 * defined by the four vertices {(0,0),(0,1),(1,1),(1,0)}. Notice that the
	 * normal is defined by sequence of the vertices and that the normal defines
	 * the direction of tactile sensing.
	 */
	class TactileArraySensor: public rw::sensor::TactileArray, public SimulatedTactileSensor {
	public:

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
							rw::kinematics::Frame* frame,
							const rw::math::Transform3D<>& fThmap,
							const ValueMatrix& heightMap,
							const rw::math::Vector2D<double>& texelSize);

		/**
		 * @brief destructor
		 */
		virtual ~TactileArraySensor(){};

		//******************* stateless interface *******************
		//// From TactileArray interface
		/**
		 * @copydoc TactileArray::acquire
		 */
		void acquire(rw::kinematics::State& state){
		    getClassState(state)->acquire();
		}

		/**
		 * @copydoc TactileArray::getTexelData
		 */
		boost::numeric::ublas::matrix<float> getTexelData(const rw::kinematics::State& state)  const{
		    return getClassState(state)->getTexelData();
		}

        void setTexelData(const boost::numeric::ublas::matrix<float>& data, rw::kinematics::State& state){
            getClassState(state)->setTexelData(data);
        }

        ///// From SimulatedTactileSensor interface
         /**
           * @copydoc SimulatedTactileSensor::reset
           */
        void reset(const rw::kinematics::State& state){
            getClassState(state)->reset(state);
        };

        /**
         * @copydoc rwlibs::simulation::SimulatedTactileSensor::addForceW
         */
        void addForceW(const rw::math::Vector3D<>& point,
                       const rw::math::Vector3D<>& force,
                       const rw::math::Vector3D<>& snormal,
                       rw::kinematics::State& state,
                       dynamics::Body *body = NULL){
            getClassState(state)->addForceW(point, force, snormal, body);
        }

        /**
         * @copydoc rwlibs::simulation::SimulatedTactileSensor::addForce
         */
        void addForce(const rw::math::Vector3D<>& point,
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& snormal,
                      rw::kinematics::State& state,
                      dynamics::Body *body = NULL){
            getClassState(state)->addForce(point, force, snormal, body);
        }

        void addWrenchToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body *body=NULL){ };

        void addWrenchWToCOM(
                      const rw::math::Vector3D<>& force,
                      const rw::math::Vector3D<>& torque,
                      rw::kinematics::State& state,
                      dynamics::Body *body=NULL){};

        /**
         * @copydoc rwlibs::simulation::SimulatedSensor::update
         */
        void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
            getClassState(state)->update(info, state);
        }

        /**
         * @brief all contacts that was accumulated into pressure
         * @return
         */
        const std::vector<rw::sensor::Contact3D>& getActualContacts(const rw::kinematics::State& state){
            return getClassState(state)->getActualContacts();
        }


        //****************** The static state interface

        std::pair<size_t, size_t> getTexelDataSize(){return std::make_pair(_w,_h); };
        /**
         * @copydoc TactileArray::getTexelSize
         */
        rw::math::Vector2D<> getTexelSize(int x, int y)  const {
            return _texelSize;
        }

        int getWidth() const{
            return _w;
        }

        int getHeight() const{
            return _h;
        }


		/**
		 * @copydoc TactileArray::getPressureLimit
		 */
		std::pair<double,double> getPressureLimit() const {
			return std::make_pair(_minPressure,_maxPressure);
		}

		/**
		 * @copydoc TactileArray::getVertexGrid
		 */
		const VertexMatrix& getVertexGrid()  const {return _vMatrix;};

		/**
		 * @copydoc TactileArray::getCenters
		 */
		const VertexMatrix& getCenters() const {return _centerMatrix;};

		/**
		 * @copydoc TactileArray::getNormals
		 */
		const VertexMatrix& getNormals() const {return _normalMatrix;}

		/**
		  * @copydoc TactileArray::getTransform
		  */
		 const rw::math::Transform3D<>& getTransform()  const {
			return _fThmap;
		}


		rw::sensor::Sensor* getSensor(){ return this;};

		/**
		 * @brief set the upper and lower pressure bound that the sensors can measure
		 * in kPa
		 * @param minPressure
		 * @param maxPressure
		 */
		void setPressureLimit(double minPressure, double maxPressure) {
			_minPressure = minPressure;
			_maxPressure = maxPressure;
		}

		/**
		 * @brief sets the deformation mask used for determining the deformation
		 * of the sensor surface when a point force is applied to it.
		 *
		 * @note make sure that the sum of all elements in the mask is 1 or less.
		 */
		void setDeformationMask(const ValueMatrix& dmask, double width, double height){
			_dmask = dmask;
			_maskWidth = width;
			_maskHeight = height;
		}

		void setMaxPenetration(double penetration){
			_maxPenetration = penetration;
		}

		rw::kinematics::Frame * getSensorFrame(){ return getFrame(); }

	public:

		struct DistPoint {
			DistPoint():p1(0,0,0),p2(0,0,0),dist(10000000.0){}
			rw::math::Vector3D<> p1;
			rw::math::Vector3D<> p2;
			double dist;
		};

		const rw::geometry::PlainTriMesh<rw::geometry::Triangle<> >& getMesh(){return *_ntrimesh;}


	public:

		class ClassState: public rw::kinematics::ObjectStateData {
		public:
		    typedef rw::common::Ptr<TactileArraySensor::ClassState> Ptr;

		    ClassState(TactileArraySensor* tsensor, const ValueMatrix& heightMap);

	        //! @copydoc TactileArray::acquire
		    virtual void acquire();

	        //! @copydoc TactileArray::getTexelData
	        virtual boost::numeric::ublas::matrix<float> getTexelData() const;

	        virtual void setTexelData(const boost::numeric::ublas::matrix<float>& data);

	        ///// From SimulatedTactileSensor interface
	         /**
	           * @copydoc SimulatedTactileSensor::reset
	           */
	        virtual void reset(const rw::kinematics::State& state);

	        //! @copydoc rwlibs::simulation::SimulatedTactileSensor::addForceW
	        virtual void addForceW(const rw::math::Vector3D<>& point,
	                       const rw::math::Vector3D<>& force,
	                       const rw::math::Vector3D<>& snormal,
	                       dynamics::Body *body = NULL);

	        //! @copydoc rwlibs::simulation::SimulatedTactileSensor::addForce
	        virtual void addForce(const rw::math::Vector3D<>& point,
	                      const rw::math::Vector3D<>& force,
	                      const rw::math::Vector3D<>& snormal,
	                      dynamics::Body *body = NULL);

            //! @copydoc rwlibs::simulation::SimulatedSensor::update
            virtual void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

            //! @copydoc rwlibs::simulation::TactileArraySensor::getActualContacts
            virtual const std::vector<rw::sensor::Contact3D>& getActualContacts(){ return _allForces; };

            std::vector<TactileArraySensor::DistPoint>
                generateContacts(dynamics::Body *body, const rw::math::Vector3D<>& normal, const rw::kinematics::State& state);

            TactileArraySensor* _tsensor;
            ValueMatrix _accForces,_pressure;
            rw::math::Transform3D<> _wTf, _fTw;
            std::vector<rw::sensor::Contact3D> _allAccForces,_allForces;
            std::map<dynamics::Body*, std::vector<rw::sensor::Contact3D> > _forces;
            double _accTime, _stime;
            rw::proximity::ProximityStrategyData _pdata;
            boost::numeric::ublas::matrix<float> _distMatrix;

        };

        const ClassState::Ptr getClassState(const rw::kinematics::State& state) const {
            return _cstate;
        }

        ClassState::Ptr getClassState(const rw::kinematics::State& state){
            return _cstate;
        }

        void setClassState(ClassState::Ptr cstate, rw::kinematics::State& state){
            _cstate = cstate;
        }


	protected:
		ClassState::Ptr _cstate;

		VertexMatrix _centerMatrix;
		// matrix containing the surface normal of each tactil. Calculated from VertexShape
		VertexMatrix _normalMatrix;
		VertexMatrix _contactMatrix;
		VertexMatrix _distCenterMatrix;

		//rw::common::Ptr<StateModel> _model;

		boost::numeric::ublas::matrix<float> _distDefMatrix;

		const rw::math::Vector2D<> _texelSize;
		const rw::math::Transform3D<> _fThmap,_hmapTf;

		// width and height of matrix
		const int _w,_h;
		// lowpass filter time constant
		double _tau;
		double _texelArea;

		// matrix containing the center position of each tactil. Calculated from VertexShape
		VertexMatrix _vMatrix;

		// distribution mask, a mask where the sum of all elements is 1.
		// it describes the deformation around a point force.
		boost::numeric::ublas::matrix<float> _dmask;
		double _maskWidth, _maskHeight;
		double _minPressure,_maxPressure;

		// the
		//const VertexMatrix& _vMatrix;
		//std::vector<Contact3D> _forces;

		rwlibs::proximitystrategies::ProximityStrategyPQP *_narrowStrategy;

		rw::proximity::ProximityModel *model;

		// max penetration in meter
		double _maxPenetration,_elasticity;

		rw::geometry::Geometry::Ptr _ngeom;
		rw::common::Ptr<rw::geometry::PlainTriMesh<rw::geometry::Triangle<> > > _ntrimesh;
		rw::proximity::ProximityModel::Ptr _nmodel;


		std::map<rw::kinematics::Frame*, std::vector<rw::geometry::Geometry::Ptr> > _frameToGeoms;



	};
	//! @}
}
}
#endif /*TactileArraySensor_HPP_*/
