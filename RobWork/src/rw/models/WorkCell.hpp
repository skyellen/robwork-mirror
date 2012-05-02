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


#ifndef RW_MODELS_WORKCELL_HPP
#define RW_MODELS_WORKCELL_HPP



/**
 * @file WorkCell.hpp
 */
#include <rw/graphics/SceneDescriptor.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/sensor/Sensor.hpp>
#include <rw/common/Ptr.hpp>

#include "Device.hpp"
#include "Object.hpp"
#include <vector>
#include <map>
#include <string>
#include <ostream>



namespace rw { namespace proximity {
    class CollisionSetup;
}}

namespace rw { namespace models {
    
	//class Device;

    /** @addtogroup models */
    /*@{*/

#ifdef RW_USE_DEPRECATED
    class WorkCell;

    //! A pointer to a WorkCell.
    typedef rw::common::Ptr<WorkCell> WorkCellPtr;
#endif
    /**
     * @brief WorkCell keeps track of devices, obstacles and objects in the
     * scene.
     *
     * WorkCell is a pretty dumb container to which you can add your devices and
     * the frames you your GUI to show as objects or camera views.
     *
     * WorkCell is responsible for keeping track of everything including all
     * devices, object and obstacles in the environment. WorkCell contains the
     * World Frame, which represents the root and the only frame without a
     * parent.
     *
     * It should be noted that WorkCell supports only very static work cells:
     * Following initialization you can not add or remove frames of the forward
     * kinematics tree. An interface for such more dynamic workcells will have
     * to be designed later.
     */
    class WorkCell {
    public:

		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<WorkCell> Ptr;

		typedef enum{STATE_DATA_ADDED,STATE_DATA_REMOVED} WorkCellEventType;

        /**
         * @brief Constructs an empty WorkCell
         *
         * @param name [in] The name of the workcell. A good name for the
         * workcell would be the (eventual) file that the workcell was loaded
         * from.
         */
        WorkCell(const std::string& name);

		/**
         * @brief Constructs a WorkCell
         *
         * @param tree [in] The (initial) tree structure of the WorkCell
         *
         * @param name [in] The name of the workcell. A good name for the
         * workcell would be the (eventual) file that the workcell was loaded
         * from.
		 * 
		 * @param filename [in] The filename from which the workcell is loaded.
         */
        WorkCell(
            kinematics::StateStructure::Ptr tree,
            const std::string& name = "",
			const std::string& filename = "");

        /**
         * Destroys a work cell including the devices that have been added.
         *
         * Management of the frames is done by a tree of which the work cell
         * knows nothing. Therefore if this kinematics tree is still in
         * existence (which it probably is), then the frames that used to be
         * accessible via this work cell will still be valid.
         */
        ~WorkCell();

        /**
           @brief The name of the workcell or the empty string if no name was
           provided.
         */
        std::string getName() const { return _name; }

        /**
         * @brief Returns pointer to the world frame
         *
         * @return Pointer to the world frame
         */
        kinematics::Frame* getWorldFrame() const;

		/**
		 * @brief Adds \b frame with \b parent as parent.
		 *
		 * If parent == NULL, then \b world is used as parent
		 *
		 * @param frame [in] Frame to add
		 * @param parent [in] Parent frame - uses World is parent == NULL
		 */
        void addFrame(kinematics::Frame* frame, kinematics::Frame* parent=NULL);

		/**
		 * @brief Adds dynamically attachable frame (DAF) \b frame with \b parent as parent.
		 *
		 * If parent == NULL, then \b world is used as parent
		 *
		 * @param frame [in] Frame to add
		 * @param parent [in] Parent frame - uses World is parent == NULL
		 */
        void addDAF(kinematics::Frame* frame, kinematics::Frame* parent=NULL);

		/**
		 * @brief Removes \b frame from work cell
		 * 
		 * @parem frame [in] Frame to remove
		 */
        void remove(kinematics::Frame* frame);

        /**
         * @brief Adds a Device to the WorkCell.
         *
         * Ownership of \b device is taken.
         *
         * @param device [in] pointer to device.
         */
		void addDevice(rw::common::Ptr<Device> device);


        /**
         * @brief Returns a reference to a vector with pointers to the Device(s)
         * in the WorkCell
         *
         * @return const vector with pointers to Device(s).
         */
		const std::vector<rw::common::Ptr<Device> >& getDevices() const;

        /**
         * @brief Returns frame with the specified name.
         *
         * If multiple frames has the same name, the first frame encountered
         * will be returned. If no frame is found, the method returns NULL.
         *
         * @param name [in] name of Frame.
         *
         * @return The frame with name \b name or NULL if no such frame.
         */
        kinematics::Frame* findFrame(const std::string& name) const;

        /**
         * @brief Returns frame with the specified name and type \b T.
         *
         * If multiple frames has the same name, the first frame encountered
         * will be returned. If no frame is found, the method returns NULL.
         * if a frame is found and it is nt of type \b T then NULL is returned.
         *
         * @param name [in] name of Frame.
         *
         * @return The frame with name \b name or NULL if no such frame or the frame is not of type \b T.
         */
        template<class T>
        T* findFrame(const std::string& name) const{
        	rw::kinematics::Frame *frame = findFrame(name);
        	if(frame==NULL) return NULL;
        	return dynamic_cast<T*>(frame);
        }

        /**
         * @brief Returns all frames of a specific type \b T.
         * @return all frames of type \b T in the workcell
         */
        template<class T>
        std::vector<T*> findFrames() const{
        	const std::vector<rw::kinematics::Frame*> frames = _tree->getFrames();
        	std::vector<T*> result;
        	BOOST_FOREACH(rw::kinematics::Frame* f, frames){
        		T* res = dynamic_cast<T*>(f);
        		if(res!=NULL)
        			result.push_back(res);
        	}
        	return result;
        }

		/**
		 * @brief Returns all frames in workcell
		 * @return List of all frames
		 */
		std::vector<rw::kinematics::Frame*> getFrames() const {
			std::vector<rw::kinematics::Frame*> frames;
			BOOST_FOREACH(rw::kinematics::Frame* frame, _tree->getFrames()){
			    if(frame!=NULL)
			        frames.push_back(frame);
			}

		    return frames;
		}

		

        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device.
         *
         * @param name [in] The device name
         *
         * @return The device named \b name or NULL if no such device.
         */
		rw::common::Ptr<Device> findDevice(const std::string& name) const;

        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device or if the device is not of type \b T.
         *
         * @param name [in] The workcell name
         * @return The device named \b name or NULL if no such device is found or if the device is not of type \b T.
         */
        template<class T>
		rw::common::Ptr<T> findDevice(const std::string& name) const{
            rw::common::Ptr<Device> dev = findDevice(name);
        	if(dev==NULL) 
				return NULL;
        	return dev.cast<T>();
        }

        /**
         * @brief Returns a vector with pointers to the Device(s) with a specific type \b T
         * in the WorkCell
         *
         * @return vector with pointers to Device(s) of type T.
         */
        template<class T>
		std::vector<rw::common::Ptr<T> > findDevices() const{
			std::vector<rw::common::Ptr<T> > result;
        	BOOST_FOREACH(rw::common::Ptr<Device> dev, _devices){
				rw::common::Ptr<T> res = dev.cast<T>(dev);
        		if(res!=NULL)
        			result.push_back(res);
        	}
        	return result;
        }

        /**
         * @brief Returns a default State
         *
         * @return default State
         */
        kinematics::State getDefaultState() const;

        /**
         * @brief Returns sensor with the specified name.
         *
         * If multiple sensors has the same name, the first sensor encountered
         * will be returned. If no sensor is found, the method returns NULL.
         *
         * @param name [in] name of sensor.
         *
         * @return The sensor with name \b name or NULL if no such sensor.
         */
        rw::sensor::Sensor::Ptr findSensor(const std::string& name) const;

        /**
         * @brief Returns sensor with the specified name and type \b T.
         *
         * If multiple sensors has the same name, the first sensor encountered
         * will be returned. If no sensor is found, the method returns NULL.
         * if a sensor is found and it is nt of type \b T then NULL is returned.
         *
         * @param name [in] name of sensor.
         *
         * @return The sensor with name \b name or NULL if no such sensor or the sensor is not of type \b T.
         */
        template<class T>
        rw::common::Ptr<T> findSensor(const std::string& name) const{
            rw::sensor::Sensor::Ptr sensor = findSensor(name);
            if(sensor==NULL)
                return NULL;
            return sensor.cast<T>();
        }

        /**
         * @brief Returns all frames of a specific type \b T.
         * @return all frames of type \b T in the workcell
         */
        template<class T>
        std::vector<rw::common::Ptr<T> > findSensors() const{
            const std::vector<rw::sensor::Sensor::Ptr> sensors = _sensors;
            std::vector<rw::common::Ptr<T> > result;
            BOOST_FOREACH(rw::sensor::Sensor::Ptr f, sensors){
                rw::common::Ptr<T> res = f.cast<T>();
                if(res!=NULL)
                    result.push_back(res);
            }
            return result;
        }

        /**
         * @brief Returns all frames in workcell
         * @return List of all frames
         */
        std::vector<rw::sensor::Sensor::Ptr> getSensors() const {
            std::vector<rw::sensor::Sensor::Ptr> sensors;
            BOOST_FOREACH(rw::sensor::Sensor::Ptr sensor, _sensors){
                if(sensor!=NULL)
                    sensors.push_back(sensor);
            }

            return sensors;
        }


		/**
		 * @brief Returns all object in the work cell
		 *
		 * @return All object in work cell
		 */
        std::vector<Object::Ptr> getObjects() const { return _objects; };


        /**
         * @brief The object named \b name of the workcell.
         *
         * NULL is returned if there is no such object.
         *
         * @param name [in] The object name
         *
         * @return The object named \b name or NULL if no such object.
         */
        rw::common::Ptr<Object> findObject(const std::string& name) const;

        /**
		 * @brief Adds de
		 */
      //  void add(rw::common::Ptr<Device> device){ addDevice(device); }
        void add(rw::common::Ptr<Object> object);
        void add(rw::common::Ptr<rw::sensor::Sensor> sensor);




        /**
         * @brief gets the complete state structure of the workcell.
         * @return the state structure of the workcell.
         */
        rw::common::Ptr<rw::kinematics::StateStructure> getStateStructure(){
            return _tree;
        }


		 /**
         * @brief Definition of work cell changed listener
         */
        typedef boost::function<void(int)> WorkCellChangedListener;

        /**
         * @brief Definition of even for work cell changed
         */
        typedef rw::common::Event<WorkCellChangedListener, int>  WorkCellChangedEvent;

        /**
         * @brief Returns the work cell changed event
         * @return
         */
        WorkCellChangedEvent& workCellChangedEvent() {
            return _workCellChangedEvent;
        }

        /**
         * @brief Properties of this workcell
         */
        rw::common::PropertyMap& getPropertyMap(){ return _map;}

		/**
		 * @brief Returns collision setup associated to work cell
		 *
		 * @return Collision setup
		 */
        rw::proximity::CollisionSetup getCollisionSetup();

        rw::graphics::SceneDescriptor::Ptr getSceneDescriptor(){ return _sceneDescriptor;}
        void setSceneDescriptor(rw::graphics::SceneDescriptor::Ptr scene){ _sceneDescriptor = scene;}

		/**
		 * @brief Returns the full path and filename of the workcell.
		 *
		 * If the workcell is loaded from file, then this method returns the full filename. Otherwise
		 * it returns an empty string.
		 */
		std::string getFilename() const;
    protected:
        void stateDataAddedListener(const rw::kinematics::StateData* data);
        void stateDataRemovedListener(const rw::kinematics::StateData* data);

    private:
        rw::kinematics::StateStructure::Ptr _tree;
		std::vector<rw::common::Ptr<Device> > _devices;
		std::vector<rw::common::Ptr<Object> > _objects;
        std::string _name;
		std::string _filename;
        rw::common::PropertyMap _map;
        WorkCellChangedEvent _workCellChangedEvent;
        std::vector<rw::sensor::Sensor::Ptr> _sensors;
        rw::common::Ptr<rw::graphics::SceneDescriptor> _sceneDescriptor;
    private:
        WorkCell();
        WorkCell(const WorkCell&);
        WorkCell& operator=(const WorkCell&);
    };

    /**
       @brief Streaming operator.
    */
    std::ostream& operator<<(std::ostream& out, const WorkCell& workcell);

    /*@}*/
}} // end namespaces

#endif // end include guard
