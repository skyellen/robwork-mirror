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

#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/Ptr.hpp>
#include <vector>
#include <map>
#include <string>
#include <ostream>

namespace rw { namespace models {

    class Device;
    class DynamicObject;

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
        /**
         * @brief Constructs a WorkCell
         *
         * @param tree [in] The (initial) tree structure of the WorkCell
         *
         * @param name [in] The name of the workcell. A good name for the
         * workcell would be the (eventual) file that the workcell was loaded
         * from.
         */
        WorkCell(
            kinematics::StateStructure::Ptr tree,
            const std::string& name = "");

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
         * @brief Adds a Device to the WorkCell.
         *
         * Ownership of \b device is taken.
         *
         * @param device [in] pointer to device.
         */
        void addDevice(Device* device);

        /**
         * @brief Returns a reference to a vector with pointers to the Device(s)
         * in the WorkCell
         *
         * @return const vector with pointers to Device(s).
         */
        const std::vector<Device*>& getDevices() const;

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
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device.
         *
         * @param name [in] The workcell name
         *
         * @return The device named \b name or NULL if no such device.
         */
        Device* findDevice(const std::string& name) const;

        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device or if the device is not of type \b T.
         *
         * @param name [in] The workcell name
         * @return The device named \b name or NULL if no such device is found or if the device is not of type \b T.
         */
        template<class T>
        T* findDevice(const std::string& name) const{
        	rw::models::Device *dev = findDevice(name);
        	if(dev==NULL) return NULL;
        	return dynamic_cast<T*>(dev);
        }

        /**
         * @brief Returns a vector with pointers to the Device(s) with a specific type \b T
         * in the WorkCell
         *
         * @return vector with pointers to Device(s) of type T.
         */
        template<class T>
        std::vector<T*> findDevices() const{
        	std::vector<T*> result;
        	BOOST_FOREACH(Device* dev, _devices){
        		T* res = dynamic_cast<T*>(dev);
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
         * @brief gets the complete state structure of the workcell.
         * @return the state structure of the workcell.
         */
        rw::common::Ptr<rw::kinematics::StateStructure> getStateStructure(){
            return _tree;
        }

        /**
         * @brief Properties of this workcell
         */
        rw::common::PropertyMap& getPropertyMap(){ return _map;}

    private:
        rw::kinematics::StateStructure::Ptr _tree;
        std::vector<Device*> _devices;
        std::string _name;
        rw::common::PropertyMap _map;

    private:
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
