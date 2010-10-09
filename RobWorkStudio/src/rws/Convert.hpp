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


#ifndef RWS_CONVERT_HPP
#define RWS_CONVERT_HPP


#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/ParallelDevice.hpp>
#include <rw/models/MobileDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/models/Joint.hpp>


#include <vector>
#include <map>


namespace rws {
/**
 * @brief used to downcast Devices to their specific type
 */
	


class Convert
{
public:
    typedef rw::kinematics::Frame Frame;
    typedef rw::models::Device Device;

private:
    void setupFrames(
        const Frame* frame,
        const rw::kinematics::State& state);
    void setupDevices(rw::models::WorkCell* workcell);

    enum DeviceTypes { SERIALDEVICE = 0, CARTESIAN6DOFDEVICE, TREEDEVICE,
                        PARALLELDEVICE, MOBILEDEVICE};
    enum FrameTypes { JOINT = 0, LINK, DYNAMICOBJECT };

    typedef std::map<Device*, DeviceTypes> DeviceMap;
    DeviceMap _devicemap;

    typedef std::map<const Frame*, FrameTypes> FrameMap;
    FrameMap _framemap;

public:
	/**
	 * @brief default constructor
	 */
    Convert() {}

	/**
	 * @brief constructor
	 * @param workcell [in] initialize the Convert object with a workcell
	 */
    explicit Convert(rw::models::WorkCell* workcell);

    /**
     * Downcast a device to a Serial device
     * @param device [in] the device that should be downcasted
     * @return A serial device if device is a serialdevice or
     * NULL if device is not a serial device
     */
    rw::models::SerialDevice* toSerialDevice(Device* device) const
    {
        const DeviceMap::const_iterator it = _devicemap.find(device);
        if (it != _devicemap.end() && (*it).second == SERIALDEVICE)
            return static_cast<rw::models::SerialDevice*>(device);
        else
            return 0;
    }

    /**
     * Downcast a device to a tree device
     * @param device [in] the device that should be downcasted
     * @return A tree device if device is a TreeDevice or
     * NULL if device is not a tree device
     */
    rw::models::TreeDevice* toTreeDevice(Device* device) const
    {
        const DeviceMap::const_iterator it = _devicemap.find(device);
        if (it != _devicemap.end() && (*it).second == TREEDEVICE)
            return static_cast<rw::models::TreeDevice*>(device);
        else
            return 0;
    }

    /**
     * Downcast a device to a parallel device
     * @param device [in] the device that should be downcasted
     * @return A parallel device if device is a ParallelDevice or
     * NULL if device is not a ParallelDevice
     */
    rw::models::ParallelDevice* toParallelDevice(Device* device) const
    {
        const DeviceMap::const_iterator it = _devicemap.find(device);
        if (it != _devicemap.end() && (*it).second == PARALLELDEVICE)
            return static_cast<rw::models::ParallelDevice*>(device);
        else
            return 0;
    }

    /**
     * Downcast a device to a MobileDevice
     * @param device [in] the device that should be downcasted
     * @return A MobileDevice if device is a MobileDevice or
     * NULL if device is not a MobileDevice
     */
    rw::models::MobileDevice* toMobileDevice(Device* device) const
    {
        const DeviceMap::const_iterator it = _devicemap.find(device);
        if (it != _devicemap.end() && (*it).second == MOBILEDEVICE)
            return static_cast<rw::models::MobileDevice*>(device);
        else
            return 0;
    }

    /**
     * Downcast a device to a Cartesian6DOFDEvice
     * @param device [in] the device that should be downcasted
     * @return A Cartesian6DOFDEvice if device is a Cartesian6DOFDEvice or
     * NULL if device is not a Cartesian6DOFDEvice
     */
    /*rw::models::Cartesian6DOFDevice* toCartesian6DOFDEvice(
        Device* device) const
    {
        const DeviceMap::const_iterator it = _devicemap.find(device);
        if (it != _devicemap.end() && (*it).second == CARTESIAN6DOFDEVICE)
            return static_cast<rw::models::Cartesian6DOFDevice*>(device);
        else
            return 0;
    }
    */

    /**
     * Downcast a Frame to a Joint
     * @param frame [in] the Frame that should be downcasted
     * @return A Joint if frame is of type Joint or
     * NULL if frame is not a Joint
     */
    rw::models::Joint* toJoint(Frame* frame) const
    {
        const FrameMap::const_iterator it = _framemap.find(frame);
        if (it != _framemap.end() && (*it).second == JOINT)
            return static_cast<rw::models::Joint*>(frame);
        else
            return 0;
    }
};

}

#endif
