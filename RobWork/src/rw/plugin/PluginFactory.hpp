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


#ifndef RW_PLUGIN_PLUGINFACTORY_HPP
#define RW_PLUGIN_PLUGINFACTORY_HPP

#include "PluginFactoryBase.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/common/macros.hpp>


namespace rw {
namespace plugin {



/** @addtogroup plugin */
/*@{*/



/**
 * @brief Template class form which to inherit when creating a factory for a plugin.
 *
 * Example: If we wish to create a new type of rw::models::Device the following steps are needed.
 * 1) Implement you device "MyDevice" which should inherit from rw::models::Device.

 * 2) Create a Factory (lets call it MyDeviceFactory) which inherits from PluginFactory<T> and 
 * specifies T=rw::models::Device. It is important to specify T=rw::models::Device and NOT MyDevice
 * as the PluginRepository can otherwise not identify it as a factory for creating a device.
 *
 * 3) Override the relevant "make" methods from PluginFactory<T> to provide a way of creating instances
 * of MyDevice. Which methods to override depends on how and where you wish to use your plugin. See the
 * documentation of the different methods.
 *
 * 4) Call DLL_FACTORY_METHOD(MyDeviceFactory) to create a proper entry in the dynamic library. Make sure 
 * that MyDeviceFactory has a default constructor. 
 */
template <class T>
class PluginFactory: public PluginFactoryBase {
public:
	//! @brief Smart pointer type for PluginFactory.
    typedef rw::common::Ptr<PluginFactory<T> > Ptr;

    /**
     * @brief Constructs a PluginFactory
     * 
     * @param identifier [in] Identifiers of the PluginFactory
     */
    PluginFactory(const std::string& identifier):
        PluginFactoryBase(identifier)
    {

    }
    
    /**
     * @brief Creates plugin of type T by calling default constructor
     *
     * If method is not overridden an exception is thrown.
     * 
     * @return Pointer with ownership to new plugin instance.
     */
    virtual rw::common::Ptr<T> make() {
        RW_THROW2(5000, "PluginFactory<T>::make() is not implemented");
    };

    /**
     * @brief Creates plugin of type T by calling constructor with std::string
     *
     * If method is not overridden an exception is thrown.
     * 
     * @return Pointer with ownership to new plugin instance.
     */
    virtual rw::common::Ptr<T> make(const std::string&) {
        RW_THROW2(5001, "PluginFactory<T>::make(const std::string&) is not implemented");
    };

};

/** @} */

} //end namespace plugin
} //end namespace rw


#endif //#ifndef RW_PLUGIN_PLUGINFACTORY_HPP
