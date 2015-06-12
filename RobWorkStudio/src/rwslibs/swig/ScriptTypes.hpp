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

#ifndef RWS_SWIG_REMOTETYPES_HPP_
#define RWS_SWIG_REMOTETYPES_HPP_

#include <rwlibs/swig/ScriptTypes.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
/*
#ifdef __cplusplus
extern "C" {
#endif
    int luaopen_rws(struct lua_State* L); // declare the wrapped module
#ifdef __cplusplus
}
#endif
*/
namespace rws {
namespace swig {

    template <typename T>
    std::string toString(const T& x)
    {
        std::ostringstream buf;
        buf << x;
        return buf.str();
    }

    typedef rws::RobWorkStudio RobWorkStudio;
    typedef rws::RobWorkStudioPlugin RobWorkStudioPlugin;

    typedef rws::RWStudioView3D RWStudioView3D;



    // for now we add all static functions here
    RobWorkStudio* getRobWorkStudio();

    /**
     * @brief set current robworkstudio instance
     */
    void setRobWorkStudio(RobWorkStudio* rwstudio);

    /// These functions all work on the current robworkstudio state

    rw::common::Ptr<RobWorkStudio> getRobWorkStudioInstance();

    rw::common::Ptr<RobWorkStudio> getRobWorkStudioInstance(const std::string& args);

    const rwlibs::swig::State& getState();
    void setState(rwlibs::swig::State& state);
    rw::common::Ptr<rwlibs::swig::Device> findDevice(const std::string& name);
    rw::common::Ptr<rwlibs::swig::JointDevice> findJointDevice(const std::string& name);
    rw::common::Ptr<rwlibs::swig::SerialDevice> findSerialDevice(const std::string& name);
    rw::common::Ptr<rwlibs::swig::TreeDevice> findTreeDevice(const std::string& name);
    rw::common::Ptr<rwlibs::swig::ParallelDevice> findParallelDevice(const std::string& name);
    rwlibs::swig::Frame* findFrame(const std::string& name);

    rwlibs::swig::MovableFrame* findMovableFrame(const std::string& name);

    rwlibs::swig::FixedFrame* findFixedFrame(const std::string& name);

    void moveTo(rwlibs::swig::MovableFrame* mframe, rwlibs::swig::Transform3d wTframe );

    void moveTo(rwlibs::swig::Frame* frame, rwlibs::swig::MovableFrame* mframe, rwlibs::swig::Transform3d wTtcp );

    void moveTo(const std::string& fname, const std::string& mname, rwlibs::swig::Transform3d wTframe );

    // utility functions for
    rwlibs::swig::Q getQ(rw::common::Ptr<rwlibs::swig::Device> dev);
    void setQ(rw::common::Ptr<rwlibs::swig::Device> dev, rwlibs::swig::Q);

    void setTransform(rwlibs::swig::Frame* mframe, rwlibs::swig::Transform3d wTframe );

    rwlibs::swig::Transform3d wTf(rwlibs::swig::Frame* frame);
    rwlibs::swig::Transform3d wTf(const std::string& name);
    rwlibs::swig::Transform3d fTf(rwlibs::swig::Frame* frame,rwlibs::swig::Frame* to);
    rwlibs::swig::Transform3d fTf(const std::string& from,const std::string& to);

    /**
     * @brief add geometry to an existing frame or object with name objName
     * @param frameName
     * @param geom
     */
    void addGeometry( const std::string& objName, rw::geometry::Geometry::Ptr geom );

    /**
     * @brief adds an rigid object to the scene. If a frame with the objName allready exist then
     * the object will not be created.
     * @param objName
     * @param geom
     */
    void addObject( const std::string& objName, rw::geometry::Geometry::Ptr geom );
    void removeObject( const std::string& objName);

}
}

#endif /* REMOTETYPES_HPP_ */
