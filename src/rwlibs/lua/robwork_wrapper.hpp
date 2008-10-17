/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rwlibs_lua_robwork_wrapper_hpp
#define rwlibs_lua_robwork_wrapper_hpp

/*
  This file is for internal use only and is of no use to no-one.
*/

#include <iostream>

#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QToTPlanner.hpp>
#include <rw/trajectory/Path.hpp>

#include <rw/kinematics/State.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include <string>

namespace rwlibs { namespace lua { class Output; }}

namespace rwlibs { namespace lua { namespace internal {

    //----------------------------------------------------------------------
    // Math
    //----------------------------------------------------------------------

    class Q // tolua_export
    {
    public:
        // tolua_begin
        Q(int n, double* vals);
        bool operator==(const Q& q);
        std::string __tostring() const;
        // tolua_end

        Q() {}

        Q(const rw::math::Q& q) : _q(q) {}
        const rw::math::Q& get() const { return _q; }

    private:
        rw::math::Q _q;
    };

    class Vector3D // tolua_export
    {
    public:
        // tolua_begin
        Vector3D(double vals[3]);
        std::string __tostring() const;

        Vector3D operator*(double scale) const;
        Vector3D operator+(const Vector3D& other) const;
        Vector3D operator-(const Vector3D& other) const;
        bool operator==(const Vector3D& v);

        // tolua_end

        Vector3D(const rw::math::Vector3D<>& vec) : _vector(vec) {}
        const rw::math::Vector3D<>& get() const;

    private:
        rw::math::Vector3D<> _vector;
    };

    class Rotation3D // tolua_export
    {
    public:
        // tolua_begin
        Rotation3D(double vals[9]);
        std::string __tostring() const;

        Rotation3D operator*(const Rotation3D& rot) const;
        Vector3D operator*(const Vector3D& vec) const;
        Rotation3D inverse() const;

        // tolua_end

        Rotation3D(const rw::math::Rotation3D<>& rotation) :
            _rotation(rotation) {}

        const rw::math::Rotation3D<>& get() const;

    private:
        rw::math::Rotation3D<> _rotation;
    };

    class Transform3D // tolua_export
    {
    public:
        // tolua_begin
        Transform3D(
            const Vector3D& position,
            const Rotation3D& rotation);

        Transform3D operator*(const Transform3D& other) const;
        Vector3D operator*(const Vector3D& other) const;
        Transform3D inverse() const;

        // Position and orientation.
        Vector3D p() const;
        Vector3D P() const;
        Rotation3D r() const;
        Rotation3D R() const;

        std::string __tostring() const;
        // tolua_end

        Transform3D(const rw::math::Transform3D<>& transform) :
            _transform(transform) {}

        const rw::math::Transform3D<>& get() const;

    private:
        rw::math::Transform3D<> _transform;
    };

    //----------------------------------------------------------------------
    // Kinematics
    //----------------------------------------------------------------------

    class State // tolua_export
    {
    public:
        // tolua_begin
        State copy();

        // tolua_end

        State() {}

        State(
            const rw::kinematics::State& state)
            :
            _state(state)
        {}

        rw::kinematics::State& get() { return _state; }
        const rw::kinematics::State& get() const { return _state; }

    private:
        rw::kinematics::State _state;
    };

    class Frame // tolua_export
    {
    public:
        // tolua_begin
        Frame(rw::kinematics::Frame* frame);

        std::string __tostring() const;

        // The transform in the world frame.
        Transform3D wt(const State& state) const;

        // The transform to the other frame.
        Transform3D to(const Frame& frame, const State& state) const;

        // An error message (possibly empty) is returned.
        std::string attachFrame(Frame& parent, State& state);

        // tolua_end

        rw::kinematics::Frame& get() { return *_frame; }
        const rw::kinematics::Frame& get() const { return *_frame; }

    private:
        rw::kinematics::Frame* _frame;
    };

    //----------------------------------------------------------------------
    // Models
    //----------------------------------------------------------------------

    class Device // tolua_export
    {
    public:
        // tolua_begin
        Device(rw::models::DevicePtr device);
        void setQ(const Q& q, State& state) const;
        Q getQ(const State& state) const;

        Frame getBase();
        Frame getEnd();

        std::string __tostring() const;
        // tolua_end

        // Default constructor so that we can have arrays.
        Device() {}
        rw::models::Device& get() { return *_device; }
        const rw::models::Device& get() const { return *_device; }

        rw::models::DevicePtr getPtr() { return _device; }
        const rw::models::DevicePtr getPtr() const { return _device; }

    private:
        rw::models::DevicePtr _device;
    };

    class WorkCell // tolua_export
    {
    public:
        // tolua_begin
        State getDefaultState() const;

        std::string __tostring() const;

        // A workcell has been loaded.
        bool internal_has() const;

        // The error message if the loading of the workcell failed.
        const std::string& internal_getErrorMessage() const;

        Frame getWorldFrame() const;

        // tolua_end

        WorkCell(rw::models::WorkCellPtr workcell);

        WorkCell(const std::string& errorMessage);

        rw::models::WorkCell& get() { return *_workcell; }
        const rw::models::WorkCell& get() const { return *_workcell; }

        rw::models::WorkCellPtr getPtr() { return _workcell; }
        const rw::models::WorkCellPtr getPtr() const { return _workcell; }

    private:
        rw::models::WorkCellPtr _workcell;
        std::string _errorMessage;
    };

    //----------------------------------------------------------------------
    // Other
    //----------------------------------------------------------------------

    class Output // tolua_export
    {
    public:
        // tolua_begin

        // This is the value of a global Lua variable that has been written by a
        // C registry method on class RobWork. The rw wrapper Lua module takes
        // care that the right global value is passed to the constructor always.
        Output(void* userdata);

        void write(const std::string& txt);

        // tolua_end

    private:
        rwlibs::lua::Output* _output;
    };

    /*
      I believe we will need a more structured (powerful) representation for
      paths that just a vector<State>. Here are some steps in that direction to
      try out on the Lua side:
    */
    class Path // tolua_export
    {
    public:
        // tolua_begin

        Path(int len, State* states);

        int size() const; // You do have to use int here.
        bool empty() const;

        // Concatenation (non-clever: We don't check if start and end are equal):
        Path operator+(const Path& other) const;

        // The last state on the path (if non-empty).
        State getEndState() const;

        // tolua_end

        Path(const rw::trajectory::StatePath& path) :
            _path(path)
        {}

        const rw::trajectory::StatePath& get() const { return _path; }

    private:
        rw::trajectory::StatePath _path;
    };

    class CollisionDetector // tolua_export
    {
    public:
        // tolua_begin

        bool inCollision(const State& state) const;

        // tolua_end

        rw::proximity::CollisionDetector& get() { return *_detector; }
        const rw::proximity::CollisionDetector& get() const { return *_detector; }

        rw::proximity::CollisionDetectorPtr getPtr() { return _detector; }
        const rw::proximity::CollisionDetectorPtr getPtr() const { return _detector; }

        CollisionDetector(
            rw::proximity::CollisionDetectorPtr detector)
            : _detector(detector)
        {}

    private:
        rw::proximity::CollisionDetectorPtr _detector;
    };

    class CollisionStrategy // tolua_export
    {
    public:
        // tolua_begin

        // tolua_end

        rw::proximity::CollisionStrategy& get() { return *_strategy; }
        const rw::proximity::CollisionStrategy& get() const { return *_strategy; }

        rw::proximity::CollisionStrategyPtr getPtr() { return _strategy; }
        const rw::proximity::CollisionStrategyPtr getPtr() const { return _strategy; }

        CollisionStrategy(
            rw::proximity::CollisionStrategyPtr strategy)
            : _strategy(strategy)
        {}

    private:
        rw::proximity::CollisionStrategyPtr _strategy;
    };

    class PathPlanner // tolua_export
    {
    public:
        // tolua_begin

        Path query(
            const Q& from,
            const Q& to);

        Path query(
            const Q& from,
            const Transform3D& to);

        // tolua_end

        PathPlanner(
            rw::pathplanning::QToQPlannerPtr toQ,
            rw::pathplanning::QToTPlannerPtr toT,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state)
            :
            _toQ(toQ),
            _toT(toT),
            _device(device),
            _state(state)
        {}

    private:
        rw::pathplanning::QToQPlannerPtr _toQ;
        rw::pathplanning::QToTPlannerPtr _toT;
        rw::models::DevicePtr _device;
        rw::kinematics::State _state;
    };

    //----------------------------------------------------------------------
    // Utility functions.

    // tolua_begin

    Rotation3D rpy(double roll, double pitch, double yank);
    Rotation3D eaa(double x, double y, double z);
    WorkCell loadWorkCell(const std::string& file);

    // Construct a workcell (a reference) from a void pointer to WorkCell.
    WorkCell makeWorkCell(void* userdata);

    // Construct a CollisionDetector from a void pointer to a robwork::CollisionDetector.
    CollisionDetector makeCollisionDetector(void* userdata);

    CollisionDetector makeCollisionDetectorFromStrategy(
        WorkCell& workcell, CollisionStrategy& strategy);

    // Construct a CollisionStrategy from a void pointer to a robwork::CollisionStrategy.
    CollisionStrategy makeCollisionStrategy(void* userdata);

    // Construct a state (a copy) from a void pointer to State.
    State makeState(void* userdata);

    // Write a state to a pointer to a state.
    void writeState(void* userdata, const State& state);

    // Store a state path to file. The workcell is used for assigning time
    // stamps for the stored states.
    void storeStatePath(
        const WorkCell& workcell, const Path& path, const std::string& file);

    rw::kinematics::Frame* findFrame(
        const WorkCell& workcell, const std::string& name);

    rw::models::Device* findDevice(
        WorkCell& workcell, const std::string& name);

    // An error message is returned or the empty string if no error.
    std::string gripFrame(State& state, Frame& item, Frame& gripper);

    Rotation3D inverse(const Rotation3D& val);
    Transform3D inverse(const Transform3D& val);

    Device makeCompositeDevice(
        const std::string& name,
        Frame& base,
        int len,
        Device* devices,
        Frame& end,
        const State& state);

    Device makeCompositeDevice(
        const std::string& name,
        int len,
        Device* devices,
        const State& state);

    PathPlanner makePathPlanner(
        Device& device,
        Frame& tcp,
        const State& state,
        CollisionDetector& detector,
        void* pathPlannerFactory);

    // tolua_end

}}} // end namespaces

#endif // end include guard
