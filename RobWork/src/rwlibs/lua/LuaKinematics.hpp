#ifndef RWLIBS_LUA_KINEMATICS_HPP
#define RWLIBS_LUA_KINEMATICS_HPP



/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/kinematics.hpp>


namespace rwlibs {
namespace lua {

    //! @addtogroup lua
    // @{

    //! @file LuaKinematics.hpp

    /**
     * @brief Lua wrapper class to rw::kinematics::State
     */
	class State: public rw::kinematics::State
	{
	public:
	    //! @brief constructor, takes a copy of rw::kinematics::State
		State(const rw::kinematics::State& state);
		//! @brief makes a copy
		State copy();
		//! @brief get size of state
		unsigned int size() const;
		//! @brief info of this object
		std::string __tostring() const;
	};

    /**
     * @brief Lua wrapper class to rw::kinematics::Frame
     */
	class Frame
	{
	public:
	    //! @brief constructor
		Frame(rw::kinematics::Frame* frame);

		//! get transform
		Transform3D getTransform(const State& state) const;
		//
		int getDOF() const;
		Frame* getParent();
		Frame* getParent(const State& state);
		void attachTo(Frame* parent, State& state);

		bool isDAF();

		Transform3D wTt(const State& state) const;
		Transform3D tTf(const Frame& frame, const State& state) const;

		const rw::kinematics::Frame* get() const;
		rw::kinematics::Frame* get();

		std::string __tostring() const;

		rw::kinematics::Frame* _frame;
	};

    /**
     * @brief Lua wrapper class to rw::kinematics::FixedFrame
     */
	class FixedFrame: public Frame
	{
	public:
		FixedFrame(rw::kinematics::FixedFrame* frame);

		void setTransform(const Transform3D& transform);


		const rw::kinematics::FixedFrame* get() const;
		std::string __tostring() const;

		rw::kinematics::FixedFrame* _fframe;
	};

    /**
     * @brief Lua wrapper class to rw::kinematics::MovableFrame
     */
	class MovableFrame: public Frame
	{
	public:
	    //! @brief constructor
		MovableFrame(rw::kinematics::MovableFrame* frame);

		/**
		 * @brief set the transform of this movableframe
		 * @param transform [in] new transform
		 * @param state [in] the state to set the transform in
		 */
		void setTransform(const Transform3D& transform, State& state);

		//! @brief get the rw::kinematics::MovableFrame
		const rw::kinematics::MovableFrame* get() const;

		//! @brief info of this object
		std::string __tostring() const;

		//! @brief the movable frame
		rw::kinematics::MovableFrame* _mframe;
	};

	/**
	 * @brief Lua wrapper function to rw::kinematics::Kinematics::frameTframe
	 */
	Transform3D frameTframe(const Frame* from, const Frame* to, const State& state);

    /**
     * @brief Lua wrapper function to rw::kinematics::Kinematics::worldTframe
     */
	Transform3D worldTframe(const Frame* to, const State& state);

    /**
     * @brief Lua wrapper function to rw::kinematics::Kinematics::worldFrame
     */
	Frame worldFrame(Frame& frame, const State& state);

    /**
     * @brief Lua wrapper function to rw::kinematics::Kinematics::gripFrame
     */
	void gripFrame(State& state, Frame& item, Frame& gripper);

    /**
     * @brief Lua wrapper function to rw::kinematics::Kinematics::isDAF
     */
	bool isDAF(const Frame& frame);

	// @}

}}


#endif
