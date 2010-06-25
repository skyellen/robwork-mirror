
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/kinematics.hpp>

#ifndef RWLIBS_LUA_KINEMATICS_HPP
#define RWLIBS_LUA_KINEMATICS_HPP



namespace rwlibs {
namespace lua {
namespace kinematics {

	class State: public rw::kinematics::State
	{
	public:
		State(const rw::kinematics::State& state);
		State copy();
		unsigned int size() const;
		std::string __tostring() const;
	};

	class Frame
	{
	public:
		Frame(rw::kinematics::Frame* frame);

		math::Transform3D getTransform(const State& state) const;
		int getDOF() const;
		Frame* getParent();
		Frame* getParent(const State& state);
		void attachTo(Frame* parent, State& state);

		bool isDAF();

		math::Transform3D wTt(const State& state) const;
		math::Transform3D tTf(const Frame& frame, const State& state) const;

		const rw::kinematics::Frame* get() const;
		rw::kinematics::Frame* get();

		std::string __tostring() const;

		rw::kinematics::Frame* _frame;
	};

	class FixedFrame: public Frame
	{
	public:
		FixedFrame(rw::kinematics::FixedFrame* frame);

		void setTransform(const math::Transform3D& transform);


		const rw::kinematics::FixedFrame* get() const;
		std::string __tostring() const;

		rw::kinematics::FixedFrame* _fframe;
	};

	class MovableFrame: public Frame
	{
	public:
		// tolua_begin
		MovableFrame(rw::kinematics::MovableFrame* frame);

		void setTransform(const math::Transform3D& transform, State& state);

		const rw::kinematics::MovableFrame* get() const;
		std::string __tostring() const;

		rw::kinematics::MovableFrame* _mframe;
	};

	math::Transform3D frameTframe(const Frame* from, const Frame* to, const State& state);

	math::Transform3D worldTframe(const Frame* to, const State& state);

	Frame worldFrame(Frame& frame, const State& state);

	void gripFrame(State& state, Frame& item, Frame& gripper);

	bool isDAF(const Frame& frame);

}}}


#endif
