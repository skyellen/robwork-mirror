
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include "LuaKinematics.hpp"
#include <rw/models.hpp>

#ifndef RWLIBS_LUA_MODELS_HPP
#define RWLIBS_LUA_MODELS_HPP


namespace rwlibs {
namespace lua {
namespace models {

	class Device
	{
	public:
		Device(rw::models::DevicePtr device);

        void setQ(const math::Q& q, kinematics::State& state) const;
        math::Q getQ(const kinematics::State& state) const;

        math::Q getAccLimits() const;
        void setAccLimits(const math::Q& lim);

        math::Q getVelLimits() const;
        void setVelLimits(const math::Q& lim);

        math::Q getMinPosLimits() const;
        math::Q getMaxPosLimits() const;

        void setPosLimits(const math::Q& min,const math::Q& max);

        unsigned int getDOF() const;

        std::string getName() const;
        void setName(const std::string& name);

        kinematics::Frame getBase();
        const kinematics::Frame getBase() const;
        kinematics::Frame getEnd();


        math::Transform3D bTf(
            const kinematics::Frame* f, const kinematics::State& state) const;
        math::Transform3D bTe(const kinematics::State& state) const;
        math::Transform3D wTb(const kinematics::State& state) const;

		rw::models::DevicePtr get() const;
		rw::models::DevicePtr _dev;
	};

    class WorkCell // tolua_export
    {
    public:
		// tolua_begin
		WorkCell();
		WorkCell(rw::models::WorkCellPtr wc);


        std::string getName() const;

        kinematics::Frame getWorldFrame() const;

        kinematics::Frame findFrame(const std::string& name) const;

        Device findDevice(const std::string& name) const;

        kinematics::State getDefaultState() const;

    	rw::models::WorkCellPtr get() const;
    	rw::models::WorkCellPtr get();

    	rw::models::WorkCellPtr _wc;
    	std::string __tostring() const;
    	 // tolua_end
    };


}}}


#endif
