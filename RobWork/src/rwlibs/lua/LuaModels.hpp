
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

    //! @addtogroup lua
    // @{
    //! @file LuaModels.hpp

    /**
     * @brief lua wrapper class for rw::models::Device
     */
	class Device
	{
	public:
		Device(rw::models::DevicePtr device);

        void setQ(const Q& q, State& state) const;
        Q getQ(const State& state) const;

        Q getAccLimits() const;
        void setAccLimits(const Q& lim);

        Q getVelLimits() const;
        void setVelLimits(const Q& lim);

        Q getMinPosLimits() const;
        Q getMaxPosLimits() const;

        void setPosLimits(const Q& min,const Q& max);

        unsigned int getDOF() const;

        std::string getName() const;
        void setName(const std::string& name);

        Frame getBase();
        const Frame getBase() const;
        Frame getEnd();


        Transform3D bTf(const Frame* f, const State& state) const;
        Transform3D bTe(const State& state) const;
        Transform3D wTb(const State& state) const;

		rw::models::DevicePtr get() const;
		rw::models::DevicePtr _dev;
	};

	/**
	 * @brief lua wrapper class for rw::models::WorkCell
	 */
    class WorkCell // tolua_export
    {
    public:
		// tolua_begin
		WorkCell();
		WorkCell(rw::models::WorkCellPtr wc);

		//! @copydoc rw::models::WorkCell::getName()
        std::string getName() const;

        Frame getWorldFrame() const;

        Frame findFrame(const std::string& name) const;

        Device findDevice(const std::string& name) const;

        State getDefaultState() const;

    	rw::models::WorkCellPtr get() const;
    	rw::models::WorkCellPtr get();

    	rw::models::WorkCellPtr _wc;
    	std::string __tostring() const;
    	 // tolua_end
    };

    // @}

}}


#endif
