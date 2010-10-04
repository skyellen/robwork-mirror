
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/trajectory.hpp>

#ifndef RWLIBS_LUA_TRAJECTORY_HPP
#define RWLIBS_LUA_TRAJECTORY_HPP


namespace rwlibs {
namespace lua {

    //! @addtogroup lua
    // @{

    typedef rw::trajectory::QPath QPath;
    typedef rw::trajectory::TimedQ TimedQ;
    typedef rw::trajectory::TimedQPath TimedQPath;

    typedef rw::trajectory::StatePath StatePath;

    typedef rw::trajectory::TimedState TimedState;
    typedef rw::trajectory::TimedStatePath TimedStatePath;

    /**
     * @brief lua wrapper class for rw::trajectory::QPath
     */
	/*class QPath: public rw::trajectory::QPath
	{
	public:
	    QPath();
		QPath(const rw::trajectory::QPath& path);
	};
	*/

//	typedef rw::trajectory::Timed<rw::math::Q> TimedQ;
	//class TimedQ {
	//public:
	//    TimedQ(rw::trajectory::Timed<rw::math::Q>& value);
	//    TimedQ(double time, Q q);

	//    double& getTime();
	//    Q& getQ();

	    //rw::trajectory::Timed<rw::math::Q>& get();
    //    std::string __tostring() const;

    //    rw::trajectory::Timed<Q> _value;
	//};

    //class TimedQPath
    //{
    //public:
    //    TimedQPath();

    //    void add(double time, rwlibs::lua::Q value);
    //    TimedQ& operator[] (int index);

    //    int size();

        //rw::trajectory::Timed<rw::math::Q>& get();
        //std::string __tostring() const;

    //    std::vector<TimedQ> _path;
    //};

	//void rw::trajectory::TimedQPath::add(double time, rwlibs::lua::Q value);

    /**
     * @brief lua wrapper class for rw::trajectory::StatePath
     */
	/*

    class StatePath: public rw::trajectory::StatePath
	{
	public:
	    StatePath();
		StatePath(const rw::trajectory::StatePath& path);
	};
    */
    /**
     * @brief lua wrapper class for rw::trajectory::TimedStatePath
     */
    /*
	class TimedStatePath: public rw::trajectory::TimedStatePath
	{
	public:
		TimedStatePath(const rw::trajectory::TimedStatePath& path);
	};
	*/

	// @}

}}


#endif
