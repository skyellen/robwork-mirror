#ifndef rw_collision_ProximityCommon_HPP_
#define rw_collision_ProximityCommon_HPP_

#include <list>

#include <rw/common/Exception.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>

#include <string>

namespace rw { namespace proximity {

	/** @addtogroup proximity */
	/*@{*/

    /**
     * @brief A pair of frames
     */
    typedef std::pair<kinematics::Frame*, kinematics::Frame*> FramePair;
    
    /**
     * @brief A list of frame pairs
     */
    typedef std::list<FramePair> FramePairList;
    
    /**
     * @brief A pair of frame names
     */
    typedef std::pair<std::string, std::string> ProximityPair;

    /**
     * @brief A list of pairs for with ProximityPairs
     */ 
    typedef std::vector<ProximityPair> ProximityPairList;
    
    /**
     * @brief ProximityCommon implements a some utility functions which are
     * common to different classes in the collision library
     */ 
    class ProximityCommon {
    public:
        /**
         * @brief lookupFrame checks if frame is in the FrameMap.
         * If not, the method throws an exception.
         * 
         * @param frameMap [in] is the map
         * @param frameName [in] is the name of the frame to look up
         * 
         * @return reference to the frame
         */
        static kinematics::Frame& lookupFrame(
            const kinematics::Kinematics::FrameMap& frameMap, 
            const std::string& frameName)
        {
            const kinematics::Kinematics::FrameMap::const_iterator pos =
                frameMap.find(frameName);
            if (pos == frameMap.end())
                RW_THROW(
                    "Frame "
                    << rw::common::StringUtil::Quote(frameName)
                    << " is not present in frame map.");

            return *pos->second;
        }
    };
    
	/*@}*/
}} // end namespaces

#endif /*rw_collision_ProximityCommon_HPP_*/
