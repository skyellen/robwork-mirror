
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include "LuaModels.hpp"
#include "LuaSensor.hpp"
#include "LuaTrajectory.hpp"
#include "LuaModels.hpp"
#include "LuaProximity.hpp"

#include <rw/loaders.hpp>

#ifndef RWLIBS_LUA_LOADERS_HPP
#define RWLIBS_LUA_LOADERS_HPP



namespace rwlibs {
namespace lua {

    //! @addtogroup lua
    // @{

	// tolua_begin
    //! @brief load a workcell
    WorkCell loadWorkCell(const std::string& filename);

    //! @brief load a collision setup file
    CollisionSetup loadCollisionSetup(const std::string& filename);

    // Image
    //! @brief load an image from file
    Image* loadImage(const std::string& filename);
    //! @brief save an image in PGM format
    void saveAsPGM(Image* img, const std::string& filename);

    // paths
    //! @brief save a trajectory to a file
    void storePath(const QPath& path, const std::string& file);

    //! @brief save a state path to a file
    void storePath(
        const WorkCell& workcell,
        const rw::trajectory::StatePath& path,
        const std::string& file);

    //! @brief save a timed state path to a file
    void storePath(
        const WorkCell& workcell,
        const TimedStatePath& path,
        const std::string& file);

    //! @brief load a qpath from file
    QPath* loadQPath(const std::string& file);

    //! @brief load a statepath from file
    StatePath* loadStatePath(
        const WorkCell& workcell,
        const std::string& file);

    //! @brief load a timed state path from file
    TimedStatePath* loadTimedStatePath(
        const WorkCell& workcell,
        const std::string& file);


    // tolua_end

    // @}
}}


#endif
