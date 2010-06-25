
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
namespace loaders {



	// tolua_begin
    models::WorkCell loadWorkCell(const std::string& filename);
    proximity::CollisionSetup loadCollisionSetup(const std::string& filename);

    // Image
    sensor::Image* loadImage(const std::string& filename);
    void saveAsPGM(sensor::Image* img, const std::string& filename);

    // paths
    void storePath(const trajectory::QPath& path, const std::string& file);
    void storePath(
        const models::WorkCell& workcell,
        const rw::trajectory::StatePath& path,
        const std::string& file);

    void storePath(
        const models::WorkCell& workcell,
        const trajectory::TimedStatePath& path,
        const std::string& file);

    trajectory::QPath* loadQPath(const std::string& file);

    trajectory::StatePath* loadStatePath(
        const models::WorkCell& workcell,
        const std::string& file);

    trajectory::TimedStatePath* loadTimedStatePath(
        const models::WorkCell& workcell,
        const std::string& file);


    // tolua_end
}}}


#endif
