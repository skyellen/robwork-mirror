/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#ifndef rw_collision_CollisionSetupLoader_HPP
#define rw_collision_CollisionSetupLoader_HPP

/**
 * @file CollisionSetupLoader.hpp
 */

#include <string>

namespace rw { 
    namespace proximity {
        class CollisionSetup;
    }
    namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Loader of collision setups from files.
     *
     * [CollisionSetupLoader could just as well simply be a utility function.]
     *
     * The XML format is:
\verbatim
      <CollisionSetup>
          <Exclude>
              <FramePair first="first_frame" second="second_frame"/>
              ...
          </Exclude>
      </CollisionSetup>
\endverbatim
     */
    class CollisionSetupLoader
    {
    public:
        /**
         * @brief Load a collision setup from the file \a file.
         *
         * \a prefix is prepended to every frame name.
         *
         * @param prefix [in] The context in which the setup is loaded.
         *
         * @param file [in] The file from which to load the collision setup.
         *
         * @return The collision setup.
         */
        static rw::proximity::CollisionSetup Load(
            const std::string& prefix,
            const std::string& file);

    private:
        CollisionSetupLoader();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
