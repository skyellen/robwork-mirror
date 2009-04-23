/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_PROXIMITY_PROXIMITYSTRATEGY_HPP
#define RW_PROXIMITY_PROXIMITYSTRATEGY_HPP

/**
 * @file ProximityStrategy.hpp
 */

#include <string>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/geometry/Face.hpp>

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief The ProximityStrategy interface is a clean interface
     * for defining methods that are common for different proximity
     * strategy classes
     *
     *  A collision model is associated with an integer collision ID, which again is
     *  associated with a frame if needed.
     */
    class ProximityStrategy {
    public:
        /**
         * @brief Destructor.
         */
        virtual ~ProximityStrategy();

        /**
         * @brief Adds a Proximity model to the strategy and associates it with a frame
         *
         * @param frame [in] the frame on which the Proximity model is to be
         * created.
         *
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        virtual int addModel(const ProximityModelInfo& model, rw::kinematics::Frame* frame) = 0;

        /**
         * @brief Adds a Proximity model with id \b id and associates it with a frame if specified
         *
         * The Proximity model is constructed from the triangle mesh
         *
         * @param frame [in] the frame to which the Proximity model should associate
         * @param faces [in] list of faces from which to construct the Proximity model
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        virtual int addModel(const rw::geometery::TriMesh& mesh, std::string id, const rw::kinematics::Frame* frame=NULL) = 0;

        /**
         * @brief Tells whether the frame has a proximity model in the strategy
         *
         * To have a proximity model does not means that it is loaded. If a \b GeoID string from
         * which a model can be loaded it returns true as well
         *
         * @param frame [in] the frame to check for
         * @return true if a model exists or can be created
         */
        virtual bool hasModel(const rw::kinematics::Frame* frame) = 0;


        virtual std::vector<int> getCollisionModelIDs(rw::kinematics::Frame* frame) = 0;

        virtual int getCollisionModelID(const std::string& stringId) = 0;

        /**
         * @brief Clears any stored model information
         */
        virtual void clear() = 0;

        /**
           @brief Clear (remove all) model information for frame \b frame.
         */
        virtual void clearFrame(const rw::kinematics::Frame* frame) = 0;

    private:
        ProximityStrategy(const ProximityStrategy&);
        ProximityStrategy& operator=(const ProximityStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        ProximityStrategy();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
