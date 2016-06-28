/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_PROXIMITY_PROXIMITYSTRATEGY_HPP
#define RW_PROXIMITY_PROXIMITYSTRATEGY_HPP

/**
 * @file rw/proximity/ProximityStrategy.hpp
 */

#include <string>

#include <rw/kinematics/FrameMap.hpp>

#include "ProximityModel.hpp"

namespace rw { namespace geometry { class Geometry; } }
namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace models { class Object; } }

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief The ProximityStrategy interface is a clean interface
     * for defining methods that are common for different proximity
     * strategy classes. Specifically adding of geometric models and
     * relating them to frames.
     */
    class ProximityStrategy {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<ProximityStrategy> Ptr;

        /**
         * @brief Destructor.
         */
        virtual ~ProximityStrategy();

        /**
         * @brief Adds a Proximity model of a frame to this strategy.
         *
         * The Proximity model is the one specified in the frames property
         *
         * @param frame [in] the frame on which the Proximity model is to be
         * created.
         *
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        //virtual bool addModel(const kinematics::Frame* frame);
        virtual bool addModel(rw::common::Ptr<rw::models::Object> object);

        /**
         * @brief Adds a Proximity model to a frame where the geometry is copied
         * in the underlying proximity strategy.
         *
         * The Proximity model is constructed from the list of faces
         *
         * @param frame [in] the frame to which the Proximity model should associate
         * @param faces [in] list of faces from which to construct the Proximity model
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        virtual bool addModel(
            const rw::kinematics::Frame* frame,
            const rw::geometry::Geometry& faces
            );

        /**
         * @brief Adds a Proximity model to a frame.
         *
         * The Proximity model is constructed from the list of faces
         *
         * @param frame [in] the frame to which the Proximity model should associate
         * @param faces [in] list of faces from which to construct the Proximity model
         * @param forceCopy [in] force the strategy to copy the geometry data, if false the
         * strategy may choose to store the geometry reference or not.
         * @return true if a Proximity model was succesfully created and linked
         * with the frame; false otherwise.
         */
        virtual bool addModel(
            const rw::kinematics::Frame* frame,
            rw::common::Ptr<rw::geometry::Geometry> faces,
            bool forceCopy = false
            );


        /**
         * @brief Tells whether the frame has a proximity model in the strategy
         *
         * To have a proximity model does not means that it is loaded. If a \b GeoID string from
         * which a model can be loaded it returns true as well
         *
         * @param frame [in] the frame to check for1.0/
         * @return true if a model exists or can be created
         */
        virtual bool hasModel(const rw::kinematics::Frame* frame);

        /**
           @brief Clear (remove all) model information for frame \b frame.
         */
        virtual void clearFrame(const rw::kinematics::Frame* frame);

        /**
           @brief Clear (remove all) model information for frame \b frame.
         */
        virtual void clearFrames();

        //// new functions added to support old interface

        /**
         * @brief get the proximitymodel associated to \b frame. If no model
         * has been associated to frame then NULL is returned.
         * @param frame [in] frame for which an proximitymodel is associated
         */
        ProximityModel::Ptr getModel(const rw::kinematics::Frame* frame);

        rw::kinematics::Frame* getFrame(ProximityModel::Ptr);

        //// this is the new interface based on CollisionModelInfo
        /**
         * @brief creates an empty ProximityModel
         */
		virtual ProximityModel::Ptr createModel() = 0;

        /**
         * @brief deallocates the memory used for \b model
         * @param model
         */
        virtual void destroyModel(ProximityModel* model) = 0;

        /**
         * @brief adds geometry to a specific proximity model. The proximity strategy copies all
         * data of the geometry.
         * @param model [in] the proximity model to add data to
         * @param geom [in] the geometry that is to be added
         */
        virtual bool addGeometry(ProximityModel* model, const rw::geometry::Geometry& geom) = 0;

        /**
         * @brief adds geometry to a specific model. Depending on the option \b forceCopy the proximity
         * strategy may choose to copy the geometry data or use it directly.
         * @param model
         * @param geom
         * @param forceCopy
         * @return
         */
        virtual bool addGeometry(ProximityModel* model, rw::common::Ptr<rw::geometry::Geometry> geom, bool forceCopy=false) = 0;

        /**
         * @brief removes a geometry from a specific proximity model
         */
        virtual bool removeGeometry(ProximityModel* model, const std::string& geomId) = 0;

        /**
         * @brief the list of all geometry ids that are associated to
         * the proximity model \b model is returned
         * @param model [in] the model containing the geometries
         * @return all geometry ids associated to the proximity model
         */
        virtual std::vector<std::string> getGeometryIDs(ProximityModel* model) = 0;

        /**
         * @brief Clears any stored model information
         */
        virtual void clear() = 0;

    private:
        ProximityStrategy(const ProximityStrategy&);
        ProximityStrategy& operator=(const ProximityStrategy&);

		rw::kinematics::FrameMap<ProximityModel::Ptr> _frameToModel;

    protected:
        /**
         * @brief Creates object
         */
        ProximityStrategy();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
