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

#ifndef rwlibs_proximitystrategies_ProximityStrategyPQP_HPP
#define rwlibs_proximitystrategies_ProximityStrategyPQP_HPP

/**
 * @file DistanceStrategyPQP.hpp
 */

#include <map>
#include <vector>
#include <list>

#include <boost/shared_ptr.hpp>

#include <rw/common/Cache.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/geometry/Face.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>
#include <rw/proximity/DistanceToleranceStrategy.hpp>

namespace PQP { class PQP_Model; }

namespace rwlibs { namespace proximitystrategies {
    /** @addtogroup proximitystrategies */
    /*@{*/

    /**
     * @brief This is a strategy wrapper for the distance library
     * PQP (Proximity Query Package).
     *
     * PQP use Oriented Bounding Boxes (OBB) and hierarchical bounding trees for
     * fast distance calculation.
     *
     * For further information check out http://www.cs.unc.edu/~geom/SSV/
     */
    class ProximityStrategyPQP :
        public rw::proximity::CollisionStrategy,
        public rw::proximity::CollisionToleranceStrategy,
        public rw::proximity::DistanceStrategy,
        public rw::proximity::DistanceToleranceStrategy
    {
    public:
        typedef rw::common::Ptr<PQP::PQP_Model> SharedModel;
    	typedef std::pair<rw::math::Transform3D<>, SharedModel> ColModel;
    	typedef std::vector<ColModel> ModelList;
        typedef std::map< const rw::kinematics::Frame*, ModelList> FrameModelMap;
        typedef std::pair<ColModel, ColModel> ModelPair;
    private:
        FrameModelMap _frameModelMap;
        bool _firstContact;

        const ModelList& getPQPModels(const rw::kinematics::Frame* frame);
        rw::common::Cache<std::string, PQP::PQP_Model> _modelCache;

    public:
        /**
         * @brief Constructor
         */
        ProximityStrategyPQP();

        /*
         * @copydoc rw::proximity::ProximityStrategy::addModel
         */
        bool addModel(const rw::kinematics::Frame* frame);

        /*
         * @copydoc rw::proximity::ProximityStrategy::addModel
         */
        bool addModel(
            const rw::kinematics::Frame* frame,
            const std::vector<rw::geometry::Face<float> >& faces);

        /**
         * @copydoc rw::proximity::ProximityStrategy
         */
        bool hasModel(const rw::kinematics::Frame* frame);

        /**
         * @copydoc rw::proximity::CollisionStrategy::setFirstContact
         */
        void setFirstContact(bool b);

        /**
         * @copydoc rw::proximity::CollisionStrategy::inCollision
         */
        bool inCollision(
            const rw::kinematics::Frame* a,
            const rw::math::Transform3D<>& wTa,
            const rw::kinematics::Frame* b,
            const rw::math::Transform3D<>& wTb);

        /**
         * @copydoc rw::proximity::CollisionToleranceStrategy::inCollision
         */
        bool inCollision(
            const rw::kinematics::Frame* a,
            const rw::math::Transform3D<>& wTa,
            const rw::kinematics::Frame* b,
            const rw::math::Transform3D<>& wTb,
            double tolerance);

        /**
         * @copydoc rw::proximity::DistanceStrategy::distance
         */
        bool distance(
            rw::proximity::DistanceResult &result,
            const rw::kinematics::Frame* a,
            const rw::math::Transform3D<>& wTa,
            const rw::kinematics::Frame* b,
            const rw::math::Transform3D<>& wTb,
            double rel_err = 0.0,
            double abs_err = 0.0);

        /**
         * @copydoc rw::proximity::DistanceToleranceStrategy::getDistances
         */
        bool getDistances(
            rw::proximity::MultiDistanceResult &result,
            const rw::kinematics::Frame* a,
            const rw::math::Transform3D<>& wTa,
            const rw::kinematics::Frame* b,
            const rw::math::Transform3D<>& wTb,
            double tolerance,
            double rel_err = 0.0,
            double abs_err = 0.0);

        /**
         *  @copydoc rw::proximity::ProximityStrategy::clear
         */
        void clear();

        /**
           @copydoc rw::proximity::ProximityStrategy::clearFrame
         */
        void clearFrame(const rw::kinematics::Frame* frame);

        /**
           @brief A PQP based collision strategy.
        */
        static rw::proximity::CollisionStrategyPtr make();
    };

}} // end namespaces

#endif // end include guard
