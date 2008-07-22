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

#ifndef rwlibs_proximitystrategies_ProximityStrategyYaobi_HPP
#define rwlibs_proximitystrategies_ProximityStrategyYaobi_HPP

/**
 * @file DistanceStrategyPQP.hpp
 */

#include <map>
#include <vector>
#include <list>

#include <yaobi/yaobi.h>

#include <boost/shared_ptr.hpp>

#include <rw/common/Cache.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/geometry/Face.hpp>
#include <rw/proximity/CollisionStrategy.hpp>

namespace rwlibs { namespace proximitystrategies {
    /** @addtogroup proximitystrategies */
    /*@{*/

    /**
     * @brief This is a strategy wrapper for the collision library
     * Yaobi.
     *
     * Yaobi use Oriented Bounding Boxes (OBB) and hierachical bounding trees for
     * fast collision detection between triangulated objects.
     *
     * For further information check out http://sourceforge.net/projects/yaobi
     */
    class ProximityStrategyYaobi:
        public rw::proximity::CollisionStrategy
    {
    public:
    	typedef boost::shared_ptr<yaobi::CollModel> SharedModel;
    	typedef std::pair<rw::math::Transform3D<>, SharedModel > ColModel;
    	typedef std::vector<ColModel> SharedModelList;

    private:
        typedef std::map< const rw::kinematics::Frame* , SharedModelList > FrameModelMap;
        FrameModelMap _frameModelMap;
        bool _firstContact;

        const SharedModelList& getModels(const rw::kinematics::Frame* frame);
        rw::common::Cache<std::string, yaobi::CollModel> _modelCache;

    public:
        /**
         * @brief Constructor
         */
        ProximityStrategyYaobi();

        /**
         * @brief Deconstructor
         */
        virtual ~ProximityStrategyYaobi();


        /*
         * @copydoc rw::proximity::ProximityStrategy::addModel
         */
        bool addModel(const rw::kinematics::Frame *frame);

        /*
         * @copydoc rw::proximity::ProximityStrategy::addModel
         */
        bool addModel(const rw::kinematics::Frame *frame,
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
        bool inCollision(const rw::kinematics::Frame *a,
                         const rw::math::Transform3D<>& wTa,
                         const rw::kinematics::Frame *b,
                         const rw::math::Transform3D<>& wTb);

        /**
         *  @copydoc rw::proximity::ProximityStrategy::clear
         */
        void clear();

        /**
         *  @copydoc rw::proximity::ProximityStrategy::clearFrame
         */
        void clearFrame(const rw::kinematics::Frame* frame);

        /**
           @brief A Yaobi based collision strategy.
        */
        static std::auto_ptr<rw::proximity::CollisionStrategy> make();
    };

}} // end namespaces

#endif // end include guard
