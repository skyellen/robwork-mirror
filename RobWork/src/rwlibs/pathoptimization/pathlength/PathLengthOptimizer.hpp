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


#ifndef RWLIBS_PATHOPTIMIZATION_PATHLENGTHOPTIMIZER_HPP
#define RWLIBS_PATHOPTIMIZATION_PATHLENGTHOPTIMIZER_HPP

#include <rw/math/Metric.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/trajectory/Path.hpp>
#include <list>

namespace rwlibs { namespace pathoptimization {

    /** @addtogroup pathoptimization */
    /*@{*/

    /**
     * @brief The PathLengthOptimizer implements the 3 different path length optimizers
     * presented in [1].
     *
     * [1]: R. Geraerts and M.H. Overmars, Creating High-Quality Paths for Motion Planning,
     * The International Journal of Robotics Research, Vol. 26, No. 8, 845-863 (2007)
     *
     * The simplest algorithm \b pathPruning runs through the path an tests if nodes with
     * index i and i+2 can be directly connected. If so it removed node i+1.
     *
     * The \b shortCut algorithm works similary except that it takes two random indices
     * i and j and tries to connect those. This algorithm is non-deterministic but more
     * powerful than pathPruning.
     *
     * The \b partialShortCut algorithm select two random node indices i and j and a random
     * position in the configuration vector. A shortcut is then only tried between the values
     * corresponding to the random position. This algorithm is generally more powerful than
     * shortCut but may in some cases be more computational expensive.
     */
    class PathLengthOptimizer
    {
    public:
        //! A list of configurations.
		typedef std::list<rw::math::Q> QList;

        /**
           @brief Constructor

           Path optimization is based on a constraint for edges together with a
           distance metric to measure whether the path modifications decrease
           the length of the path.

           @param constraint [in] Verification of edges and configurations.
           @param metric [in] Distance metric for edge lengths
        */
        PathLengthOptimizer(const rw::pathplanning::PlannerConstraint& constraint,
			rw::math::QMetric::CPtr metric);

        /**
           @brief Destructor
        */
        virtual ~PathLengthOptimizer();

        /**
           @brief Optimizes using path pruning.

           \b pathPruning runs through the path an tests if nodes with
           index i and i+2 can be directly connected. If so it removes node i+1.

           @param path [in] Path to optimize
        */
        rw::trajectory::QPath pathPruning(const rw::trajectory::QPath& path);

        /**
         * @brief Optimizes using the shortcut technique
         *
         * The \b shortCut algorithm works by selecting two random indices i and j and
         * try to connect those.
         *
         * The algorithm will loop until either the specified \b cnt is of met or the specified
         * time is reached.
         *
         * @param path [inout] Path to optimize
         * @param cnt [in] Max count to use. If cnt=0, only the time limit will be used
         * @param time [in] Max time to use (in seconds). If time=0, only the cnt limit will be used
         * @param subDivideLength [in] The length into which the path is subdivided
         */
        rw::trajectory::QPath shortCut(const rw::trajectory::QPath& path,
                                       size_t cnt,
                                       double time,
                                       double subDivideLength);

        /**
         * @brief Optimizes using the shortcut technique
         *
         * Works similar to shortCut(const rw::pathplanning::Path&, size_t,
         * double, double) except that parameters are read from the propertymap.
         *
         * @param path [inout] Path to optimize
         */
        rw::trajectory::QPath shortCut(const rw::trajectory::QPath& path);

        /**
         * @brief Optimizes using the partial shortcut technique
         *
         * The \b partialShortCut algorithm select two random node indices i and j and a random
         * position in the configuration vector. A shortcut is then only tried between the values
         * corresponding to the random position.
         *
         * The algorithm will loop until either the specified \b cnt is of met or the specified
         * time is reached.
         *
         * @param path [inout] Path to optimize
         * @param cnt [in] Max count to use. If cnt=0, only the time limit will be used
         * @param time [in] Max time to use (in seconds). If time=0, only the cnt limit will be used
         * @param subDivideLength [in] The length into which the path is subdivided
         */
        rw::trajectory::QPath partialShortCut(const rw::trajectory::QPath& path,
                                              size_t cnt,
                                              double time,
                                              double subDivideLength);

        /**
         * @brief Optimizes using the partial shortcut technique
         *
         * Works similar to partialShortCut(const rw::pathplanning::Path&,
         * size_t, double, double) except that parameters are read from the
         * propertymap.
         *
         * @param path [inout] Path to optimize
         * @return The optimized path
         */
        rw::trajectory::QPath partialShortCut(const rw::trajectory::QPath& path);

        //----------------------------------------------------------------------

        /**
         * @brief Returns the propertymap
         * @return Reference to the property map
         */
        rw::common::PropertyMap& getPropertyMap();

        //!Property key for the maximal number of loops
        static const std::string PROP_LOOPCOUNT;

        //!Property key for max time
        static const std::string PROP_MAXTIME;

        //!Property key for length of segment in when subdividing
        static const std::string PROP_SUBDIVLENGTH;

    private:
        rw::pathplanning::PlannerConstraint _constraint;
		rw::math::QMetric::CPtr _metric;
        rw::common::PropertyMap _propertyMap;

        void pathPruning(QList& path);

        void shortCut(QList& path,
                      size_t cnt,
                      double time,
                      double subDivideLength);

        void shortCut(QList& path);

        void partialShortCut(QList& path,
                             size_t cnt,
                             double time,
                             double subDivideLength);

        void partialShortCut(QList& path);

        void resamplePath(QList& path, double subDivisionSize);

        QList::iterator resample(QList::iterator it1,
                                 const rw::math::Q& q2,
                                 double subDivisionSize,
                                 QList& result);

        bool validPath(const rw::math::Q& from, const rw::math::Q& to);

        bool _testQStart;
        void setTestQStart(bool value) { _testQStart = value; }

        bool _testQEnd;
        void setTestQEnd(bool value) { _testQEnd = value; }
    };

    /** @} */

}} // end namespaces

#endif /*RWLIBS_PATHOPTIMIZATION_PATHLENGTHOPTIMIZER_HPP*/
