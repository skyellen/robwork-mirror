
#ifndef RWLIBS_PATHOPTIMIZATION_PATHLENGTHOPTIMIZER_HPP
#define RWLIBS_PATHOPTIMIZATION_PATHLENGTHOPTIMIZER_HPP

#include <rw/math/Metric.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/trajectory/Path.hpp>

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
        /**
           @brief Constructor

           Path optimization is based on a constraint for edges together with a
           distance metric to measure whether the path modifications decrease
           the length of the path.

           @param constraint [in] Verification of edges and configurations.
           @param metric [in] Distance metric for edge lengths
        */
        PathLengthOptimizer(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::math::QMetricPtr metric);

        /**
         * @brief Destructor
         */
        virtual ~PathLengthOptimizer();

        /**
         * @brief Optimizes using path pruning.
         *
         * \b pathPruning runs through the path an tests if nodes with
         * index i and i+2 can be directly connected. If so it removed node i+1.
         *
         * @param path [in] Path to optimize
         * @return The optimized path
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
         * @param path [in] Path to optimize
         * @param cnt [in] Max count to use. If cnt=0, only the time limit will be used
         * @param time [in] Max time to use (in seconds). If time=0, only the cnt limit will be used
         * @param subDivideLength [in] The length into which the path is subdivided
         * @return The optimized path
         */
        rw::trajectory::QPath shortCut(
            const rw::trajectory::QPath& path,
            size_t cnt,
            double time,
            double subDivideLength);

        /**
         * @brief Optimizes using the shortcut technique
         *
         * Works similar to shortCut(const rw::pathplanning::Path&, size_t,
         * double, double) except that parameters are read from the propertymap.
         *
         * @param path [in] Path to optimize
         * @return The optimized path
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
         * @param path [in] Path to optimize
         * @param cnt [in] Max count to use. If cnt=0, only the time limit will be used
         * @param time [in] Max time to use (in seconds). If time=0, only the cnt limit will be used
         * @param subDivideLength [in] The length into which the path is subdivided
         * @return The optimized path
         */
        rw::trajectory::QPath partialShortCut(
            const rw::trajectory::QPath& path,
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
         * @param path [in] Path to optimize
         * @return The optimized path
         */
        rw::trajectory::QPath partialShortCut(const rw::trajectory::QPath& path);

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
        rw::math::QMetricPtr _metric;

        rw::common::PropertyMap _propertyMap;

        void resamplePath(rw::trajectory::QPath& path, double subDivisionSize);

        rw::trajectory::QPath::iterator resample(
            rw::trajectory::QPath::iterator it1,
            rw::trajectory::QPath::iterator it2,
            double subDivisionSize,
            rw::trajectory::QPath& result);

        bool validPath(const rw::math::Q& from, const rw::math::Q& to);

        bool _testQStart;
        void setTestQStart(bool value) { _testQStart = value; }

        bool _testQEnd;
        void setTestQEnd(bool value) { _testQEnd = value; }
    };

    /** @} */

}} // end namespaces

#endif /*PATHLENGTHOPTIMIZER_HPP_*/
