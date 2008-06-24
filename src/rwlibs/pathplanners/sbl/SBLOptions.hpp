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

#ifndef RWLIBS_PATHPLANNERS_SBL_SBLOPTIONS_HPP
#define RWLIBS_PATHPLANNERS_SBL_SBLOPTIONS_HPP

/**
   @file SBLOptions.hpp

   Options for SBL path planner.
*/

namespace rwlibs { namespace pathplanners {

    /**
       Options for the SBL path planner.

       These are for internal use mostly, and will likely change over time.
    */
    class SBLOptions
    {
    public:
        enum ExpandMode {
            UniformBox
        };

        enum NearNodeSelection {
            UniformSelect,
            UniformFromCell,
            NearestFromCell,
            NearestNode
        };

        double connectRadius;
        double extendRadius;
        int resetCount;
        int rootSampleInterval;
        double nodesPerCell;
        bool useArrayMap;
        ExpandMode expandMode;
        NearNodeSelection nearNodeSelection;

        SBLOptions()
        {
            connectRadius = 0.2;
            extendRadius = 0.2;
            resetCount = 200;
            rootSampleInterval = 25;
            nodesPerCell = 10;
            useArrayMap = false;
            expandMode = UniformBox;
            nearNodeSelection = NearestNode;
        }
    };

}} // end namespaces

#endif // end include guard
