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

#ifndef RW_LOADERS_DOMTRAJECTORYLOADER_HPP
#define RW_LOADERS_DOMTRAJECTORYLOADER_HPP


#include <rw/common/Ptr.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <string>

namespace rw { namespace common { class DOMElem; } }

namespace rw {
namespace loaders {


/** @addtogroup loaders */
/*@{*/


/**
 * @brief Enables loading in trajectories file specified in the RobWork Trajectory XML format.
 *
 * The DOMTrajectoryLoader loads in a file containing a trajectory specified according to the rwxml_trajectory.xsd schema.
 * The XML-file can be parsed either with or without schema verification. The schema can either be specified in the
 * XML-file or given as argument to the constructor.
 *
 * A trajectory can contain either rw::math::Q, rw::math::Vector3D, rw::math::Rotation3D or rw::math::Transform3D elements.
 * If the type of the trajectory in the file in unknown it can be determined using the DOMTrajectoryLoader::getType after loading.
 *
 * If reading in a trajectory fails an exception is thrown
 */
class DOMTrajectoryLoader
{
public:
    /**
     * @brief Constructs DOMTrajectoryLoader and parser \b filename
     *
     * It is possible to specify whether to use the default schema which is the default behavior. If a
     * schema is specified in the XML-file or no schema should be used set \b useDefaultSchema to false.
     *
     * If reading in the trajectory fails an exception is thrown
     *
     * @param filename [in] The file to load
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     */
    DOMTrajectoryLoader(const std::string& filename, const std::string& schemaFileName = "");


    /**
     * @brief Constract DOMTrajectoryLoader and parser input from \b instream
     *
     * It is possible to specify whether to use the default schema which is the default behavior. If a
     * schema is specified in the XML-file or no schema should be used set \b useDefaultSchema to false.
     *
     * If reading in the trajectory fails an exception is thrown
     *
     * @param instream [in] The istream to read from
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     */
    DOMTrajectoryLoader(std::istream& instream, const std::string& schemaFileName = "");


    /**
     * @brief Destructor
     */
    virtual ~DOMTrajectoryLoader();

    /**
     * @brief Enumeration specifying which type of trajectory, that has been loaded
     */
    enum Type { QType = 0,      /** @brief rw::trajectory::Trajectory<Q> */
                Vector3DType,   /** @brief rw::trajectory::Trajectory<Vector3D> */
                Rotation3DType, /** @brief rw::trajectory::Trajectory<Rotation3D> */
                Transform3DType /** @brief rw::trajectory::Trajectory<Transform3D> */
                };

    /**
     * @brief Returns the type of the trajectory loaded
     */
    Type getType();

    /**
     * @brief Returns trajectory with template type rw::math::Q.
     *
     * If the loaded path is not of type Transform3DPath it throws an exception.
     *
     * @return Copy of trajectory
     */
	rw::trajectory::QTrajectory::Ptr getQTrajectory();

    /**
     * @brief Returns trajectory with template type rw::math::Vector3D<>
     *
     * If the loaded trajectory does not contain this type an exception is thrown.
     *
     * @return Copy of trajectory
     */
	rw::trajectory::Vector3DTrajectory::Ptr getVector3DTrajectory();

    /**
     * @brief Returns trajectory with template type rw::math::Rotation3D<>
     *
     * If the loaded path is not of type Transform3DPath it throws an exception.
     *
     * @return Copy of trajectory
     */
	rw::trajectory::Rotation3DTrajectory::Ptr getRotation3DTrajectory();

    /**
     * @brief Returns trajectory with template type rw::math::Transform3D<>
     *
     * If the loaded path is not of type Transform3DPath it throws an exception.
     *
     * @return Copy of trajectory
     */
	rw::trajectory::Transform3DTrajectory::Ptr getTransform3DTrajectory();
public:
public:
    /** @brief Identifier for rw::trajectory::Trajectory<rw::math::Q> in the XML format  */
    static const std::string QTrajectoryId;

    /** @brief Identifier for rw::trajectory::Trajectory<rw::math::Vector3D> in the XML format  */
    static const std::string V3DTrajectoryId;

    /** @brief Identifier for rw::trajectory::Trajectory<rw::math::Rotation3D> in the XML format  */
    static const std::string R3DTrajectoryId;

    /** @brief Identifier for rw::trajectory::Trajectory<rw::math::Transform3D> in the XML format  */
    static const std::string T3DTrajectoryId;

    /** @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Q> in the XML format  */
    static const std::string QLinearInterpolatorId;

    /** @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Q> in the XML format  */
    static const std::string QCubicSplineInterpolatorId;

    /** @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Vector3D<> > in the XML format  */
    static const std::string V3DLinearInterpolatorId;

    /** @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Vector3D<> > in the XML format  */
    static const std::string V3DCubicSplineInterpolatorId;

    /** @brief Identifier for rw::trajectory::CircularInterpolator<rw::math::Vector3D<> > in the XML format  */
    static const std::string V3DCircularInterpolatorId;

    /** @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Rotation3D<> > in the XML format  */
    static const std::string R3DLinearInterpolatorId;

    /** @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Rotation3D<> > in the XML format  */
    static const std::string R3DCubicSplineInterpolatorId;

    /** @brief Identifier for rw::trajectory::LinearInterpolator<rw::math::Transform3D<> > in the XML format  */
    static const std::string T3DLinearInterpolatorId;

    /** @brief Identifier for rw::trajectory::CubicSplineInterpolator<rw::math::Transform3D<> > in the XML format  */
    static const std::string T3DCubicSplineInterpolatorId;

    /** @brief Identifier for rw::trajectory::ParabolicBlend in the XML format  */
    static const std::string ParabolicBlendId;

    /** @brief Identifier for rw::trajectory::LloydHaywardblend in the XML format  */
    static const std::string LloydHaywardBlendId;

    /** @brief Identifier for duration specification for interpolators  */
    static const std::string DurationAttributeId;

    /** @brief Identifier for duration specification for interpolators  */
    static const std::string StartTimeAttributeId;

    /** @brief Identifier for the blend time tau used for blends  */
    static const std::string TauAttributeId;

    /** @brief Identifier for the parameter kappa used in LloydHayward blends  */
    static const std::string KappaAttributeId;

private:

    void readTrajectory(rw::common::Ptr<rw::common::DOMElem> element);

    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Q> > _qTrajectory;
    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Vector3D<> > > _v3dTrajectory;
    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Rotation3D<> > > _r3dTrajectory;
    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Transform3D<> > > _t3dTrajectory;

    Type _type;
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //enc include guard
