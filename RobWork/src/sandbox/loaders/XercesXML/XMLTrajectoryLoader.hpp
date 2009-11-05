
#ifndef RW_LOADERS_TRAJECTORYLOADER_HPP
#define RW_LOADERS_TRAJECTORYLOADER_HPP


#include <rw/common/Ptr.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <xercesc/dom/DOMElement.hpp>
#include <string>

namespace rw {
namespace loaders {



/**
 * @brief Enables loading in trajectories file specified in the RobWork Trajectory XML format.
 *
 * The XMLTrajectoryLoader loads in a file containing a trajectory specified according to the rwxml_trajectory.xsd schema.
 * The XML-file can be parsed either with or without schema verification. The schema can either be specified in the
 * XML-file or given as argument to the constructor.
 *
 * A trajectory can contain either rw::math::Q, rw::math::Vector3D, rw::math::Rotation3D or rw::math::Transform3D elements.
 * If the type of the trajectory in the file in unknown it can be determined using the XMLTrajectoryLoader::getType after loading.
 *
 * If reading in a trajectory fails an exception is thrown
 */
class XMLTrajectoryLoader
{
public:
    /**
     * @brief Constructs XMLTrajectoryLoader and parser \b filename
     *
     * It is possible to specify whether to use the default schema which is the default behavior. If a
     * schema is specified in the XML-file or no schema should be used set \b useDefaultSchema to false.
     *
     * If reading in the trajectory fails an exception is thrown
     *
     * @param filename [in] The file to load
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     */
    XMLTrajectoryLoader(const std::string& filename, const std::string& schemaFileName = "");

    /**
     * @brief Destructor
     */
    virtual ~XMLTrajectoryLoader();

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
    rw::trajectory::QTrajectoryPtr getQTrajectory();

    /**
     * @brief Returns trajectory with template type rw::math::Vector3D<>
     *
     * If the loaded trajectory does not contain this type an exception is thrown.
     *
     * @return Copy of trajectory
     */
    rw::trajectory::Vector3DTrajectoryPtr getVector3DTrajectory();

    /**
     * @brief Returns trajectory with template type rw::math::Rotation3D<>
     *
     * If the loaded path is not of type Transform3DPath it throws an exception.
     *
     * @return Copy of trajectory
     */
    rw::trajectory::Rotation3DTrajectoryPtr getRotation3DTrajectory();

    /**
     * @brief Returns trajectory with template type rw::math::Transform3D<>
     *
     * If the loaded path is not of type Transform3DPath it throws an exception.
     *
     * @return Copy of trajectory
     */
    rw::trajectory::Transform3DTrajectoryPtr getTransform3DTrajectory();


private:
    void readTrajectory(xercesc::DOMElement* element);

    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Q> > _qTrajectory;
    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Vector3D<> > > _v3dTrajectory;
    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Rotation3D<> > > _r3dTrajectory;
    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Transform3D<> > > _t3dTrajectory;

    Type _type;
};

} //end namespace loaders
} //end namespace rw

#endif //enc include guard
