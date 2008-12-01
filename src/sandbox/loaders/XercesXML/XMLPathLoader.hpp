
#ifndef RW_LOADERS_PATHLOADER_HPP
#define RW_LOADERS_PATHLOADER_HPP


#include <rw/trajectory/Path.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <xercesc/dom/DOMElement.hpp>
#include <string>

namespace rw {
namespace loaders {

/**
 * @brief Enables loading in path file specified in the RobWork Path XML format.
 *
 * The XMLPathLoader loads in a file containing a path specified according to the rwxml_path.xsd schema.
 * The XML-file can be parsed either with or without schema verification. The schema can either be specified in the
 * XML-file or given as argument to the constructor.
 *
 * A path can contain either rw::math::Q, rw::math::Vector3D, rw::math::Rotation3D or rw::math::Transform3D elements.
 * If the type of the path in the file in unknown it can be determined using the XMLPathLoader::getType after loading.
 *
 * If reading in a path fails an exception is thrown
 */
class XMLPathLoader
{
public:
    /**
     * @brief Constructs XMLPathLoader and parser \b filename
     *
     * It is possible to specify whether to use the default schema which is the default behavior. If a
     * schema is specified in the XML-file or no schema should be used set \b useDefaultSchema to false.
     *
     * If reading in the path fails an exception is thrown
     *
     * @param filename [in] The file to load
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     */
    XMLPathLoader(const std::string& filename, const std::string& schemeFileName = "");

    /**
     * @brief Destructor
     */
    virtual ~XMLPathLoader();

    /**
     * @brief Enumeration specifying which type of trajectory, that has been loaded
     */
    enum Type { QType = 0,      /** @brief rw::trajectory::QPath */
                Vector3DType,   /** @brief rw::trajectory::Vector3DPath */
                Rotation3DType, /** @brief rw::trajectory::Rotation3DPath */
                Transform3DType /** @brief rw::trajectory::Transform3DPath */
                };

    /**
     * @brief Returns the type of the path loaded
     */
    Type getType();

    /**
     * @brief Returns path loaded
     *
     * If the loaded path is not of type QPath it throws an exception.
     *
     * @return Pointer to the path
     */
    rw::trajectory::QPathPtr getQPath();

    /**
     * @brief Returns path loaded
     *
     * If the loaded path is not of type Vector3DPath it throws an exception.
     *
     * @return Pointer to the path
     */
    rw::trajectory::Vector3DPathPtr getVector3DPath();

    /**
     * @brief Returns path loaded
     *
     * If the loaded path is not of type Rotation3DPath it throws an exception.
     *
     * @return Pointer to the path
     */
    rw::trajectory::Rotation3DPathPtr getRotation3DPath();

    /**
     * @brief Returns loaded path
     *
     * If the loaded path is not of type Transform3DPath it throws an exception.
     *
     * @return Pointer to the path
     */
    rw::trajectory::Transform3DPathPtr getTransform3DPath();



private:
    void readTrajectory(xercesc::DOMElement* element);

    rw::trajectory::QPathPtr _qPath;
    rw::trajectory::Vector3DPathPtr _v3dPath;
    rw::trajectory::Rotation3DPathPtr _r3dPath;
    rw::trajectory::Transform3DPathPtr _t3dPath;

    Type _type;
};

} //end namespace loaders
} //end namespace rw

#endif //enc include guard
