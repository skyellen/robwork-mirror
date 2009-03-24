#ifndef RW_LOADERS_XMLTASKFORMAT_HPP
#define RW_LOADERS_XMLTASKFORMAT_HPP

#include <rw/loaders/xml/XercesUtils.hpp>

namespace rw {

namespace loaders {

/**
 * @brief Class storing the identifiers used for paths in the XML Task Format
 */
class XMLTaskFormat
{
private:
    static const XercesInitializer initializer;
public:

    /** @brief Identifier for rw::task::Task with rw::math::Q as content in the XML format  */
    static const XMLCh* QTaskId;

    static const XMLCh* CartesianTaskId;

    static const XMLCh* TargetsId;

    static const XMLCh* EntitiesId;

    static const XMLCh* AugmentationsId;


    static const XMLCh* QTargetId;

    static const XMLCh* CartesianTargetId;


    static const XMLCh* MotionId;
    static const XMLCh* ActionId;




    static const XMLCh* EntityIndexId;
    static const XMLCh* EntityIdId;

    static const XMLCh* TargetIdAttrId;

    static const XMLCh* MotionTypeAttrId;



    static const XMLCh* MotionStartId;
    static const XMLCh* MotionMidId;
    static const XMLCh* MotionEndId;

    static const XMLCh* LinearMotionId;
    static const XMLCh* P2PMotionId;
    static const XMLCh* CircularMotionId;


    static const XMLCh* ActionTypeAttrId;

private:
	XMLTaskFormat() {};
};

} //end namespace loaders
} //end namespace rw

#endif //End include guard
