
#include "Pose2D.hpp"
#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::math;

// some explicit template specifications
template class Pose2D<double>;
template class Pose2D<float>;


namespace rw{ namespace common {namespace serialization {

    template<class T>
    void write(const rw::math::Pose2D<T>& tmp, rw::common::OutputArchive& oar, const std::string& id){
        oar.writeEnterScope(id);
        oar.write( tmp.getPos() , "pos" );
        oar.write( tmp.theta() , "theta" );
        oar.writeLeaveScope(id);
    }

    template<class T>
    void read(rw::math::Pose2D<T>& tmp, rw::common::InputArchive& iar, const std::string& id){
        rw::math::Vector2D<T> pos;
        T theta;
        iar.readEnterScope(id);
        iar.read( pos , "pos" );
        iar.read( theta , "theta" );
        iar.readLeaveScope(id);
        tmp = rw::math::Pose2D<T>(pos,theta);
    }

    // some explicit template specifications
    template void write(const rw::math::Pose2D<double>& tmp, rw::common::OutputArchive& oar, const std::string& id);
    template void write(const rw::math::Pose2D<float>& tmp, rw::common::OutputArchive& oar, const std::string& id);
    template void read(rw::math::Pose2D<double>& tmp, rw::common::InputArchive& iar, const std::string& id);
    template void read(rw::math::Pose2D<float>& tmp, rw::common::InputArchive& iar, const std::string& id);

}}} // end namespaces

