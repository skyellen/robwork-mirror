
#include "Pose2D.hpp"
#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::common;
using namespace rw::math;

// some explicit template specifications
template class Pose2D<double>;
template class Pose2D<float>;

namespace rw{ namespace common {namespace serialization {

    template<class T>
    void writeImpl(const Pose2D<T>& tmp, OutputArchive& oar, const std::string& id){
        oar.writeEnterScope(id);
        oar.write( tmp.getPos() , "pos" );
        oar.write( tmp.theta() , "theta" );
        oar.writeLeaveScope(id);
    }

    template<class T>
    void readImpl(Pose2D<T>& tmp, InputArchive& iar, const std::string& id){
        Vector2D<T> pos;
        T theta;
        iar.readEnterScope(id);
        iar.read( pos , "pos" );
        iar.read( theta , "theta" );
        iar.readLeaveScope(id);
        tmp = Pose2D<T>(pos,theta);
    }

    // some explicit template specifications
    template<> void write(const Pose2D<double>& tmp, OutputArchive& oar, const std::string& id) { writeImpl(tmp,oar,id); }
    template<> void write(const Pose2D<float>& tmp, OutputArchive& oar, const std::string& id) { writeImpl(tmp,oar,id); }
    template<> void read(Pose2D<double>& tmp, InputArchive& iar, const std::string& id) { readImpl(tmp,iar,id); }
    template<> void read(Pose2D<float>& tmp, InputArchive& iar, const std::string& id) { readImpl(tmp,iar,id); }

}}} // end namespaces

