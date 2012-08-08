#include "serializeutil.hpp"

#include <rw/math/Q.hpp>
#include "InputArchive.hpp"
#include "OutputArchive.hpp"

namespace serialization {

    void save(const rw::math::Q& tmp, OutputArchive& oar, const std::string& id){
        oar.write(tmp.size(), "size");
        for(int i=0;i<tmp.size();i++){
            oar.write(tmp[i],"q");
        }
    }

    rw::math::Q* load(rw::math::Q* tmp, InputArchive& iar, const std::string& id){
        int size = iar.readInt("size");
        rw::math::Q *q = new rw::math::Q(size);
        for(int i=0;i<size;i++){
            (*q)[i] = iar.readDouble("q");
        }
        return q;
    }

}
