#ifndef DYNAMICWORKCELLLOADER_HPP_
#define DYNAMICWORKCELLLOADER_HPP_

#include <dynamics/DynamicWorkcell.hpp>

namespace loaders {


    class DynamicWorkCellLoader
    {
    public:

        static rw::common::Ptr<dynamics::DynamicWorkcell>
        	load(const std::string& filename);

    };


}

#endif /*DYNAMICWORKCELLLOADER_HPP_*/
