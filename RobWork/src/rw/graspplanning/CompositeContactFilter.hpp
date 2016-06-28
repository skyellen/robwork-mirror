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

#ifndef RW_GRASPPLANNING_COMPOSITECONTACTFILTER_HPP_
#define RW_GRASPPLANNING_COMPOSITECONTACTFILTER_HPP_

#include "ContactValidateFilter.hpp"

#include <vector>

namespace rw {
namespace graspplanning {

/**
 * @brief makes it possible to combine several contact filters into one contact filter.
 * Statistics are maintained of the validation succes which can be queried and analysed.
 */
class CompositeContactFilter : public ContactValidateFilter {
public:

    /**
     * @brief Cconstructor
     */
    CompositeContactFilter(bool enableFullStats=false);

    /**
     * @brief destructor
     */
    virtual ~CompositeContactFilter(){};

    /**
     * @copydoc ContactValidateFilter::isValid
     */
    bool isValid(const rw::sensor::Contact3D& contact);

    /**
     * @brief add contact validation filter
     * @param filter
     */
    void addFilter(ContactValidateFilter* filter);

    /**
     * @brief get a vector of all contact filters
     * @return
     */
    std::vector<ContactValidateFilter*> getFilters(){return _cfilters;};

    /**
     * @brief set all contact filters
     */
    void setFilters(std::vector<ContactValidateFilter*> filters);

private:
    std::vector<ContactValidateFilter*> _cfilters;
    // the number of contacts that was invalid by the filter indicated by the index
    std::vector<int> _stats;
    int _nrOfTests;
    bool _fullTestEnabled;
};

}
}

#endif /* PLANECLEARANCEFILTER_HPP_ */
