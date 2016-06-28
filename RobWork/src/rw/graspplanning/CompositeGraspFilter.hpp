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

#ifndef RW_GRASPPLANNING_COMPOSITEGRASPFILTER_HPP_
#define RW_GRASPPLANNING_COMPOSITEGRASPFILTER_HPP_

#include "GraspValidateFilter.hpp"

#include <vector>

namespace rw {
namespace graspplanning {

/**
 * @brief makes it possible to combine several contact filters into one contact filter.
 * Statistics are maintained of the validation succes which can be queried and analyzed.
 */
class CompositeGraspFilter : public GraspValidateFilter {
public:

    /**
     * @brief Constructor
     */
    CompositeGraspFilter();

    /**
     * @brief destructor
     */
    virtual ~CompositeGraspFilter(){};

    /**
     * @copydoc ContactValidateFilter::isValid
     */
    bool isValid(const Grasp3D& contact);

    /**
     * @brief add a composition to this filter
     * @param filter
     */
    void addFilter(GraspValidateFilter* filter);

    /**
     * @brief get all filters of this filter
     * @return list of GraspValidateFilter
     */
    std::vector<GraspValidateFilter*> getFilters(){return _gfilters;};

    /**
     * @brief set the list of grasp validate filters
     * @param filters
     */
    void setFilters(std::vector<GraspValidateFilter*> filters);

    /**
     * @brief clear the statistics
     */
    void clearStats();

    /**
     * @brief get statistics
     *
     * the number of contacts that was invalid by the filter indicated by the index
     */
    std::vector<int>& getStats(){return _stats;};

private:
    std::vector<GraspValidateFilter*> _gfilters;
    // the number of contacts that was invalid by the filter indicated by the index
    std::vector<int> _stats;
    int _nrOfTests;
    //bool _fullTestEnabled;
};
}
}
#endif /* PLANECLEARANCEFILTER_HPP_ */
