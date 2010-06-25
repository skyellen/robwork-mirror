/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#ifndef COMPOSITEGRASPFILTER_HPP_
#define COMPOSITEGRASPFILTER_HPP_

#include <rw/math/Transform3D.hpp>
#include "Grasp3D.hpp"
#include "GraspValidateFilter.hpp"

namespace rw {
namespace graspplanning {

/**
 * @brief makes it possible to combine several contact filters into one contact filter.
 * Statistics are maintained of the validation succes which can be queried and analysed.
 */
class CompositeGraspFilter : public GraspValidateFilter {
public:

    /**
     * Constructor
     */
    CompositeGraspFilter(bool enableFullStats=false);

    /**
     * @brief destructor
     */
    virtual ~CompositeGraspFilter(){};

    /**
     * @copydoc ContactValidateFilter::isValid
     */
    bool isValid(const Grasp3D& contact);

    void addFilter(GraspValidateFilter* filter);

    std::vector<GraspValidateFilter*> getFilters(){return _gfilters;};

    void setFilters(std::vector<GraspValidateFilter*> filters);

private:
    std::vector<GraspValidateFilter*> _gfilters;
    // the number of contacts that was invalid by the filter indicated by the index
    std::vector<int> _stats;
    int _nrOfTests;
    bool _fullTestEnabled;
};
}
}
#endif /* PLANECLEARANCEFILTER_HPP_ */
