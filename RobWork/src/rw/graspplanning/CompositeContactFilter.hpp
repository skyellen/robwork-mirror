/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#ifndef COMPOSITECONTACTFILTER_HPP_
#define COMPOSITECONTACTFILTER_HPP_

#include <rw/math/Transform3D.hpp>

#include "ContactValidateFilter.hpp"


namespace rw {
namespace graspplanning {

/**
 * @brief makes it possible to combine several contact filters into one contact filter.
 * Statistics are maintained of the validation succes which can be queried and analysed.
 */
class CompositeContactFilter : public ContactValidateFilter {
public:

    /**
     * Constructor
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

    void addFilter(ContactValidateFilter* filter);

    std::vector<ContactValidateFilter*> getFilters(){return _cfilters;};

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
