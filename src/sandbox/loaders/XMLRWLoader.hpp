/*
 * XMLRWLoader.hpp
 *
 *  Created on: 27-07-2008
 *      Author: jimali
 */

#ifndef XMLRWLOADER_HPP_
#define XMLRWLOADER_HPP_

#include <rw/models/WorkCell.hpp>

class XMLRWLoader {
public:
    static std::auto_ptr<rw::models::WorkCell> load(const std::string& file);

};


#endif /* XMLRWLOADER_HPP_ */
