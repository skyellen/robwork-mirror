#ifndef __ANIMAL_HPP
#define __ANIMAL_HPP

#include <rw/common/Ptr.hpp>



namespace animals {

class Animal {
public:

	typedef rw::common::Ptr<Animal> Ptr;
	
public:

	virtual void makeSound() const = 0;

};

}

#endif
