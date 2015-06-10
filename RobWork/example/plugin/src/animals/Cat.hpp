#ifndef __CAT_HPP
#define __CAT_HPP

#include "Animal.hpp"

namespace animals {

class Cat: public Animal {
public:

	Cat();
	virtual ~Cat();

	/**
	 * "Meow!"
	 */
	virtual void makeSound() const;

};

}

#endif
