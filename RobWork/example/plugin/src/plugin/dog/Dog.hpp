#ifndef __DOG_HPP
#define __DOG_HPP

#include <animals/Animal.hpp>



namespace animals {

/**
 * Additional type of Animal, added in the plugin.
 */
class Dog: public Animal {
public:

	Dog();
	virtual ~Dog();

	/**
	 * "Woof!"
	 */
	virtual void makeSound() const;

};

}

#endif
