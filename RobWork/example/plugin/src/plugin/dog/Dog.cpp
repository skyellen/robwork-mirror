#include "Dog.hpp"

#include <iostream>



using namespace std;
using namespace animals;



Dog::Dog() {
}


Dog::~Dog() {
}


void Dog::makeSound() const {
	cout << "woof" << endl;
}
