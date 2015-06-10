#include "Cat.hpp"

#include <iostream>



using namespace animals;

using namespace std;



Cat::Cat() {
}


Cat::~Cat() {
}


void Cat::makeSound() const {
	cout << "meow" << endl;
}
