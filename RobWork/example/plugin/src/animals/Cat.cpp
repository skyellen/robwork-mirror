#include "Cat.hpp"

#include <iostream>



using animals::Cat;

using namespace std;



Cat::Cat() {
}


Cat::~Cat() {
}


void Cat::makeSound() const {
	cout << "meow" << endl;
}
