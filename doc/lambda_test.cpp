#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>

class Animal 
{
	int x_value;
 public:
	int y_value;
	int x = 42;
};

class Dog : public Animal 
{};

int main() {
	
	Dog waldi;
	waldi.y_value = 33;
	std::vector<std::unique_ptr<Dog>> vec;
	/*
	for(int i = 0; i<10; i++) {
		//std::unique_ptr<Dog> doggy;
		//doggy.y_value = i*2;
		vec.push_back(std::make_unique<Dog>());
		}
	*/
	
	auto wuffi = std::make_unique<Dog>();
	wuffi->y_value = 11;
	
	auto waldo = std::make_unique<Dog>();
	waldo->y_value = 33;
	
	auto frodo = std::make_unique<Dog>();
	waldo->y_value = 22;
	
	vec.push_back(std::move(wuffi));
	vec.push_back(std::move(frodo));
	vec.push_back(std::move(waldo));
	
	std::cout << vec.size() << '\n';
	std::cout << vec.back()->y_value << '\n';
	
	auto it = std::max_element(vec.begin(), vec.end(), 
    [](std::unique_ptr<Dog>& x, std::unique_ptr<Dog>& y) {  return x->y_value < y->y_value; });
    
	std::cout << (*it)->y_value << '\n';
	int foobar = it-vec.begin();
    
	std::cout << foobar << '\n';

	return EXIT_SUCCESS;
}

