#include "includes.h"
#include "globals.h"
#include "Shape.hpp"

class ShapeList {
public:
    /*
    Edge Constructor
    */
    ShapeList();
    
    ~ShapeList();
	
	// Declare public functions here:
	void addShape(ShapeType shape, float width, float length, float radius, float currentX, float currentY);
    
private:

	// Declare variables for use here:
	std::vector<Shape> shapes;
	
};
