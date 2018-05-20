#include "includes.h"
#include "globals.h"

class Shape {
public:
    /*
    Edge Constructor
    */
    Shape(float currentX, float currentY, ShapeType shape, float length, float width, float radius);
    
    ~Shape();
	
	// Declare public functions here:
	void setID(int id);
	int getID();
	
	float getCurrentX();
	float getCurrentY();
	float getLength();
	float getWidth();
	float getRadius();
	ShapeType getShape();
    
    void incrementDiffShapeCounter();
    int getDiffShapeCounter();
    void resetDiffShapeCounter();
    void printShape();
    
private:

	// Declare variables for use here:
	int shapeID;
	float currentX;
	float currentY; 
	float length;
	float width;
	float radius;
	int diffShapeCounter;
	ShapeType shape;
};
