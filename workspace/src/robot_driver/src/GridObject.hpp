#include "includes.h"
#include "globals.h"

class GridObject {
public:
	enum detectionType { 
		NO_EDGE,
		FALLING_EDGE,
		RISING_EDGE
	};	

    /*
    Edge Constructor
    */
    GridObject(float startDistance, float middleDistance, float endDistance, int startIndex, int middleIndex, int endIndex);
    
    ~GridObject();
	
	// Declare public functions here:
	
	float getCurrentX();
	float getCurrentY();
	bool getConstantGradient();
	float getWidth();
	float getLength();
	float getRadius();
	ShapeType getShapeType();
	
	float determinePose(float distanceA, float angleA, float distanceB, float angleB, float distanceC, float angleC);
	void determineLocation(float distanceA, float angleA, float distanceB, float angleB);
	void determineShapeSR(float distanceA, float angleA, float distanceB, float angleB, float increment);
	void determineShapeC(float distanceA, float distanceB, float distanceC, float increment, int index);
	void determineGlobalLocation(float robotX, float robotY, float robotYaw);
    
private:
    /*
    Private edge copy constructor - edge cannot be copied, must be passed around via pointers and references.
    */
    GridObject(const GridObject&){};
    
    /*
    Private edge copy assignment operator - edge cannot be copied, must be passed around via pointers and
    references.
    */
    GridObject& operator=(const GridObject&) {
        return *this;
    };
    
    float cross(Coord A, Coord C);
    float cosineRule(float distanceA, float distanceB, float angleC);

	// Declare variables for use here:
	int startIndex;
	int middleIndex;
	int endIndex;
	float startDistance;
	float middleDistance;
	float endDistance;
	bool constantGradient;
	
	float currentX;
	float currentY; 
	float width;
	float length;
	float radius;
    ShapeType shape;
};
