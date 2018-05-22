#include "GridObject.hpp"

using namespace std;
// TODO: complete the constructoredge
GridObject::GridObject(float startDistance, float middleDistance, float endDistance, int startIndex, int middleIndex, int endIndex) 
			: startDistance(startDistance), middleDistance(middleDistance), endDistance(endDistance), startIndex(startIndex), middleIndex(middleIndex), endIndex(endIndex) {	
	this->currentX = 0;
	this->currentY = 0;
}

// Write functions here:

float GridObject::getCurrentX() {
	return currentX;
}

float GridObject::getCurrentY() {
	return currentY;
}

bool GridObject::getConstantGradient() {
	return constantGradient;
}

float GridObject::getWidth() {
	return this->width;
}

float GridObject::getLength() {
	return this->length;
}

float GridObject::getRadius() {
	return this->radius;
}

ShapeType GridObject::getShapeType() {
	return this->shape;
}

void GridObject::setDefaultShapeType() {
	this->shape = MORE_INFO;
}

float GridObject::determinePose(float distanceA, float angleA, float distanceB, float angleB, float distanceC, float angleC) {
	// Current position is a global so should be ok
	float xA = distanceA * cos(angleA);
	float yA = abs(distanceA * sin(angleA));
	float xB = distanceB * cos(angleB);
	float yB = abs(distanceB * sin(angleB));
	float xC = distanceC * cos(angleC);
	float yC = abs(distanceC * sin(angleC));
	
	Coord A;
	Coord C;
	
	A.x = xA - xB;
	A.y = -(abs(yA-yB));
	C.x = xC - xB;
	C.y = abs(yC-yB);
	
	if (((yA-yB)/abs(xA-xB) < ((yB-yC)/abs(xB-xC))+TOLERANCE) && ((yA-yB)/abs(xA-xB) > ((yB-yC)/abs(xB-xC))-TOLERANCE)) {
		// Constant gradient detected
		constantGradient = true;
	} else {
		constantGradient = false;
	}

	return abs(cross(A,C));
}

void GridObject::determineShapeSR(float distanceA, float angleA, float distanceB, float angleB, float increment) {
	width = cosineRule(this->startDistance, this->middleDistance, (this->middleIndex-this->startIndex)*increment);
	length = cosineRule(this->endDistance, this->middleDistance, (this->endIndex-this->middleIndex)*increment);
		
	if ((width > length-TOLERANCE) && (width < length+TOLERANCE)) {
		// Equal lengths - this is square
		//cout << "Found a square!    " << "x: " << width << " : " << "y: " << length << "\n";
		shape = SQUARE;
	} else {
		//cout << "Found a rectangle! " << "x: " << width << " : " << "y: " << length << "\n";
		shape = RECTANGLE;
	}
	determineLocation(distanceA, angleA, distanceB, angleB);
}

void GridObject::determineShapeC(float distanceA, float distanceB, float distanceC, float increment, int index) {
	// Determine the constantGradient parameter
	float ignoreValue = determinePose(distanceA, (-1.5708 + (index-RANGE)*increment), distanceB, (-1.5708 + index*increment), distanceC, (-1.5708 + (index+RANGE)*increment));

	if (this->constantGradient) {
		// Potential for vertex of square OR rectangle therefore unsure
		shape = MORE_INFO;
	} else {
		radius = cosineRule(this->startDistance, this->endDistance, (this->endIndex-this->startIndex)*increment)/2;
		shape = CIRCLE;
		//cout << "Found a circle     " << "Radius: " << radius << "\n";
		determineLocation(this->startDistance, (1.5708-(this->startIndex)*increment), this->endDistance, (1.5708-(this->endIndex)*increment));
	}
}

void GridObject::determineLocation(float distanceA, float angleA, float distanceB, float angleB) {
	float xA = distanceA * cos(angleA);
	float yA = distanceA * sin(angleA);
	float xB = distanceB * cos(angleB);
	float yB = distanceB * sin(angleB);
	
	float midX;
	
	if (xA > xB) {
		midX = (xA-xB)/2 + xB + LASER_POSE;
	} else if (xB > xA) {
		midX = (xB-xA)/2 + xA + LASER_POSE;
	}
	
	float midY = (yB - yA)/2 + yA;
	
	this->currentX = midX;
	this->currentY = midY;
}

void GridObject::determineGlobalLocation(float robotX, float robotY, float robotYaw) {
	float tempX = cos(robotYaw)*this->currentX - sin(robotYaw)*this->currentY;
	float tempY = cos(robotYaw)*this->currentY + sin(robotYaw)*this->currentX;
	
	this->currentX = tempX + robotX;
	this->currentY = tempY - robotY;
	
	//cout << "Location x: " << this->currentX << "\n";
	//cout << "Location y: " << this->currentY << "\n";
}

float GridObject::cross(Coord A, Coord C) {
	float dot = A.x*C.x + A.y*C.y;
	float det = A.y*C.x - A.x*C.y;
	return atan2(det, dot);
}

float GridObject::cosineRule(float distanceA, float distanceB, float angleC) { 
	return sqrt(pow(distanceA, 2.0) + pow(distanceB, 2.0) - 2*distanceA*distanceB*cos(angleC));
}


GridObject::~GridObject() {

}
