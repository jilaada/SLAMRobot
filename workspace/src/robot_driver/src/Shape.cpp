#include "Shape.hpp"

using namespace std;

Shape::Shape(float currentX, float currentY, ShapeType shape, float length, float width, float radius) 
			: currentX(currentX), currentY(currentY), shape(shape), length(length), width(width), radius(radius) {	
			
}

// Write functions here:
void Shape::setID(int id) {
	this->shapeID = id;
}

int Shape::getID() {
	return this->shapeID;
}

float Shape::getCurrentX() {
	return currentX;
}

float Shape::getCurrentY() {
	return currentY;
}

float Shape::getLength() {
	return length;
}

float Shape::getWidth() {
	return width;
}

float Shape::getRadius() {
	return radius;
}

ShapeType Shape::getShape() {
	return shape;
}

int Shape::getDiffShapeCounter() {
	return diffShapeCounter;
}

void Shape::incrementDiffShapeCounter() {
	this->diffShapeCounter++;
}

void Shape::resetDiffShapeCounter() {
	this->diffShapeCounter = 0;
}

void Shape::updateLocation(float currentX, float currentY) {
	// Running average
	this->currentX = (this->currentX + currentX)/2.0;
	this->currentY = (this->currentY + currentY)/2.0;
}

void Shape::printShape() {
	cout << "ID: " << this->shapeID << "\n";
	if (this->shape == CIRCLE) {
		// Print circle characteristics
		cout << "Found a circle     " << "Radius: " << this->radius << "\n";
	} else if (this->shape == RECTANGLE) {
		// Print 
		cout << "Found a rectangle! " << "x: " << this->width << " : " << "y: " << this->length << "\n";
	} else if (this->shape == SQUARE) {
		// Print
		cout << "Found a square!    " << "x: " << this->width << " : " << "y: " << this->length << "\n";
	} else {
		cout << "This shape should not be here: " << this->shape << " : " << this->length << " : " << this->width << " : " << this->radius << "\n";
	}
	cout << "Location x: " << this->currentX << "\n";
	cout << "Location y: " << this->currentY << "\n";
}    

Shape::~Shape() {

}
