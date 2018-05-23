#include "ShapeList.hpp"

using namespace std;

ShapeList::ShapeList() {	
	
}

// Write functions here: 
int ShapeList::getShapeListSize() {
	return shapes.size();
}

float ShapeList::getShapeCurrentX(int element) {
	return shapes.at(element).getCurrentX();
}

float ShapeList::getShapeCurrentY(int element) {
	return shapes.at(element).getCurrentY();
}

void ShapeList::removeElement(int element) {
	shapes.erase(shapes.begin() + element);
	//Potentially may need to change the ID numbers
}

void ShapeList::addShape(ShapeType shape, float width, float length, float radius, float currentX, float currentY) {
	// All the components required to create a shape
	Shape newShape = Shape(currentX, currentY, shape, length, width, radius);
	if ((shape == CIRCLE) || (shape == RECTANGLE) || (shape == SQUARE)) {
		// Something in the vector therefore interate through
		for(std::vector<Shape>::iterator itm = shapes.begin(); itm != shapes.end(); ++itm) {
			// Determine which measurement to increment the changes by
			float smallestMeasurement = 0;
			if (itm->getShape() == CIRCLE) {
				// Determine threshold for circular shapes
				smallestMeasurement = itm->getRadius() + LOCATION_TOLERANCE + ROBOT_SIZE;
			} else {
				// Determine if the current shape location is within the smallest dimension of an existing shape
				smallestMeasurement = (itm->getWidth() < itm->getLength() ? itm->getWidth() : itm->getLength()) + LOCATION_TOLERANCE + ROBOT_SIZE;			
			}
			
			if (((currentX < itm->getCurrentX()+smallestMeasurement) && (currentX > itm->getCurrentX()-smallestMeasurement)) && ((currentY < itm->getCurrentY()+smallestMeasurement) && (currentY > itm->getCurrentY()-smallestMeasurement))) {
				// Object is in the same region
				if (shape != itm->getShape()) {
					itm->incrementDiffShapeCounter();
					// Determine if the resulting increment means that the new shape could be the actual shape
					if (itm->getDiffShapeCounter() == SWITCH_THRESHOLD) {
						// Switch the shapes
						cout << "Switched shapes\n";
						newShape.setID(itm->getID()); // Set to the new ID
						shapes.insert(itm, newShape); // Insert the new shape to the position in the array
						shapes.erase(itm+1);	// Erase the old shape
						// Print new information
					}
					return;
				} else {
					// Update the location (running average)
					itm->updateLocation(currentX, currentY);
					itm->resetDiffShapeCounter();
					return;
				}
			}
		} 
		// If we make it out here it is a new shape
		// Add the shape into the vector
		newShape.setID(shapes.size());
		shapes.push_back(newShape);
		// Print characteristics
	}
	return;
}

void ShapeList::printShapes() {
	for(std::vector<Shape>::iterator itm = shapes.begin(); itm != shapes.end(); ++itm) {
		itm->printShape();
	}
}

ShapeList::~ShapeList() {

}
